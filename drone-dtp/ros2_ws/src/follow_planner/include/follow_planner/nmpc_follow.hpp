// The solver behind follow_planner's `goal_mode: nmpc`. See docs/fact-nmpc-follow-goal.md.
//
// WHAT IS OPTIMISED, AND WHY IT IS NOT THE AIRCRAFT
// -------------------------------------------------
// The decision variables are the GOAL's positions over a short horizon, g_1 .. g_N, continuing
// from the goal that was last published (g_0). They are not the aircraft's positions. A horizon
// of aircraft positions is speed-limited, so its end sits at most max_vel * T ~ 1.2 m ahead, and
// publishing that hands SUPER a short rest-to-rest hop -- the 2026-09-15 follow flight (median
// 0.36 m/s) by construction. SUPER already models the aircraft, with corridors and a map. What it
// lacked was a goal with a memory: the geometric standoff is re-decided from nothing every tick.
//
// So the "model" is a double integrator on the GOAL. Its velocity and acceleration are finite
// differences of the decision variables, which makes the dynamics implicit (no equality
// constraints, no shooting) and the problem unconstrained -- the same structure SUPER's own
// trajectory optimiser has, and what lets an L-BFGS solve it. Bounding them is what makes the
// published goal a continuous signal by construction instead of by luck.
//
// EVERYTHING HERE IS A PENALTY. The standoff floor, the Z range and the fence appear as costs so
// the solver rarely proposes a goal that breaks them; none of them is a guarantee. The guarantee
// is FollowPlanner::ApplyHardGuards, which runs on this solver's output exactly as it runs on
// the geometric goal.
//
// No rclcpp, no clock but steady_clock, no logging: the node owns all of that. That is what lets
// test/test_nmpc_follow.cpp check the gradient, which nothing downstream would ever catch -- a
// wrong gradient still converges, to the wrong place.
#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include "follow_planner/thirdparty/lbfgs.hpp"

namespace follow_planner {
namespace nmpc {

    // Residuals are normalised to O(1) at a nominal violation and the stage sum is divided by N,
    // so these survive a change of horizon or of dt. The reasons for the relative sizes are in the
    // fact doc; the short version is that altitude is the largest shaping weight because goal
    // height jumps were the measured defect, and floor / z-limit / fence are an order above
    // everything else because they shadow hard guards.
    struct Weights {
        double standoff{4.0};
        double bearing{1.0};
        double alt{8.0};
        double floor{500.0};
        double zlim{200.0};
        double fence{200.0};
        double prev{2.0};
        double prev_decay{0.85};
        double pub{3.0};
        double vlim{60.0};
        double alim{60.0};
        double acc{2.0};
        double jerk{0.1};
        // The goal's velocity RELATIVE TO THE PERSON'S. This is the friction. The published-stage
        // anchor and the agreement term both carry the goal's velocity forward, which is what
        // removed the lag behind a walker -- and also made the goal a frictionless integrator:
        // beside a STANDING person it never settled, wandering +-5 deg and right across the
        // 3.1-3.9 m band for as long as it was watched. Relative, not absolute, so a walker costs
        // nothing in steady state; it bites only when the goal moves and the person does not.
        double vrel{4.0};
        // Stage 2. Both default to 0, and at 0 the problem IS the Stage 1 problem: the degraded
        // mode of the visibility cost is a configuration that has already been gated.
        double vis{0.0};     // on the known-occluded fraction at the goal's azimuth, in [0, 1]
        double clear{0.0};   // on metres beyond the radius the inflated map leaves clear there
    };

    // Limits on the GOAL's motion, not the aircraft's. They are deliberately above SUPER's
    // max_vel / max_acc: a goal throttled to the aircraft's envelope turns "the aircraft is
    // lagging" into "the goal is lagging", which loses the lock instead of trailing. Their job is
    // to forbid jumps. The norm is 3-D because SUPER's max_vel is a 3-D cap.
    //
    // goal_acc_max is 3.0 and not the 1.0 first tried, and that is measured, not taste: with a
    // 1.2 s horizon a goal limited to 1.0 m/s^2 cannot brake inside what the horizon sees, so
    // after a 90 deg turn at 1 m/s it overshot 1.5 m sideways and swung for 10 s (the ClosedLoop
    // test). At 3.0 the overshoot is 0.27 m and it settles in 3 s. The step per tick is bounded
    // by goal_vel_max either way (0.1 m at 15 Hz), so the published goal stays continuous.
    struct Limits {
        double goal_vel_max{1.5};
        double goal_acc_max{3.0};
        double smooth_mu{0.01};
    };

    // Mirror of command_mux's box, off unless the node is told otherwise. x/y in the world frame
    // rotated by yaw; the hinge starts `margin` inside the edge, where the mux would clamp.
    struct Fence {
        bool enable{false};
        double x_min{-2.0}, x_max{28.0}, y_min{-2.0}, y_max{2.0};
        double yaw{0.0};
        double margin{0.5};
    };

    // A function of azimuth as a truncated Fourier series: a0/2 + sum a_k cos(k az) + b_k sin(k az).
    // That is how a 24-bin VisibilityField becomes a differentiable cost: smooth, with a closed-
    // form gradient, and exactly periodic, which a spline has to be told.
    //
    // 8 harmonics, and not the 4 first built. 4 is an azimuthal low-pass ~45 deg wide, chosen to
    // ride out ROG-Map's flickering tangent cells -- but a trunk 2 m from the person casts an
    // ~11 deg shadow, and at 45 deg that became a bump too wide and too shallow to leave: end to
    // end, against the real map, the goal stayed in the shadow. 8 resolves ~22 deg. Flicker is
    // handled where it belongs instead: fsm_node samples several azimuths inside every bin, and
    // the node averages successive fields before fitting. (kVisFineHarmonics, below.)
    struct AzimuthSeries {
        static constexpr int kMaxHarmonics = 8;
        int harmonics{0};
        double a[kMaxHarmonics + 1]{};
        double b[kMaxHarmonics + 1]{};

        // Least-squares fit to `n` uniform samples v[i] at az0 + i*daz over a full turn (for
        // uniform samples, the DFT), truncated at `harmonics` and then WINDOWED with Lanczos
        // sigma factors, sinc(k / (harmonics + 1)). The window is not optional. The field is
        // made of boxes (a bin is dark or it is not), and a bare truncated series of a box rings
        // -- Gibbs ripples of ~+-9 % all the way across a wide shadow's plateau, each ripple a
        // little well the optimiser settles into: the unit test's goal sat 2 deg from the middle
        // of a 60 deg shadow, in one. Windowed, a box comes back monotone across each edge.
        static AzimuthSeries Fit(const std::vector<double> &v, double az0, double daz, int harmonics) {
            AzimuthSeries s;
            s.harmonics = std::min(std::max(harmonics, 0), kMaxHarmonics);
            const int n = static_cast<int>(v.size());
            if (n == 0) { return s; }
            for (int k = 0; k <= s.harmonics; ++k) {
                for (int i = 0; i < n; ++i) {
                    const double az = az0 + i * daz;
                    s.a[k] += v[i] * std::cos(k * az) * 2.0 / n;
                    s.b[k] += v[i] * std::sin(k * az) * 2.0 / n;
                }
                if (k > 0) {
                    const double x = M_PI * k / (s.harmonics + 1.0);
                    s.a[k] *= std::sin(x) / x;
                    s.b[k] *= std::sin(x) / x;
                }
            }
            return s;
        }

        double Eval(double az, double &d_daz) const {
            double f = 0.5 * a[0];
            d_daz = 0.0;
            for (int k = 1; k <= harmonics; ++k) {
                const double c = std::cos(k * az), sn = std::sin(k * az);
                f += a[k] * c + b[k] * sn;
                d_daz += k * (-a[k] * sn + b[k] * c);
            }
            return f;
        }
    };

    constexpr int kVisFineHarmonics = 8;     // resolves ~22 deg: a trunk's shadow
    constexpr int kVisCoarseHarmonics = 3;   // resolves ~60 deg: which way out of a wide one

    // The visibility field, already reduced by the node: azimuths are measured from the person
    // OUTWARD, in the world frame, so the field rides along with the forecast person.
    struct Visibility {
        // The KNOWN occluded fraction (1 - vis - unk; unknown is not bad), at two scales.
        AzimuthSeries occluded, occluded_coarse;
        AzimuthSeries clear_r;    // how far out the inflated map stays clear, metres

        static Visibility FromBins(const std::vector<double> &occ, const std::vector<double> &clr,
                                   double az0, double daz) {
            Visibility v;
            v.occluded = AzimuthSeries::Fit(occ, az0, daz, kVisFineHarmonics);
            v.occluded_coarse = AzimuthSeries::Fit(occ, az0, daz, kVisCoarseHarmonics);
            v.clear_r = AzimuthSeries::Fit(clr, az0, daz, kVisFineHarmonics);
            return v;
        }
    };

    struct Problem {
        int n{0};                 // stages = columns of target / init / prev
        double dt{1.0 / 15.0};
        Eigen::Vector3d g0{Eigen::Vector3d::Zero()};  // the goal the horizon continues from
        Eigen::Vector3d v0{Eigen::Vector3d::Zero()};  // and its velocity
        Eigen::Matrix3Xd target;  // the person at stage k = 1..n (column k-1)
        Eigen::Vector3d p_now{Eigen::Vector3d::Zero()};    // the person's PRESENT position
        double az_des{0.0};       // the trail azimuth, decided by the node (near-side bias, reproject)
        double band_lo{3.0}, band_hi{4.0};
        double floor_m{2.75};     // min_standoff_m + the node's floor reserve
        double z_offset{0.5}, z_min{0.5}, z_max{3.0};
        Eigen::Matrix3Xd init;    // where the search starts
        Eigen::Matrix3Xd prev;    // the previous solution re-sampled onto this tick's stages
        bool has_prev{false};
        int publish_stage{1};     // 1-based
        Weights w;
        Limits lim;
        Fence fence;
        Visibility visibility;    // read only where w.vis or w.clear is > 0
        // Not 40: a warm start usually needs a handful, but through a turn it needs ~100, and
        // stopping short there leaves the plan trailing reality (1.0 m outside the band in the
        // closed-loop test). The time budget below is the real cap.
        int max_iters{150};
        double time_budget_ms{8.0};
    };

    struct Result {
        bool ok{false};           // finite and no worse than where the search started
        int lbfgs_code{0};
        bool budget_cancel{false};
        int iterations{0};
        double cost0{0.0}, cost{0.0};
        double solve_ms{0.0};
        Eigen::Matrix3Xd horizon;
        Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
        const char *why{""};
    };

    inline double Hinge(double x) { return x > 0.0 ? x : 0.0; }

    // A C2 one-sided penalty: 0 below 0, linear above mu, a quartic blend between. Value and
    // slope meet the linear branch at mu (mu/2 and 1), so a line search never meets a kink.
    inline double SmoothedL1(double x, double mu, double &df) {
        if (x <= 0.0) {
            df = 0.0;
            return 0.0;
        }
        if (x >= mu) {
            df = 1.0;
            return x - 0.5 * mu;
        }
        const double mu3 = mu * mu * mu;
        df = (3.0 * mu * x * x - 2.0 * x * x * x) / mu3;
        return (mu - 0.5 * x) * x * x * x / mu3;
    }

    inline bool AllFinite(const Eigen::Matrix3Xd &m) { return m.allFinite(); }

    // Cost and analytic gradient. x is g_1..g_n flattened column-major. `grad` may be null.
    inline double Cost(const Problem &p, const Eigen::VectorXd &x, Eigen::VectorXd *grad) {
        const int n = p.n;
        const double dt = p.dt;
        const double inv_n = 1.0 / static_cast<double>(n);

        // Columns: 0 = g_{-1} (so that a_0 exists), 1 = g_0, k+1 = g_k. The first two are data.
        Eigen::Matrix3Xd P(3, n + 2);
        P.col(0) = p.g0 - p.v0 * dt;
        P.col(1) = p.g0;
        for (int k = 1; k <= n; ++k) { P.col(k + 1) = x.segment<3>(3 * (k - 1)); }
        Eigen::Matrix3Xd GP = Eigen::Matrix3Xd::Zero(3, n + 2);

        const Eigen::Vector2d u(std::cos(p.az_des), std::sin(p.az_des));
        const Eigen::Vector2d nrm(-u.y(), u.x());
        const double cy = std::cos(p.fence.yaw), sy = std::sin(p.fence.yaw);
        double J = 0.0;
        double decay = 1.0;

        for (int k = 1; k <= n; ++k) {
            const Eigen::Vector3d g = P.col(k + 1);
            const Eigen::Vector3d &s = p.target.col(k - 1);
            Eigen::Vector3d dg = Eigen::Vector3d::Zero();
            double c = 0.0;

            // Standoff BAND, a dead zone: D17 asks for a band, and a point target would have the
            // goal chase +-0.5 m of range noise all day. Horizontal, as EnforceStandoffFloor is.
            const Eigen::Vector2d d = g.head<2>() - s.head<2>();
            const double r = d.norm();
            const Eigen::Vector2d dir = r > 1e-9 ? Eigen::Vector2d(d / r) : u;
            const double h_lo = Hinge(p.band_lo - r), h_hi = Hinge(r - p.band_hi);
            c += p.w.standoff * (h_lo * h_lo + h_hi * h_hi);
            dg.head<2>() += p.w.standoff * 2.0 * (h_hi - h_lo) * dir;

            // Bearing, as the SINE of the angle off the desired trail ray, scaled to metres at the
            // nominal radius: no atan2 branch cut in the gradient. It is normalised by the range
            // on purpose. As a plain lateral offset in metres (d . nrm) it is smaller the closer
            // the goal is, so any off-bearing goal was pulled INWARD against the band -- to
            // 2.99 m when the visibility cost held it 30 deg off. Bearing must not buy range.
            // The hinge on the along-ray component removes the mirror optimum IN FRONT of the
            // person, which has zero angle off the ray too.
            const double r_nom = 0.5 * (p.band_lo + p.band_hi);
            const double h_front = Hinge(-d.dot(u));
            c += p.w.bearing * h_front * h_front;
            dg.head<2>() += -p.w.bearing * 2.0 * h_front * u;
            if (r > 1e-6) {
                const double sin_off = d.dot(nrm) / r;
                const double e_lat = r_nom * sin_off;
                c += p.w.bearing * e_lat * e_lat;
                dg.head<2>() += p.w.bearing * 2.0 * e_lat * r_nom * (nrm - sin_off * dir) / r;
            }

            // Altitude. The reference is clamped to the Z range so this never fights zlim.
            const double z_des = std::clamp(s.z() + p.z_offset, p.z_min, p.z_max);
            const double ez = g.z() - z_des;
            c += p.w.alt * ez * ez;
            dg.z() += p.w.alt * 2.0 * ez;

            // There is deliberately NO term keeping the goal away from the aircraft. One was
            // tried, to steer clear of the node's 0.25 m park guard, and it was wrong twice: a
            // hovering aircraft SHOULD sit on its goal, so repelling the goal makes it creep
            // away from a follower that has arrived; and with a person backing towards the goal
            // the goal could not retreat past the aircraft (2.65 m from the person where the
            // geometric goal kept 3.74 m, nmpc_check N4).

            // The standoff floor, against the person's PRESENT position: a safety number
            // checked against a prediction is only as good as the prediction.
            const Eigen::Vector2d dn = g.head<2>() - p.p_now.head<2>();
            const double rn = dn.norm();
            const double h_floor = Hinge(p.floor_m - rn);
            if (h_floor > 0.0) {
                const Eigen::Vector2d dirn = rn > 1e-9 ? Eigen::Vector2d(dn / rn) : u;
                c += p.w.floor * h_floor * h_floor;
                dg.head<2>() += -p.w.floor * 2.0 * h_floor * dirn;
            }

            // Stage 2: stay where the map says the person can be seen from, and inside the
            // radius it leaves clear there. Both depend on the azimuth of the goal about the
            // FORECAST person, so the field moves with them. Skipped close in, where the
            // azimuth is ill-conditioned and the floor term owns the geometry anyway.
            if ((p.w.vis > 0.0 || p.w.clear > 0.0) && r > 0.5) {
                const double az = std::atan2(d.y(), d.x());
                const Eigen::Vector2d daz_dg(-d.y() / (r * r), d.x() / (r * r));
                if (p.w.vis > 0.0) {
                    // TWO scales of the same field. The full series resolves a trunk's ~11 deg
                    // shadow; but a WIDE shadow is then a plateau -- occluded ~1 across all of it,
                    // slope zero -- and a gradient method standing in the middle has no way out
                    // (the unit test sat at 0 deg off for 10 s). The coarse series is the same
                    // field seen from far away: a broad hill whose slope, anywhere inside a wide
                    // shadow, points at the nearer edge.
                    for (const AzimuthSeries *f : {&p.visibility.occluded, &p.visibility.occluded_coarse}) {
                        double dO = 0.0;
                        const double h_occ = Hinge(f->Eval(az, dO));
                        c += p.w.vis * h_occ * h_occ;
                        dg.head<2>() += p.w.vis * 2.0 * h_occ * dO * daz_dg;
                    }
                }
                if (p.w.clear > 0.0) {
                    double dR = 0.0;
                    const double h_out = Hinge(r - p.visibility.clear_r.Eval(az, dR));
                    c += p.w.clear * h_out * h_out;
                    dg.head<2>() += p.w.clear * 2.0 * h_out * (dir - dR * daz_dg);
                }
            }

            // Z range: outside it ROG-Map reads virtual occupancy and SUPER drops the goal.
            const double h_zlo = Hinge(p.z_min - g.z()), h_zhi = Hinge(g.z() - p.z_max);
            c += p.w.zlim * (h_zlo * h_zlo + h_zhi * h_zhi);
            dg.z() += p.w.zlim * 2.0 * (h_zhi - h_zlo);

            if (p.fence.enable) {
                const double bx = cy * g.x() + sy * g.y(), by = -sy * g.x() + cy * g.y();
                const double m = p.fence.margin;
                const double hx = Hinge(bx - (p.fence.x_max - m)) - Hinge((p.fence.x_min + m) - bx);
                const double hy = Hinge(by - (p.fence.y_max - m)) - Hinge((p.fence.y_min + m) - by);
                c += p.w.fence * (hx * hx + hy * hy);
                dg.x() += p.w.fence * 2.0 * (hx * cy - hy * sy);
                dg.y() += p.w.fence * 2.0 * (hx * sy + hy * cy);
            }

            // Agreement with the previous solution, strongest on the near stages (they decide
            // what is published) and fading so the tail stays free to re-plan.
            decay *= p.w.prev_decay;
            if (p.has_prev) {
                const Eigen::Vector3d e = g - p.prev.col(k - 1);
                c += p.w.prev * decay * e.squaredNorm();
                dg += p.w.prev * decay * 2.0 * e;
            }

            J += inv_n * c;
            GP.col(k + 1) += inv_n * dg;
        }

        // Velocity limit, on the squared norm so nothing is non-differentiable at rest.
        const double vmax2 = p.lim.goal_vel_max * p.lim.goal_vel_max;
        for (int k = 1; k <= n; ++k) {
            const Eigen::Vector3d v = (P.col(k + 1) - P.col(k)) / dt;
            double df = 0.0;
            J += inv_n * p.w.vlim * SmoothedL1(v.squaredNorm() / vmax2 - 1.0, p.lim.smooth_mu, df);
            const Eigen::Vector3d gv = inv_n * p.w.vlim * df * 2.0 * v / vmax2;
            GP.col(k + 1) += gv / dt;
            GP.col(k) -= gv / dt;
        }

        // Friction against the person's own motion (see Weights::vrel). The person's velocity at
        // stage k is the forecast's own finite difference; before the first stage it is taken to
        // continue at the first interval's rate.
        for (int k = 1; k <= n; ++k) {
            const Eigen::Vector3d v = (P.col(k + 1) - P.col(k)) / dt;
            const Eigen::Vector3d vt = (k >= 2) ? Eigen::Vector3d((p.target.col(k - 1) - p.target.col(k - 2)) / dt)
                                                : Eigen::Vector3d((p.target.col(1) - p.target.col(0)) / dt);
            const Eigen::Vector3d e = v - vt;
            J += inv_n * p.w.vrel * e.squaredNorm() / vmax2;
            const Eigen::Vector3d ge = inv_n * p.w.vrel * 2.0 * e / vmax2;
            GP.col(k + 1) += ge / dt;
            GP.col(k) -= ge / dt;
        }

        // Acceleration: a limit and a small effort term. Then jerk, which is what makes the
        // horizon one object rather than n separate placements. Both regularisers stay small:
        // over-weighted they make the goal lag the person.
        const double dt2 = dt * dt;
        const double amax2 = p.lim.goal_acc_max * p.lim.goal_acc_max;
        const double jn2 = amax2 / dt2;
        Eigen::Matrix3Xd A(3, n), GA = Eigen::Matrix3Xd::Zero(3, n);
        for (int k = 0; k < n; ++k) {
            A.col(k) = (P.col(k + 2) - 2.0 * P.col(k + 1) + P.col(k)) / dt2;
            const double a2 = A.col(k).squaredNorm();
            double df = 0.0;
            J += inv_n * (p.w.alim * SmoothedL1(a2 / amax2 - 1.0, p.lim.smooth_mu, df)
                          + p.w.acc * a2 / amax2);
            GA.col(k) += inv_n * (p.w.alim * df + p.w.acc) * 2.0 * A.col(k) / amax2;
        }
        for (int k = 1; k < n; ++k) {
            const Eigen::Vector3d j = (A.col(k) - A.col(k - 1)) / dt;
            J += inv_n * p.w.jerk * j.squaredNorm() / jn2;
            const Eigen::Vector3d gj = inv_n * p.w.jerk * 2.0 * j / jn2;
            GA.col(k) += gj / dt;
            GA.col(k - 1) -= gj / dt;
        }
        for (int k = 0; k < n; ++k) {
            GP.col(k + 2) += GA.col(k) / dt2;
            GP.col(k + 1) -= 2.0 * GA.col(k) / dt2;
            GP.col(k) += GA.col(k) / dt2;
        }

        // The published stage is held to where the published signal was already heading, by a
        // term of its own. Without it the node's 0.15 m deadband is all that holds the output
        // still, and a deadband is a latch that chatters at its edge. The anchor is the
        // constant-velocity continuation g_0 + v_0 t and NOT g_0 itself: anchored to g_0 this is
        // a penalty on speed, and the goal trails a walking person by a distance proportional to
        // their pace (1.1 m outside the band at 1 m/s in the closed-loop test, which is how it
        // was found). At rest the two are the same point.
        const Eigen::Vector3d ep = P.col(p.publish_stage + 1) - p.g0
                                   - p.v0 * (dt * static_cast<double>(p.publish_stage));
        J += p.w.pub * ep.squaredNorm();
        GP.col(p.publish_stage + 1) += p.w.pub * 2.0 * ep;

        if (grad != nullptr) {
            grad->resize(3 * n);
            for (int k = 1; k <= n; ++k) { grad->segment<3>(3 * (k - 1)) = GP.col(k + 1); }
        }
        return J;
    }

    // Re-sample a previous horizon onto this tick's stages. `elapsed` is the time since that
    // horizon's g_0. With elapsed == dt this is an exact index shift; otherwise it interpolates.
    // Past the end it extends at the horizon's last velocity.
    inline Eigen::Matrix3Xd ShiftHorizon(const Eigen::Matrix3Xd &prev, const Eigen::Vector3d &prev_g0,
                                         double elapsed, double dt, int n) {
        const int m = static_cast<int>(prev.cols());
        Eigen::Matrix3Xd out(3, n);
        auto at = [&](int i) -> Eigen::Vector3d { return i < 0 ? prev_g0 : Eigen::Vector3d(prev.col(i)); };
        const Eigen::Vector3d v_end = (at(m - 1) - at(m - 2)) / dt;
        for (int k = 1; k <= n; ++k) {
            const double f = elapsed / dt + static_cast<double>(k) - 1.0;  // fractional column
            const int i = static_cast<int>(std::floor(f));
            if (i >= m - 1) {
                out.col(k - 1) = at(m - 1) + v_end * (f - static_cast<double>(m - 1)) * dt;
            } else {
                const double a = f - static_cast<double>(i);
                out.col(k - 1) = (1.0 - a) * at(i) + a * at(i + 1);
            }
        }
        return out;
    }

    // The person at t0 + k*dt, k = 1..n, from the forecast polyline (times ts, points pts, both
    // relative to the target's stamp). Reading stops at the first non-finite or non-increasing
    // sample. Before the first sample and past the last it moves at `vel`: a forecast is never
    // bent to reach further than it said. `usable` reports how much forecast there really was.
    inline Eigen::Matrix3Xd SampleTargetPath(const std::vector<double> &ts,
                                             const std::vector<Eigen::Vector3d> &pts,
                                             const Eigen::Vector3d &p_ref, const Eigen::Vector3d &vel,
                                             double t0, double dt, int n, int &usable) {
        std::vector<double> t;
        std::vector<Eigen::Vector3d> q;
        for (size_t i = 0; i < ts.size() && i < pts.size(); ++i) {
            if (!std::isfinite(ts[i]) || !pts[i].allFinite()) { break; }
            if (!t.empty() && ts[i] <= t.back()) { break; }
            t.push_back(ts[i]);
            q.push_back(pts[i]);
        }
        usable = static_cast<int>(t.size());
        Eigen::Vector3d v = vel;
        v.z() = 0.0;  // the forecast is a ground-plane model; an invented climb would steer goal z
        Eigen::Matrix3Xd out(3, n);
        for (int k = 1; k <= n; ++k) {
            const double tk = t0 + static_cast<double>(k) * dt;
            if (t.empty()) {
                out.col(k - 1) = p_ref + v * (tk - t0);
            } else if (tk <= t.front()) {
                out.col(k - 1) = q.front() + v * (tk - t.front());
            } else if (tk >= t.back()) {
                out.col(k - 1) = q.back() + v * (tk - t.back());
            } else {
                const size_t j = std::upper_bound(t.begin(), t.end(), tk) - t.begin();
                const double a = (tk - t[j - 1]) / (t[j] - t[j - 1]);
                out.col(k - 1) = (1.0 - a) * q[j - 1] + a * q[j];
            }
        }
        return out;
    }

    // The trail bearing, moved out of a known shadow. If the bearing the node asked for is dark,
    // return the NEAREST bearing that is clear instead, and let the solver's ordinary bearing term
    // carry the goal there at the goal's own speed limit.
    //
    // This exists because the visibility cost alone cannot be relied on to get a goal out of a
    // shadow it starts in the middle of. The middle is the top of a hill: slope zero. And it is
    // not a corner case -- a person standing still is trailed from wherever the aircraft already
    // is, so whatever comes between them lands EXACTLY on the trail bearing. Worse, in a wide
    // shadow the middle is STABLE: the bearing term pulls back towards the asked-for bearing at
    // ~24.5 per radian of offset (r_nom^2 = 3.5^2, doubled), and a 60 deg shadow's coarse hill
    // pushes out at only ~21.7. A first attempt nudged the solver's start 3 deg a tick; it won
    // or lost that race run to run, against the same binaries (0.43 m clear of a tube in one
    // run, 0.02 m in the next). Deciding the bearing OUTSIDE the solver has no race in it.
    //
    // `side` is the caller's memory (-1, 0, +1): once displaced to one side it stays there until
    // the other is closer by more than kSideHysteresis, and it returns to the asked-for bearing
    // only when that has become properly clear -- otherwise a field hovering at the threshold
    // would swing the goal from one side of the person to the other.
    inline double ClearBearing(const Visibility &v, double az_des, int &side) {
        constexpr double kDark = 0.30;    // the windowed fit roughly halves a one-bin (trunk) shadow
        constexpr double kClear = 0.15;
        constexpr double kStep = 5.0 * M_PI / 180.0;
        // More than one 15 deg bin plus a search step: ONE bin toggling (ROG-Map's tangent cells
        // flicker) moves a side's offset by a whole bin, and must not flip the goal to the other
        // side of the person.
        constexpr double kSideHysteresis = 25.0 * M_PI / 180.0;
        constexpr int kMaxSteps = 18;     // look no further than 90 deg round
        double slope = 0.0;
        const double here = v.occluded.Eval(az_des, slope);
        if (here < (side == 0 ? kDark : kClear)) {
            side = 0;
            return az_des;
        }
        double off[2] = {-1.0, -1.0};     // [0] = turning +, [1] = turning -
        for (int i = 1; i <= kMaxSteps; ++i) {
            for (int j = 0; j < 2; ++j) {
                if (off[j] < 0.0 && v.occluded.Eval(az_des + (j == 0 ? 1.0 : -1.0) * i * kStep, slope) < kClear) {
                    off[j] = i * kStep;
                }
            }
        }
        if (off[0] < 0.0 && off[1] < 0.0) {   // dark all the way round: nothing better to offer
            side = 0;
            return az_des;
        }
        int pick;
        if (off[0] < 0.0) { pick = 1; }
        else if (off[1] < 0.0) { pick = 0; }
        else if (side > 0) { pick = (off[1] + kSideHysteresis < off[0]) ? 1 : 0; }
        else if (side < 0) { pick = (off[0] + kSideHysteresis < off[1]) ? 0 : 1; }
        else { pick = (off[0] <= off[1]) ? 0 : 1; }
        side = (pick == 0) ? 1 : -1;
        return az_des + side * off[pick];
    }

    class Solver {
    public:
        Result Solve(const Problem &p) {
            Result r;
            const auto t_start = std::chrono::steady_clock::now();
            auto finish = [&](const char *why) {
                r.why = why;
                r.solve_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - t_start).count();
                return r;
            };
            if (p.n < 2 || p.target.cols() != p.n || p.init.cols() != p.n ||
                (p.has_prev && p.prev.cols() != p.n) || p.publish_stage < 1 || p.publish_stage > p.n ||
                !(p.dt > 0.0)) {
                return finish("problem malformed");
            }
            if (!AllFinite(p.target) || !AllFinite(p.init) || (p.has_prev && !AllFinite(p.prev)) ||
                !p.g0.allFinite() || !p.v0.allFinite() || !p.p_now.allFinite() ||
                !std::isfinite(p.az_des)) {
                return finish("non-finite input");
            }

            problem_ = &p;
            deadline_ = t_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                      std::chrono::duration<double, std::milli>(p.time_budget_ms));
            canceled_ = false;
            iterations_ = 0;

            Eigen::VectorXd x(3 * p.n);
            for (int k = 0; k < p.n; ++k) { x.segment<3>(3 * k) = p.init.col(k); }
            r.cost0 = Cost(p, x, nullptr);
            if (!std::isfinite(r.cost0)) { return finish("non-finite initial cost"); }

            lbfgs::lbfgs_parameter_t prm;
            prm.mem_size = 16;
            // Stop on a flat cost (past 3, delta 1e-4): a warm start is then ~16 iterations and
            // 0.06 ms, against ~90 and 0.55 ms for delta 1e-7, for the same closed-loop result
            // (band excursion 0.06 m vs 0.04 m in the ClosedLoop test). What must NOT be tight
            // is max_iters: a turn needs ~100 iterations, and capped at 40 the plan trailed
            // reality by 1.0 m. The cheap test and the generous cap are a pair.
            prm.past = 3;
            prm.delta = 1.0e-4;
            prm.g_epsilon = 1.0e-5;  // legitimate: every term is C1
            prm.max_iterations = p.max_iters;
            prm.max_linesearch = 16;

            double f = r.cost0;
            r.lbfgs_code = lbfgs::lbfgs_optimize(x, f, &Solver::Evaluate, nullptr, &Solver::Progress,
                                                 this, prm);
            r.iterations = iterations_;
            r.budget_cancel = canceled_;
            // The return code is recorded, not judged. Every exit leaves x on a descent iterate
            // (a failed line search reverts to the last good point; a cancel keeps the current
            // one), and with a warm start "line search failed" mostly means "already there".
            // What is judged is the result itself.
            r.cost = Cost(p, x, nullptr);
            if (!x.allFinite() || !std::isfinite(r.cost)) { return finish("non-finite solution"); }
            if (r.cost > r.cost0 * (1.0 + 1e-9) + 1e-12) { return finish("cost increased"); }

            r.horizon.resize(3, p.n);
            for (int k = 0; k < p.n; ++k) { r.horizon.col(k) = x.segment<3>(3 * k); }
            r.goal = r.horizon.col(p.publish_stage - 1);
            r.ok = true;
            return finish("");
        }

    private:
        // LBFGS-Lite takes plain function pointers, hence static members and the instance cast.
        static double Evaluate(void *instance, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
            const Solver *self = static_cast<Solver *>(instance);
            return Cost(*self->problem_, x, &g);
        }

        // The time budget. A non-zero return cancels the solve and keeps the current iterate,
        // so the budget costs solution quality and never validity.
        static int Progress(void *instance, const Eigen::VectorXd &, const Eigen::VectorXd &,
                            const double, const double, const int k, const int) {
            Solver *self = static_cast<Solver *>(instance);
            self->iterations_ = k;
            if (std::chrono::steady_clock::now() >= self->deadline_) {
                self->canceled_ = true;
                return 1;
            }
            return 0;
        }

        const Problem *problem_{nullptr};
        std::chrono::steady_clock::time_point deadline_;
        bool canceled_{false};
        int iterations_{0};
    };

}  // namespace nmpc
}  // namespace follow_planner
