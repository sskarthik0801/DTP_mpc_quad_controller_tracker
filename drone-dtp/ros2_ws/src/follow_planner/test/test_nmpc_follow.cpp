// Unit tests for the goal_mode nmpc solver (include/follow_planner/nmpc_follow.hpp). Pure C++:
// no rclcpp, no node, no DDS domain. Run:
//   colcon test --packages-select follow_planner && colcon test-result --verbose
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <random>
#include <vector>

#include "follow_planner/nmpc_follow.hpp"

namespace nm = follow_planner::nmpc;

// follow_planner had no C++ test before this file. This one exists so that a broken harness and
// a broken solver can be told apart.
TEST(Harness, Runs) { EXPECT_EQ(1, 1); }

namespace {

// A person walking along +x at `speed`, the goal trailing on -x. Everything a test does not care
// about sits at the node's defaults.
nm::Problem WalkProblem(int n, double speed) {
    nm::Problem p;
    p.n = n;
    p.dt = 1.0 / 15.0;
    p.az_des = M_PI;
    p.p_now = Eigen::Vector3d(5.0, 0.0, 0.1);
    p.z_min = 1.2;
    p.target.resize(3, n);
    p.init.resize(3, n);
    for (int k = 1; k <= n; ++k) {
        p.target.col(k - 1) = p.p_now + Eigen::Vector3d(speed * (0.2 + k * p.dt), 0.0, 0.0);
        p.init.col(k - 1) = p.target.col(k - 1) + Eigen::Vector3d(-3.5, 0.0, 1.1);
    }
    p.g0 = p.p_now + Eigen::Vector3d(-3.5 + 0.2 * speed, 0.0, 1.1);
    p.v0 = Eigen::Vector3d(speed, 0.0, 0.0);
    return p;
}

// Random geometry that puts every hinge on both sides of its knot across the sample.
nm::Problem RandomProblem(std::mt19937 &rng, int n) {
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    nm::Problem p = WalkProblem(n, 0.8);
    p.az_des = M_PI * U(rng);
    p.g0 = Eigen::Vector3d(2.0 + 2.0 * U(rng), 2.0 * U(rng), 1.2 + U(rng));
    p.v0 = Eigen::Vector3d(U(rng), U(rng), 0.2 * U(rng));
    p.has_prev = true;
    p.prev.resize(3, n);
    p.fence.enable = true;
    p.fence.yaw = 0.4 * U(rng);
    p.fence.x_max = 6.0;
    std::vector<double> occ(24), clr(24);   // a field with structure in it
    for (int i = 0; i < 24; ++i) {
        occ[i] = 0.5 + 0.5 * U(rng);
        clr[i] = 3.5 + 0.5 * U(rng);
    }
    p.visibility = nm::Visibility::FromBins(occ, clr, -M_PI, 2.0 * M_PI / 24);
    for (int k = 0; k < n; ++k) {
        p.init.col(k) = p.g0 + Eigen::Vector3d(3.0 * U(rng), 3.0 * U(rng), 2.0 * U(rng));
        p.prev.col(k) = p.init.col(k) + 0.3 * Eigen::Vector3d(U(rng), U(rng), U(rng));
    }
    return p;
}

Eigen::VectorXd Flat(const Eigen::Matrix3Xd &m) {
    Eigen::VectorXd x(3 * m.cols());
    for (int k = 0; k < m.cols(); ++k) { x.segment<3>(3 * k) = m.col(k); }
    return x;
}

double MaxGradError(const nm::Problem &p, const Eigen::VectorXd &x) {
    Eigen::VectorXd g;
    nm::Cost(p, x, &g);
    const double h = 1e-6;
    double worst = 0.0;
    Eigen::VectorXd xp = x;
    for (int i = 0; i < x.size(); ++i) {
        xp(i) = x(i) + h;
        const double fp = nm::Cost(p, xp, nullptr);
        xp(i) = x(i) - h;
        const double fm = nm::Cost(p, xp, nullptr);
        xp(i) = x(i);
        const double fd = (fp - fm) / (2.0 * h);
        worst = std::max(worst, std::abs(fd - g(i)) / std::max(1.0, std::abs(g(i))));
    }
    return worst;
}

nm::Weights Only(double nm::Weights::*field) {
    nm::Weights w;
    w.standoff = w.bearing = w.alt = w.floor = w.zlim = w.fence = 0.0;
    w.prev = w.pub = w.vlim = w.alim = w.acc = w.jerk = w.vrel = w.vis = w.clear = 0.0;
    // Stage 2's weights default to 0 (off), so "the default" would test nothing.
    const bool stage2 = field == &nm::Weights::vis || field == &nm::Weights::clear;
    w.*field = stage2 ? 5.0 : nm::Weights().*field;
    return w;
}

}  // namespace

// A wrong gradient still converges, to the wrong place, and no flight would say why. Each term
// alone (so one term's error cannot hide in another's magnitude), then the sum.
TEST(Gradient, EveryTermMatchesCentralDifferences) {
    const std::vector<std::pair<const char *, double nm::Weights::*>> terms = {
        {"standoff", &nm::Weights::standoff}, {"bearing", &nm::Weights::bearing},
        {"alt", &nm::Weights::alt},
        {"floor", &nm::Weights::floor},       {"zlim", &nm::Weights::zlim},
        {"fence", &nm::Weights::fence},       {"prev", &nm::Weights::prev},
        {"pub", &nm::Weights::pub},           {"vlim", &nm::Weights::vlim},
        {"alim", &nm::Weights::alim},         {"acc", &nm::Weights::acc},
        {"jerk", &nm::Weights::jerk},         {"vrel", &nm::Weights::vrel},
        {"vis", &nm::Weights::vis},
        {"clear", &nm::Weights::clear}};
    std::mt19937 rng(7);
    for (int trial = 0; trial < 20; ++trial) {
        nm::Problem p = RandomProblem(rng, 8);
        const Eigen::VectorXd x = Flat(p.init);
        const nm::Weights all = p.w;
        for (const auto &t : terms) {
            p.w = Only(t.second);
            if (t.second == &nm::Weights::prev) { p.w.prev_decay = all.prev_decay; }
            EXPECT_LT(MaxGradError(p, x), 1e-5) << "term " << t.first << ", trial " << trial;
        }
        p.w = all;
        p.w.vis = p.w.clear = 5.0;
        EXPECT_LT(MaxGradError(p, x), 1e-5) << "all terms, trial " << trial;
    }
}

// The terms must actually be exercised by the sample above, or the test proves nothing.
TEST(Gradient, RandomSampleActivatesEveryTerm) {
    std::mt19937 rng(7);
    std::vector<double nm::Weights::*> fields = {
        &nm::Weights::standoff, &nm::Weights::bearing, &nm::Weights::alt,
        &nm::Weights::floor,    &nm::Weights::zlim,    &nm::Weights::fence, &nm::Weights::prev,
        &nm::Weights::pub,      &nm::Weights::vlim,    &nm::Weights::alim,  &nm::Weights::acc,
        &nm::Weights::jerk,     &nm::Weights::vrel,    &nm::Weights::vis,   &nm::Weights::clear};
    std::vector<int> active(fields.size(), 0);
    for (int trial = 0; trial < 20; ++trial) {
        nm::Problem p = RandomProblem(rng, 8);
        const Eigen::VectorXd x = Flat(p.init);
        for (size_t i = 0; i < fields.size(); ++i) {
            p.w = Only(fields[i]);
            if (nm::Cost(p, x, nullptr) > 0.0) { ++active[i]; }
        }
    }
    for (size_t i = 0; i < fields.size(); ++i) { EXPECT_GT(active[i], 0) << "term index " << i; }
}

TEST(SmoothedL1, IsC1AcrossBothKnots) {
    const double mu = 0.01, e = 1e-9;
    for (double knot : {0.0, mu}) {
        double dl = 0.0, dr = 0.0;
        const double fl = nm::SmoothedL1(knot - e, mu, dl), fr = nm::SmoothedL1(knot + e, mu, dr);
        EXPECT_NEAR(fl, fr, 1e-8);
        EXPECT_NEAR(dl, dr, 1e-6);
    }
}

// Where the geometric answer is already optimal the solver must leave it alone: this is what
// makes an nmpc-vs-geometric comparison interpretable.
TEST(Solve, AlreadyOptimalGoalIsAFixedPoint) {
    nm::Problem p = WalkProblem(18, 0.0);
    p.v0.setZero();
    p.g0 = p.init.col(0);
    nm::Solver s;
    const nm::Result r = s.Solve(p);
    ASSERT_TRUE(r.ok) << r.why;
    EXPECT_LT((r.goal - p.init.col(0)).norm(), 1e-6);
    EXPECT_LE(r.iterations, 2);
}

TEST(Solve, NeverEndsWorseThanItStarted) {
    std::mt19937 rng(11);
    nm::Solver s;
    for (int trial = 0; trial < 100; ++trial) {
        const nm::Problem p = RandomProblem(rng, 18);
        const nm::Result r = s.Solve(p);
        ASSERT_TRUE(r.ok) << r.why << " (lbfgs " << r.lbfgs_code << "), trial " << trial;
        EXPECT_LE(r.cost, r.cost0 * (1.0 + 1e-9) + 1e-12);
        EXPECT_TRUE(r.horizon.allFinite());
    }
}

TEST(Solve, IsDeterministic) {
    std::mt19937 rng(3);
    const nm::Problem p = RandomProblem(rng, 18);
    nm::Solver a, b;
    const nm::Result ra = a.Solve(p), rb = b.Solve(p);
    ASSERT_TRUE(ra.ok && rb.ok);
    EXPECT_TRUE(ra.horizon == rb.horizon);  // bit-identical, not "close"
}

TEST(Solve, RefusesGarbageAndSurvivesStarvation) {
    nm::Solver s;
    nm::Problem bad = WalkProblem(18, 0.8);
    bad.target(0, 3) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(s.Solve(bad).ok);

    bad = WalkProblem(18, 0.8);
    bad.n = 17;  // columns no longer match
    EXPECT_FALSE(s.Solve(bad).ok);

    nm::Problem far = WalkProblem(18, 0.8);
    far.target.col(9) = Eigen::Vector3d(1e9, 0.0, 0.0);
    const nm::Result rf = s.Solve(far);
    if (rf.ok) { EXPECT_TRUE(rf.horizon.allFinite()); }

    std::mt19937 rng(5);
    nm::Problem one = RandomProblem(rng, 18);
    one.max_iters = 1;
    const nm::Result r1 = s.Solve(one);
    ASSERT_TRUE(r1.ok) << r1.why;
    EXPECT_TRUE(r1.horizon.allFinite());

    nm::Problem zero = RandomProblem(rng, 18);
    zero.time_budget_ms = 0.0;
    const nm::Result r0 = s.Solve(zero);
    ASSERT_TRUE(r0.ok) << r0.why;  // a cancel costs quality, never validity
    EXPECT_TRUE(r0.budget_cancel);
    EXPECT_LE(r0.cost, r0.cost0);
}

TEST(ShiftHorizon, OneTickIsAnExactIndexShift) {
    std::mt19937 rng(9);
    const nm::Problem p = RandomProblem(rng, 18);
    const Eigen::Matrix3Xd out = nm::ShiftHorizon(p.init, p.g0, p.dt, p.dt, 18);
    for (int k = 0; k < 17; ++k) { EXPECT_TRUE(out.col(k) == p.init.col(k + 1)) << k; }
    const Eigen::Vector3d tail = p.init.col(17) + (p.init.col(17) - p.init.col(16));
    EXPECT_LT((out.col(17) - tail).norm(), 1e-12);
    // No time elapsed: the horizon maps onto itself.
    EXPECT_LT((nm::ShiftHorizon(p.init, p.g0, 0.0, p.dt, 18) - p.init).norm(), 1e-12);
}

TEST(SampleTargetPath, InterpolatesStopsAtGarbageAndNeverBendsTheForecast) {
    std::vector<double> ts;
    std::vector<Eigen::Vector3d> pts;
    for (int i = 0; i < 20; ++i) {
        ts.push_back(0.2 + 0.075 * i);
        pts.emplace_back(1.0 * ts.back(), 0.0, 0.1);
    }
    const Eigen::Vector3d vel(1.0, 0.0, 0.7);  // a z velocity that must NOT be flown
    int usable = 0;
    Eigen::Matrix3Xd s = nm::SampleTargetPath(ts, pts, pts.front(), vel, 0.2, 1.0 / 15.0, 30, usable);
    EXPECT_EQ(usable, 20);
    for (int k = 1; k <= 30; ++k) {
        EXPECT_NEAR(s(0, k - 1), 0.2 + k / 15.0, 1e-9) << k;  // inside AND past the forecast
        EXPECT_NEAR(s(2, k - 1), 0.1, 1e-12);
    }
    pts[3].x() = std::numeric_limits<double>::quiet_NaN();
    s = nm::SampleTargetPath(ts, pts, pts.front(), vel, 0.2, 1.0 / 15.0, 18, usable);
    EXPECT_EQ(usable, 3);
    EXPECT_TRUE(s.allFinite());
    s = nm::SampleTargetPath({}, {}, Eigen::Vector3d(1, 2, 0.1), vel, 0.2, 0.1, 5, usable);
    EXPECT_EQ(usable, 0);
    EXPECT_NEAR(s(0, 4), 1.5, 1e-12);
}

// ---- Stage 2: the visibility field ---------------------------------------------------------------

std::vector<double> Bins(int n, double az0, const std::function<double(double)> &f) {
    std::vector<double> v(n);
    for (int i = 0; i < n; ++i) { v[i] = f(az0 + i * 2.0 * M_PI / n); }
    return v;
}

TEST(AzimuthSeries, ABoxComesBackWithoutRippleAndOneBinSurvives) {
    const int n = 24;
    const double az0 = -M_PI, daz = 2.0 * M_PI / n;
    auto wrap = [](double a) { return std::atan2(std::sin(a), std::cos(a)); };
    // A 60 deg shadow. A bare truncated series rings +-9 % across its plateau, and every ripple
    // is a well to get stuck in. Windowed: no overshoot, no undershoot, one maximum.
    const nm::AzimuthSeries box = nm::AzimuthSeries::Fit(
        Bins(n, az0, [&](double a) { return std::abs(wrap(a - 1.0)) < M_PI / 6.0 ? 1.0 : 0.0; }), az0, daz,
        nm::kVisFineHarmonics);
    int maxima = 0;
    double d = 0.0, prev_d = 0.0;
    for (int i = 0; i <= 720; ++i) {
        const double a = -M_PI + i * M_PI / 360.0;
        const double f = box.Eval(a, d);
        EXPECT_GT(f, -0.02);
        EXPECT_LT(f, 1.05);   // the window leaves ~3 %; what matters is the single maximum below
        if (i > 0 && prev_d > 0.0 && d <= 0.0 && f > 0.1) { ++maxima; }
        prev_d = d;
        // and the slope is the slope
        double dd = 0.0;
        EXPECT_NEAR(d, (box.Eval(a + 1e-6, dd) - box.Eval(a - 1e-6, dd)) / 2e-6, 1e-5);
    }
    EXPECT_EQ(maxima, 1);
    // ONE dark bin -- about a trunk's shadow -- must survive as a bump worth leaving, and be gone
    // two bins away. At 4 harmonics it was 0.375 high and ~45 deg wide, and the goal stayed put.
    std::vector<double> one(n, 0.0);
    one[7] = 1.0;
    const nm::AzimuthSeries t = nm::AzimuthSeries::Fit(one, az0, daz, nm::kVisFineHarmonics);
    EXPECT_GT(t.Eval(az0 + 7 * daz, d), 0.4);
    EXPECT_LT(std::abs(t.Eval(az0 + 9 * daz, d)), 0.12);
    // A constant comes back exactly: the mean is never windowed.
    const nm::AzimuthSeries c = nm::AzimuthSeries::Fit(std::vector<double>(n, 4.0), az0, daz, nm::kVisFineHarmonics);
    EXPECT_NEAR(c.Eval(0.7, d), 4.0, 1e-12);
}

// With both Stage 2 weights at 0 the field is never read: the problem IS Stage 1, bit for bit,
// whatever garbage the field holds. That is what makes "field absent or stale" a safe state.
TEST(Visibility, ZeroWeightIsExactlyStageOne) {
    std::mt19937 rng(21);
    nm::Problem a = RandomProblem(rng, 18), b = a;
    for (int k = 0; k <= nm::AzimuthSeries::kMaxHarmonics; ++k) {
        b.visibility.occluded.a[k] = std::numeric_limits<double>::quiet_NaN();
        b.visibility.occluded_coarse.a[k] = std::numeric_limits<double>::quiet_NaN();
        b.visibility.clear_r.b[k] = 1e9;
    }
    nm::Solver s;
    const nm::Result ra = s.Solve(a), rb = s.Solve(b);
    ASSERT_TRUE(ra.ok && rb.ok);
    EXPECT_TRUE(ra.horizon == rb.horizon);
}

TEST(Visibility, ClearBearingMovesOutOfAShadowAndDoesNotFlipSides) {
    const int bins = 24;
    const double az0 = -M_PI, daz = 2.0 * M_PI / bins;
    auto wrap = [](double a) { return std::atan2(std::sin(a), std::cos(a)); };
    auto field = [&](double centre, double half) {
        return nm::Visibility::FromBins(
            Bins(bins, az0, [&](double a) { return std::abs(wrap(a - centre)) < half ? 1.0 : 0.0; }),
            Bins(bins, az0, [](double) { return 4.0; }), az0, daz);
    };
    double slope = 0.0;
    int side = 0;
    // Clear where asked: left alone.
    EXPECT_EQ(nm::ClearBearing(field(2.0, M_PI / 6.0), -1.0, side), -1.0);
    EXPECT_EQ(side, 0);
    // Dead centre of a 60 deg shadow, and of a ONE-bin (trunk) shadow: moved out, to a bearing
    // that really is clear, by no more than the shadow's width and a margin.
    for (const double half : {M_PI / 6.0, M_PI / 24.0}) {
        const nm::Visibility v = field(1.0, half);
        side = 0;
        const double az = nm::ClearBearing(v, 1.0, side);
        EXPECT_NE(side, 0);
        EXPECT_LT(v.occluded.Eval(az, slope), 0.15);
        EXPECT_GT(std::abs(wrap(az - 1.0)), half);
        EXPECT_LT(std::abs(wrap(az - 1.0)), half + 30.0 * M_PI / 180.0);
    }
    // Hysteresis, in whole bins (15 deg) because that is how a real field changes. The asked-for
    // bearing is a bin centre in the middle of a shadow, and the goal is already displaced to the
    // + side. The shadow grows by ONE bin on that side (one flickering bin): the - side is now a
    // bin nearer, and the goal must stay where it is. By THREE bins: it goes.
    const double c0 = az0 + 16 * daz;   // a bin centre
    auto lopsided = [&](int extra_bins_on_plus) {
        return nm::Visibility::FromBins(
            Bins(bins, az0, [&](double a) {
                const double d = wrap(a - c0);
                return (d > -daz * 2.5 && d < daz * (2.5 + extra_bins_on_plus)) ? 1.0 : 0.0; }),
            Bins(bins, az0, [](double) { return 4.0; }), az0, daz);
    };
    int s1 = +1;   // already displaced to the + side
    nm::ClearBearing(lopsided(1), c0, s1);
    EXPECT_EQ(s1, +1);
    int s3 = +1;
    nm::ClearBearing(lopsided(3), c0, s3);
    EXPECT_EQ(s3, -1);
    // Dark all the way round: nothing better to offer, so the asked-for bearing stands.
    side = 0;
    EXPECT_EQ(nm::ClearBearing(field(0.0, 4.0), 1.0, side), 1.0);
}

// A person standing still with the trail bearing (behind them, -x) occluded by something 60 deg
// wide. Stage 1 parks the goal in the shadow; Stage 2 must walk it out to where the person can
// be seen, stay in the band, and not drift off round the circle.
TEST(Visibility, GoalLeavesAnOccludedTrailBearingAndStaysInTheBand) {
    const int n = 18, bins = 24;
    const double dt = 1.0 / 15.0, az0 = -M_PI, daz = 2.0 * M_PI / bins;
    const Eigen::Vector3d person(5.0, 0.0, 0.1);
    auto wrap = [](double a) { return std::atan2(std::sin(a), std::cos(a)); };
    const nm::Visibility vis = nm::Visibility::FromBins(
        Bins(bins, az0, [&](double a) { return std::abs(wrap(a - M_PI)) < M_PI / 6.0 ? 1.0 : 0.0; }),
        Bins(bins, az0, [](double) { return 4.0; }), az0, daz);

    for (const double w_vis : {0.0, 10.0}) {   // 10: settles 35 deg off a 30 deg half-width (6: on the edge)
        nm::Solver s;
        Eigen::Vector3d goal(1.5, 0.0, 1.2), goal_prev = goal;
        Eigen::Matrix3Xd prev;
        bool has_prev = false;
        int side = 0;
        for (int tick = 0; tick < 150; ++tick) {
            nm::Problem p;
            p.n = n;
            p.dt = dt;
            p.z_min = 1.2;
            p.az_des = M_PI;
            p.band_lo = 3.25;
            p.band_hi = 3.75;
            p.p_now = person;
            p.g0 = goal;
            p.v0 = (goal - goal_prev) / dt;
            p.target.resize(3, n);
            for (int k = 0; k < n; ++k) { p.target.col(k) = person; }
            p.w.vis = w_vis;
            p.w.clear = w_vis > 0.0 ? 4.0 : 0.0;
            p.visibility = vis;
            p.has_prev = has_prev;
            if (has_prev) {
                p.prev = nm::ShiftHorizon(prev, goal_prev, dt, dt, n);
                p.init = p.prev;
            } else {
                p.init.resize(3, n);
                for (int k = 0; k < n; ++k) { p.init.col(k) = goal; }   // dead centre: the top of the hill
            }
            if (w_vis > 0.0) { p.az_des = nm::ClearBearing(p.visibility, M_PI, side); }   // as the node does
            const nm::Result r = s.Solve(p);
            ASSERT_TRUE(r.ok) << r.why;
            goal_prev = goal;
            goal = r.goal;
            prev = r.horizon;
            has_prev = true;
        }
        const Eigen::Vector2d d = goal.head<2>() - person.head<2>();
        const double off = std::abs(wrap(std::atan2(d.y(), d.x()) - M_PI)) * 180.0 / M_PI;
        double dd = 0.0;
        const double occ = vis.occluded.Eval(std::atan2(d.y(), d.x()), dd);
        std::printf("[nmpc] visibility w_vis %.0f: goal settles %.0f deg off the trail bearing, range %.2f m, "
                    "occluded fraction there %.2f\n", w_vis, off, d.norm(), occ);
        if (w_vis == 0.0) {
            EXPECT_LT(off, 2.0);           // Stage 1: straight behind, in the shadow
        } else {
            EXPECT_GT(off, 30.0);          // out of the 30 deg half-width
            EXPECT_LT(off, 75.0);          // but no further round than it needs to go
            EXPECT_LT(occ, 0.35);
        }
        EXPECT_GT(d.norm(), 3.0);
        EXPECT_LT(d.norm(), 4.0);
    }
}

// The closed loop the node runs: a person walks, turns and stops; each tick warm-starts from
// the last. What the goal_mode exists for is asserted here: a continuous goal, a level one, and
// one that stays on the standoff contract without the hard guards' help.
TEST(ClosedLoop, WalkTurnStopGivesAContinuousLevelGoalInsideTheBand) {
    const int n = 18;
    const double dt = 1.0 / 15.0;
    nm::Solver s;
    Eigen::Vector3d person(5.0, 0.0, 0.1), goal(1.5, 0.0, 1.2), goal_prev = goal;
    double heading = 0.0;
    Eigen::Matrix3Xd prev;
    bool has_prev = false;
    int warm_iters = 0, solves = 0, floor_breaches = 0;
    double max_step = 0.0, max_dz = 0.0, worst_band = 0.0, worst_range = 0.0;
    int worst_tick = -1;
    std::vector<double> ms;
    for (int tick = 0; tick < 450; ++tick) {
        const double t = tick * dt;
        const double speed = t < 22.0 ? 1.0 : 0.0;  // walks, then stops
        if (t > 8.0 && t < 10.0) { heading += (M_PI / 2.0) / 2.0 * dt; }  // 90 deg over 2 s
        const Eigen::Vector3d vel(speed * std::cos(heading), speed * std::sin(heading), 0.0);
        person += vel * dt;

        nm::Problem p;
        p.n = n;
        p.dt = dt;
        p.z_min = 1.2;
        p.az_des = heading + M_PI;
        p.band_lo = 3.25;  // the node's nmpc_band_margin_m inset
        p.band_hi = 3.75;
        p.p_now = person;
        p.g0 = goal;
        p.v0 = (goal - goal_prev) / dt;
        p.target.resize(3, n);
        for (int k = 1; k <= n; ++k) { p.target.col(k - 1) = person + vel * (0.2 + k * dt); }
        p.has_prev = has_prev;
        if (has_prev) {
            p.prev = nm::ShiftHorizon(prev, goal_prev, dt, dt, n);
            p.init = p.prev;
        } else {
            p.init.resize(3, n);
            for (int k = 0; k < n; ++k) { p.init.col(k) = goal; }
        }
        const nm::Result r = s.Solve(p);
        ASSERT_TRUE(r.ok) << r.why << " at tick " << tick;
        ms.push_back(r.solve_ms);
        if (has_prev) { warm_iters += r.iterations; ++solves; }
        goal_prev = goal;
        goal = r.goal;
        prev = r.horizon;
        has_prev = true;
        if (std::getenv("NMPC_TRACE") && tick % 8 == 0) {
            std::printf("t %5.1f range %.2f goal_speed %.2f iters %d cost %.4f lat %.2f\n", t,
                        (goal.head<2>() - person.head<2>()).norm(), (goal - goal_prev).norm() / dt,
                        r.iterations, r.cost,
                        (goal.head<2>() - person.head<2>()).dot(Eigen::Vector2d(-std::sin(p.az_des), std::cos(p.az_des))));
        }
        if (tick > 60) {  // after the start-up transient
            max_step = std::max(max_step, (goal - goal_prev).norm());
            max_dz = std::max(max_dz, std::abs(goal.z() - goal_prev.z()));
            const double range = (goal.head<2>() - person.head<2>()).norm();
            if (std::max(3.0 - range, range - 4.0) > worst_band) { worst_tick = tick; worst_range = range; }
            worst_band = std::max({worst_band, 3.0 - range, range - 4.0});
            if (range < 2.5) { ++floor_breaches; }
        }
    }
    std::sort(ms.begin(), ms.end());
    std::printf("[nmpc] closed loop: %.1f warm iterations/solve, solve p50 %.3f ms p99 %.3f ms "
                "max %.3f ms; max goal step %.3f m, max z step %.4f m, worst band excursion %.3f m (range %.2f m at t=%.1f s)\n",
                static_cast<double>(warm_iters) / solves, ms[ms.size() / 2], ms[ms.size() * 99 / 100],
                ms.back(), max_step, max_dz, worst_band, worst_range, worst_tick * dt);
    EXPECT_LT(max_step, 1.5 * dt * 1.25);  // the goal never jumps: bounded by its velocity limit
    EXPECT_LT(max_dz, 0.01);               // and never bobs
    EXPECT_LT(worst_band, 0.25);           // holds the band through the turn and the stop
    EXPECT_EQ(floor_breaches, 0);
    EXPECT_LT(ms[ms.size() * 99 / 100], 5.0);  // the p99 budget, on an idle box
}
