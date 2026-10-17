# Third-party code in `follow_planner`

## `include/follow_planner/thirdparty/lbfgs.hpp` — LBFGS-Lite

| | |
|---|---|
| Upstream | https://github.com/ZJU-FAST-Lab/LBFGS-Lite, `include/lbfgs.hpp` |
| Commit | `a96c50e3fdf8b5278577522a2c5e75766c8670bb` (2022-11-24, the "L-BFGS 2.3" release) |
| SHA-256 of the upstream file | `9fcc11472bed1f4cb468c366ee847148806efebaca3a56b052f8fddacdc5851c` |
| Changes | none to the code; a provenance comment block is prepended |
| Used by | `nmpc_follow.hpp` (the `goal_mode: nmpc` follow-goal solver) |

Header-only, depends on Eigen only, `namespace lbfgs`.

**Why upstream and not SUPER's copy (checked 2026-09-19).** SUPER vendors the same solver as
`super_planner/include/utils/optimization/lbfgs.h` + `src/utils/lbfgs.cpp` (`namespace
math_utils::lbfgs`). 219 of its 261 code lines are verbatim this file; the rest is the
re-namespacing and the .h/.cpp split. But SUPER is LGPL-3.0 (see the header of
`optimization_utils.h`), its copy carries no licence notice at all, and this package is declared
`Proprietary`. Taking the file from its MIT origin at a known commit removes the question.
For the same reason **nothing else is copied from SUPER's `utils/optimization/`**: the smoothed
penalty in `nmpc_follow.hpp` is written from its definition, not lifted from
`optimization_utils.h`.

**If this package is ever linked against SUPER's `libsuper`:** the two copies live in different
namespaces (`lbfgs` here, `math_utils::lbfgs` there), so there is no symbol clash. Re-check if
either side changes its namespace.

### Licence (MIT), verbatim from upstream `LICENSE`

    The MIT License
    
    Copyright (c) 1990 Jorge Nocedal
    Copyright (c) 2007-2010 Naoaki Okazaki
    Copyright (c) 2020-2022 Zhepei Wang
    
    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
