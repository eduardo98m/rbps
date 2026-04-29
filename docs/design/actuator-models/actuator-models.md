@page feature_actuator_models Actuator Models

# Actuator Models

## Current State
[include/rbps/constraints/Joint.hpp](../../../include/rbps/constraints/Joint.hpp)
exposes a joint-actuation enum `FREE / SPEED / POSITION`. The current
implementation drives joints toward a target with no gain tuning, no
torque limit, no velocity limit, and no concept of motor friction or
backlash. The corresponding solver lives in
[src/rbps/constraints/Joint.cpp](../../../src/rbps/constraints/Joint.cpp).
No PD controller, no effort/torque mode, no per-joint configuration
beyond the target value.

## Motivation
- **URDF requires it.** URDF actuator tags specify
  `effort_controller`, `position_controller`, and `velocity_controller`
  with explicit `max_effort`, `max_velocity`, and gain parameters.
  Without them, a loaded URDF cannot match the source robot's behavior.
- **Robotics MPC needs torque limits.** A planner that ignores torque
  saturation produces infeasible trajectories that fail on real hardware
  and look unrealistic in sim. Paper 2's centerpiece result depends on
  this.
- **Sim-to-real transfer.** Motor saturation is the most common cause
  of policy transfer failure. Modeling it during training is the
  cheapest sim-to-real intervention.
- **Reviewers will check.** Standard manipulation / locomotion
  benchmarks ship with calibrated actuator parameters; not honoring
  them invalidates direct comparisons.

## Proposed Approach
**Extend the existing joint actuator enum into a parametrized
controller, applied via velocity-level corrections to preserve
XPBD semantics.**

1. New struct in [include/rbps/constraints/Joint.hpp](../../../include/rbps/constraints/Joint.hpp):
   ```cpp
   struct ActuatorParams {
       enum Mode { Free, Position, Velocity, Effort } mode = Free;
       scalar target          = 0;       // q_target, v_target, or τ
       scalar kp              = 0;       // position gain
       scalar kd              = 0;       // velocity gain (also damping in Free mode)
       scalar max_effort      = INFINITY;
       scalar max_velocity    = INFINITY;
   };
   ```
   Add an `actuator` field per joint in `JointCollection`.
2. **Controller laws** (applied each substep, after constraint solve,
   before velocity-level friction):
   - **Position:** τ = clamp(kp · (target − q) − kd · q̇,  ±max_effort).
   - **Velocity:** τ = clamp(kd · (target − q̇),          ±max_effort).
   - **Effort:**   τ = clamp(target,                      ±max_effort).
   - **Free:** τ = −kd · q̇ (passive damping only; kd defaults to 0).
3. **Velocity clamp.** After applying τ via impulse, clamp |q̇| to
   `max_velocity` — preserves XPBD energy bounds and prevents windup.
4. **Application path.** Translate τ into an angular impulse aligned
   with the joint axis and apply via the existing
   `compute_rotational_constraint_impulse` helper. No new constraint
   solver code required.
5. **URDF mapping.** [scene-format](../scene-format/scene-format.md)'s
   loader translates URDF transmission / actuator tags into
   `ActuatorParams`. Default mapping: position controller with the
   URDF `<dynamics damping>` becoming `kd`.
6. **Tests:** `tests/rbps/test_actuator_position.cpp` (drive a 1-DoF
   joint to a target, verify settling time matches an analytic
   spring-damper); `test_actuator_torque_limit.cpp` (verify clamp);
   `test_actuator_velocity_limit.cpp`.

## Risks / Open Questions
- **High-gain instability.** Aggressive `kp` with small substep counts
  destabilizes the joint. Document a recommended `kp · h² ≤ 1` rule of
  thumb and emit a warning if violated.
- **Coupling with joint compliance.** PD control acts on top of the
  XPBD joint constraint, which itself has a compliance term. Their
  interaction is non-trivial; the validation harness from
  [articulated-bodies-xpbd](../articulated-bodies-xpbd/articulated-bodies-xpbd.md)
  should regression-test against pinocchio with the same gains.
- **Asymmetric limits.** Some real motors have asymmetric torque
  envelopes. Defer to a `min_effort` / `max_effort` pair if a paper
  requires it; v1 stays symmetric.
- **Friction / backlash / static stiction.** Out of scope for v1; emit
  ideal-actuator behavior. Add a `JointFriction` struct as a follow-up
  if reviewers ask.
