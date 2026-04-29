@page feature_sample_mpc Sample-Based MPC (CEM / MPPI)

# Sample-Based MPC (CEM / MPPI)

## Current State
RBPS is a passive simulator: external code calls `World::step` and
inspects state. There is no controller, planner, or trajectory
optimizer. The python integration (see
[python-bindings](../python-bindings/python-bindings.md)) is also
absent. Centerpiece-paper experiments require a planner that can run
*inside* the agent decision loop and call the engine for short
rollouts.

## Motivation
- **Centerpiece paper depends on it.** Direction H ("In-Agent
  Physics") needs a controller that plans by simulating the real
  engine forward, scoring trajectories, and committing the best
  first action. Sample-based MPC fits this exactly.
- **Why sample-based, not gradient-based.** CEM and MPPI need only a
  forward simulator and a cost function. They sidestep the autodiff
  work that differentiable XPBD would require — keeping paper 2
  scope-bounded. Gradient-based MPC remains as a follow-up paper if
  the comparison is worth making.
- **PPO + MPC integration.** With sample-based MPC built outside the
  C++ engine, integration with Python RL frameworks is mechanical
  rather than architectural.
- **Reusable beyond paper 2.** The same controller drives manipulator
  reaching, locomotion gait optimization, and trajectory replay in
  later paper drafts.

## Proposed Approach
**Pure Python package `rbps_control` layered on
[python-bindings](../python-bindings/python-bindings.md). The C++ side
gains nothing; controllers are a learning/control concern, not a
physics concern.**

1. New Python package `python/rbps_control/`:
   - `cost.py` — cost function protocol:
     `Cost(state, action, t) -> float`, plus terminal cost.
   - `cem.py` — cross-entropy method.
   - `mppi.py` — model predictive path integral.
   - `runner.py` — closed-loop driver (apply best action, advance,
     re-plan).
2. **CEM loop.** For horizon `H`, samples `K`, iterations `I`,
   elite fraction `e`:
   ```python
   for it in range(I):
       Us = sample_gaussian(mu, sigma, shape=(K, H, action_dim))
       costs = parallel_rollout(world, snapshot, Us, cost_fn)
       elites = Us[argsort(costs)[:int(K*e)]]
       mu, sigma = fit_gaussian(elites)
   return mu[0]   # first action of the best mean trajectory
   ```
3. **MPPI loop.** Same outer shape, but mean is updated by a
   softmax-weighted average over all samples:
   ```python
   weights = softmax(-costs / temperature)
   mu = (weights[:, None, None] * Us).sum(axis=0)
   ```
4. **Parallel rollouts.** `parallel_rollout` snapshots the world,
   forks K rollouts, and evaluates costs. With
   [multi-world](../multi-world/multi-world.md) it runs across
   threads; without, it loops sequentially. The C++ side exposes a
   `WorldPool` + `parallel_step` to make the parallel path zero-copy.
5. **Warm starting.** At each control step, shift the previous mean
   by one timestep and re-seed sigma — the standard MPPI/CEM warm
   start. Cuts `I` from ~5 to ~2 in steady-state operation.
6. **Action smoothing.** Optional Savitzky-Golay or low-pass filter
   on the committed action sequence — prevents jittery controls
   during deployment.
7. **PPO integration glue** (paper 2 prep): `runner.py` exposes a
   `mpc_action(state) -> action` callable that PPO can wrap as a
   "policy refinement" step.
8. **Tests:** `tests/python/test_cem_pendulum.py` — drive a pendulum
   to upright; `test_mppi_reaching.py` — manipulator reaching; both
   should converge in <50 control steps.

## Risks / Open Questions
- **Sample efficiency.** Wall-clock per control step scales with
  `K · H · substeps`. Budget: at 20 ms per control step, K=64, H=32,
  substeps=8 — needs the C++ engine well under 10 µs per step. The
  benchmark harness validates this is feasible.
- **Snapshot/restore is on the critical path.** If
  [world-snapshot-restore](../world-snapshot-restore/world-snapshot-restore.md)
  doesn't hit its latency target, MPC throughput collapses linearly.
- **Action space discretization.** Sample-based methods assume bounded
  continuous actions. Discrete or hybrid action spaces (e.g. gait
  selection) require extension; out of scope for v1.
- **Non-Markovian costs.** History-dependent costs (e.g. tracking with
  smoothness penalty) need careful indexing across the horizon —
  document the contract clearly in `cost.py`.
