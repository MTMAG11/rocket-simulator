# Performance

Measured with `rocketsim benchmark` (raw output: `output/benchmarks/latest.json`) on Windows 11, 12 logical cores
(6 physical + SMT), Python 3.11.9, example 29 mm vehicle with a parachute descent (~190 s of flight), machine otherwise idle.

| case | wall | steps | steps/s | peak alloc |
|---|---|---|---|---|
| single, L3 6-DOF, dt 0.01 (descent dt 0.05) | 0.60 s | 5353 | 8.9 k | 7.5 MB |
| single, L3 6-DOF, dt 0.001 (reference quality) | 2.08 s | 16376 | 7.9 k | 16.6 MB |
| single, L2 3-DOF, dt 0.01 | 0.31 s | 3253 | 10.7 k | 3.5 MB |
| single, L2 FAST, dt 0.02 | 0.20 s | 2683 | 13.4 k | 3.2 MB |

| batch (all cores, full telemetry Parquet) | wall | throughput |
|---|---|---|
| 100 sims, L3 6-DOF | 24.0 s | 4.2 sims/s |
| 100 sims, L2 FAST | 7.6 s | 13.2 sims/s |
| 1000 sims, L3 6-DOF | 129 s | 7.8 sims/s |
| 1000 sims, L2 FAST | 50.6 s | 19.8 sims/s |
| 1000 sims windowed dataset, L2 FAST (49.8 MB) | 40.9 s | 24.4 sims/s |

Process RSS at the end of the whole benchmark: 93 MB (memory is bounded by one shard of `shard_runs` runs per worker).

## Reading the numbers honestly

* Parallel efficiency is only ~40-50 % (7.8 sims/s vs ~18 ideal for 6-DOF on 11 workers). Causes, not yet fixed: SMT
  cores do not double throughput, per-run overhead outside the integrator (config parse/validate, motor-file parse,
  config hashing, Arrow conversion of ~100 columns x thousands of rows, Parquet write), and process start-up
  (~2-3 s of imports per worker, significant for the 100-sim rows).
* 100,000 FAST simulations would take roughly 1.5 h on this machine; 100,000 6-DOF about 3.5 h. Sensor-enabled datasets
  are much more expensive for long descents (steps must hit every sensor tick), hence `simulation.stop_after_apogee_s`.
* Optimisations already applied: reuse of the force evaluation between logging and RK4 stage 1 (-20 %), larger step
  during parachute descent (`descent_dt_s`, 4x fewer steps, identical results to the printed precision), no body-axis math
  in 3-DOF, analytic impulse/propellant evaluation, closed-form ISA pressure inversion, Parquet row group per run.
* Next candidates, in order of expected payoff: cache motor/config parsing per worker; vectorised RK4 across many runs
  in NumPy (the dynamics is ~15 floating-point operations per axis; Python call overhead dominates); a compiled core for
  `derivative`. Not done: correctness and validation came first (spec section 70).
