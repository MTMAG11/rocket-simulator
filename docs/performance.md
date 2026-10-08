# Performance

Measured with `rocketsim benchmark` on Windows 11, 12 logical cores (6 physical), Python 3.11.9, the example 29 mm vehicle with a
parachute descent (about 190 s of flight), on an otherwise idle machine. Results are written to `output/benchmarks/`.

## Single runs (physics 1.2.0)

| case | wall | steps/s |
|---|---|---|
| L3 6-DOF, dt 0.01 | 0.60 s | 9.0 k |
| L3 6-DOF, dt 0.001 | 3.94 s | 4.2 k |
| L2 3-DOF, dt 0.01 | 0.66 s | 4.9 k |
| L2 FAST, dt 0.02 | 0.44 s | 6.1 k |

Earlier measurements (before the generic force-coefficient interface) were 2.08 s, 0.31 s and 0.20 s for the last three rows, so
dt-limited runs are now 1.5-2x slower. The interface has not been optimised, and the L2 3-DOF slowdown was not re-run to separate
noise from regression.

## Batch scaling (all cores)

| case | wall | throughput | disk |
|---|---|---|---|
| 1 sim, windowed dataset, L2 FAST | 2.2 s | start-up dominated | 0.1 MB |
| 100 sims | 6.8 s | 14.6 sims/s | 3.4 MB |
| 1000 sims | 50.3 s | 19.9 sims/s | 32 MB |
| 10000 sims | 631 s | 15.8 sims/s | 323 MB |
| 100 sims, L3 6-DOF full telemetry | 19.8 s | 5.1 sims/s | |
| 1000 sims, L3 6-DOF full telemetry | 211 s | 4.7 sims/s | |

Peak RSS 83 MB; memory is bounded by one shard of `shard_runs` runs per worker. Throughput at 10000 runs is about 20 % below the
1000-run rate (more shards, more merge work). Extrapolated, 100,000 FAST windowed runs take about 1.8 h on this machine.

## Notes

* Parallel efficiency is 40-50 %: SMT cores do not double throughput, there is per-run overhead outside the integrator (config
  parse and validation, motor parse, hashing, Arrow conversion, Parquet write), and each worker spends 2-3 s importing.
* Sensor-enabled datasets cost more for long descents because steps must hit every sensor tick; use `simulation.stop_after_apogee_s`.
* Applied optimisations: force evaluation shared between logging and RK4 stage 1 (-20 %), a larger step during parachute descent
  (`descent_dt_s`), no body-axis math in 3-DOF, analytic impulse/propellant evaluation, closed-form ISA pressure inversion, one
  Parquet row group per run.
* Candidates: cache motor/config parsing per worker; vectorise RK4 across runs; a compiled `derivative`.
