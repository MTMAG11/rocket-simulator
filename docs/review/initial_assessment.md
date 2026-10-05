# Assessment of the project as received (before any change)

Inspected: every tracked file (`README.md`, `pyproject.toml`, `src/rocket_sim/{config,motor,physics,plotting,simulation,state}.py`,
`ui/main_window.py` (1051 lines), `ui/styles.py`, four `.eng` files). No tests, no CLI, no docs beyond the README.

| aspect | finding |
|---|---|
| **Architecture** | Flat package; `simulation.py` runs a single hard-coded loop; physics is one function mutating a `RocketState`; UI imports the backend directly and works on a dict of Python lists. |
| **Capabilities** | 2-D (x, y) thrust + gravity point-mass flight from an `.eng` file; peak-value summary; matplotlib/PySide6 plots of altitude, velocity, acceleration, thrust, TWR with a time slider. |
| **Physics implemented** | Constant gravity; thrust along a fixed launch angle; mass from impulse-fraction burn. **Missing**: drag, atmosphere, wind, ground, attitude/rotation, aerodynamics, stability, sensors, estimation, control, TVC, recovery. |
| **Bugs** | (1) no ground contact: after a ballistic descent the rocket flew underground until a hard-coded `burn_time + 20 s`; (2) "apogee" triggered at the first `vy <= 0`, including before liftoff if thrust < weight; (3) thrust angle was fixed in the inertial frame (never followed the vehicle); (4) mass kept in grams, converted to kg in the physics, motor header kg multiplied by 1000 (units churn, easy to break); (5) semi-implicit Euler with the mass updated *after* the force evaluation (one-step-lagged mass); (6) `O(n^2)` re-integration of the thrust curve at every step for propellant mass; (7) `rocketpy` imported only to parse `.eng` (a heavy dependency for 30 lines of parsing); (8) `max_acceleration` reset logic ignored descent; no input validation at all. |
| **Data structures** | `RocketState` dataclass with x, y, vx, vy, ax, ay; results as dict of lists; no schema, no versioning, no metadata, no seed. |
| **UI** | Substantial and useful as a concept (dark theme, graph selector, time cursor, summary) but tightly coupled to the old result dict; a "3D View: coming later" placeholder. |
| **Testing** | None. |
| **Performance** | Pure-Python lists; acceptable for one 1-D run, but a per-step `O(n)` motor lookup and no headless/batch mode. |
| **Extensibility** | Low: adding wind/drag/6-DOF would mean rewriting `simulation.py` and the UI. |

## Decision

* **Language: keep Python.** The workload is many independent short flights (10^3-10^4 steps each); parallelism across processes
  is where throughput comes from, and the ML/data stack is Python. A C++/Rust rewrite would add FFI and build complexity without
  addressing a measured bottleneck (see docs/architecture.md).
* **Architecture: replace the core, keep the ideas.** New package layout with explicit frames, quaternion 6-DOF dynamics,
  environment/vehicle/motor models, an event-driven loop, a data layer and tooling. Preserved: `.eng` motor files, the "motor data
  separate from physics" idea, the dark-theme GUI concept (rebuilt as a thin client of the new engine).
* **Keep what is worth keeping:** motor files, styling; discard: state dataclass, result dict, loops and the rocketpy dependency.
