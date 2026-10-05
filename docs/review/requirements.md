# Project requirements (condensed from the owner's 82-section specification)

The numbering follows the original specification sections. "Must" items are explicit in the spec. The critic scores
against THIS list and must flag anything here that appears to under-state the original intent.

**Project context.** Serious physics-based rocket simulation and data-generation system, foundation for an autonomous
rocket guidance/landing system. Vehicle: *solid* motor, predetermined thrust curve, no throttling; control authority via TVC,
aerodynamic surfaces/fins; NOT a Falcon-9-style throttled powered landing. Languages: keep Python unless another
is substantially better (justify). Priority order: correct physics > integration > state representation > data architecture >
validation > testing > performance > visualization > UI polish.

1. Overall goal: launch, ascent, coast, descent, landing/impact; foundation for design analysis, flight prediction, guidance,
   control, sensors, state estimation, HIL, Monte Carlo, dataset generation, NN training, real-data comparison, landing research.
2. Context: solid motor, predetermined thrust, TVC/fins; not Falcon 9.
3. First task: inspect the existing project, make an internal assessment (architecture, capabilities, bugs, physics, missing
   physics, data structures, UI, testing, performance, extensibility, recommended architecture).
4. Physics and data architecture before appearance.
5. Core simulation: time tracking, configurable timestep (0.1, 0.01, 0.005, 0.001), variable timestep if useful, event timestamps.
6. Explicit documented state: position, velocity, acceleration, orientation (roll/pitch/yaw + quaternion internally), angular
   velocity (p,q,r), angular acceleration, masses (total/propellant/dry).
7. Distinct, documented reference frames (Earth/inertial, launch, body, aerodynamic): axes, units, handedness, rotation and angle
   conventions. SI internally.
8. Gravity as an explicit model; constant and g(h)=GM/(R+h)^2.
9. Configurable atmosphere (density, pressure, temperature, speed of sound vs altitude); pluggable models (standard, simplified,
   custom, measured).
10. Wind: constant, altitude-dependent; eventually random profiles/turbulence/gusts/stochastic; applied through relative air velocity
    (v_vehicle - v_wind), never added to ground velocity.
11. Motor: real thrust curves, clean import, `.eng` retained, future formats possible, interpolation, no constant thrust assumption;
    inputs include time, thrust, propellant mass, total impulse, burn time, dry mass.
12. Mass changes during burn; m(t)=dry+remaining propellant; mass flow consistent with the motor; CG, moment of inertia, CP as
    functions of configuration where possible.
13. Configurable rocket geometry (diameter, length, nose, fins, motor location, payload, mass, CG); data architecture must not
    prevent future OpenRocket .ork import (not required now).
14. Aerodynamics: drag from density, relative velocity, reference area, Cd, Cl; Mach-dependent Cd, AoA-dependent Cd, Reynolds
    effects, fin/nose/body/base/induced drag, lift, aerodynamic moments eventually; lookup tables and functions; no constant Cd forever.
15. Mach number computed and logged every step; coefficients can depend on it.
16. Angle of attack derived from orientation, velocity direction and relative wind; affects lift, drag, moments, stability.
17. Stability: CG, CP, static margin; identify unstable configurations.
18. 6-DOF (forces, moments, angular velocity, inertia, orientation), quaternions internally, Euler for display.
19. TVC architecture: thrust direction differs from body axis; max gimbal angle, actuator response, rate, delay, control update frequency;
    ideal first, realistic limits later.
20. Control system separated from the physics engine (state -> controller -> command -> physics); pluggable controllers (PID, state
    feedback, trajectory following, guidance, custom, NN).
21. Sensors: true state != measured state; IMU/accelerometer/gyro/barometer/GPS/magnetometer; noise, bias, drift, update rate,
    quantization, latency, saturation.
22. State estimation (complementary/Kalman/EKF) with truth -> sensors -> estimator -> estimated state; no fake filters.
23. Flight phases (pre-launch, ignition, powered ascent, burnout, coast, apogee, descent, landing), configurable state machine, logged transitions.
24. Launch detection from sensor data (eventually).
25. Ground and impact: detect impact, impact velocity/time, stop or post-impact state, never underground; eventually terrain/landing zones/non-flat terrain.
26. Monte Carlo: randomise mass, Cd, thrust, launch angle, wind, atmosphere, sensor noise, actuator performance, initial conditions,
    manufacturing tolerances; unique simulation ID and stored random seed; reproducible.
27. Mass data generation: 10 .. 100,000+ simulations, headless, no GUI needed; config -> parameters -> batch -> validation -> export.
28. Export: CSV, JSON, Parquet if practical, NumPy formats, binary if needed.
29. Documented, versioned data schema with simulation metadata, vehicle state, environment, flight, control, sensors, truth columns.
30. ML data pipeline: general dataset-generation interface (inputs: sensors/previous states/environment/control; targets: position/velocity/
    attitude/acceleration/landing location/corrections/control outputs); separate train/val/test seeds; avoid leakage and near-identical distributions.
31. User-configurable feature/label definitions without modifying simulation code.
32. Windowed time-series datasets (last N steps -> future state or trajectory), efficient.
33. After the simulator is complete: major validation phase with public real rocket flight data; assess source quality; document source,
    rocket, motor, mass, atmosphere, telemetry, sampling, uncertainties.
34. Real-data comparison: reconstruct inputs, run, compare altitude/velocity/acceleration, apogee, burnout, duration, descent, max v/a.
35. Quantitative error: RMSE, MAE, max abs error, percentage error, timing error.
36. Calibration with physical justification only; investigate causes; no arbitrary constants.
37. Calibration loop (simulate, compare, identify, cause, modify, re-run, measure, document); do not overfit; validate on additional flights.
38. Mandatory critic/reviewer: independent assessment of technical correctness, software quality, requirements compliance, data capability, validation.
39. Critic scoring 0-10 with strengths, weaknesses, missing requirements, incorrect assumptions, physics/software/data/validation problems, exact changes needed.
40. Three critic iterations (build, critic, fix, critic, fix, critic); no artificially increased scores; investigate decreases.
41. Critic must be requirement-based: PASS/PARTIAL/FAIL per requirement, then a score; a good UI alone must not earn 9-10.
42. Automated tests (gravity, thrust, mass depletion, drag, integration, coordinate transformations, atmosphere, Mach, impact, phase transitions,
    determinism, export) using analytical solutions; the simulator must not be its own only source of truth.
43. Numerical validation across timesteps (0.1, 0.01, 0.001), convergence, documented numerical error.
44. Performance: headless, multiprocessing, parallel, memory-efficient, batch, streaming, checkpointing; GUI not involved in bulk generation.
45. CLI: simulate, batch, validate, export, benchmark (and generate-dataset).
46. Config files (YAML/JSON/TOML): rocket, motor, atmosphere, wind, launch, timestep, physics options, sensors, controller, dataset generation, output.
47. Reproducibility: store configuration, software version, seed, simulator version, physics model version, schema version.
48. Versioning of config, schema, physics model, exported datasets; no silent physics changes.
49. GUI: altitude, velocity, acceleration, thrust, mass, drag, Mach vs time, flight path, orientation; graph selection, zoom, timeline scrubbing, events.
50. 3-D visualisation (secondary): rocket, trajectory, orientation, ground, wind direction, phases.
51. Run inspection: max altitude/velocity/acceleration, burnout time, apogee time, impact velocity, flight time, max Mach, landing position, others.
52. Data browser: inspect datasets/metadata without loading every timestep.
53. Error handling: useful errors for negative mass, missing motor, invalid timestep/geometry/atmosphere, malformed motor; no silent physics failures.
54. SI units internally; units on every user-facing quantity.
55. Documentation: architecture, physics, equations, coordinate systems, data schema, configuration, CLI, dataset generation, validation, testing; assumptions; simplified models labelled.
56. Clean separation: physics, vehicle, environment, motor, simulation, control, sensors, estimation, data, validation, UI, CLI, tests, configuration.
57. Pipeline architecture: config -> vehicle -> environment -> motor -> physics -> true state -> sensors -> estimation -> controller -> actuators -> physics.
58. Future NN integration: realistic randomised training environments; do not train a network yet.
59. Hardware-in-the-loop must be possible (not necessarily implemented).
60. Standardised telemetry representation resembling flight-computer telemetry.
61. Real telemetry import (CSV/JSON/...) mapped to the standard representation, with source metadata.
62. Simulation-vs-real comparison tool with plots (altitude, velocity, acceleration) and numerical metrics.
63. No fake precision; uncertainty made clear.
64. Model fidelity hierarchy (levels 0-6) allowing fast low-fidelity models.
65. Fast mode for dataset generation (simplified atmosphere/aero); fidelity level recorded in datasets.
66. High-fidelity mode prioritising accuracy for validation/analysis/controller verification.
67. Dataset quality: reject NaN, exploding states, impossible velocity, invalid orientation, crashes, missing telemetry; bad runs never enter datasets.
68. Dataset manifest: id, date, simulator/physics versions, configuration, counts, seed info, fidelity, feature schema, label schema.
69. Checkpointing/resume for long dataset generation.
70. Parallelisation where appropriate; correctness first, then benchmark, then optimise.
71. Benchmarks: single sim, 100, 1,000, dataset generation; sims/s, steps/s, memory.
72. Production-quality code: type checking, linting, formatting, unit and integration tests, documentation; no needless abstraction.
73. Staged development order (inspect, physics, 3-DOF, 6-DOF, aero/stability, sensors, estimation, control, batch, dataset, validation, critic cycles).
74. No fake features: honest simplified models and clean APIs rather than box-ticking.
75. Source quality for external information (NASA, agencies, universities, papers, documentation).
76. Physics references documented per major model: equation, assumptions, source, limitations.
77. Validation hierarchy: analytic, unit tests, known calculations, independent simulators, real data, multiple flights.
78. Independent cross-check against reputable tools (OpenRocket etc.); investigate disagreements.
79. Final acceptance checklist (clean install, preserved functionality, physics/integration tests, motor/atmosphere/wind/aero/3-DOF/6-DOF,
    export, batch, seeds, dataset, schema documented, CLI, GUI, reproducibility, real-world validation, errors quantified, calibration documented,
    three critic iterations, final score recorded).
80. Final engineering report: architecture, physics, assumptions, validation, accuracy, testing, performance, data, ML readiness, limitations, roadmap. Do not claim "NASA-level" without evidence.
81. Most important rule: do not optimise for saying "Done"; say what is wrong; replace flawed architecture; be honest.
82. Workflow: inspect, decide, build, test, validate, optimise, batch, export, real data, sim-vs-real, calibrate, critic x3 with fixes, final validation, final report.
