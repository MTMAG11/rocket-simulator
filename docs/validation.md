# Validation

Validation hierarchy followed (spec section 77): analytic physics -> unit tests -> known rocket calculations ->
independent simulator -> real flight data -> multiple real flights. Numbers below are reproducible:

```bash
rocketsim validate validation_data/bella_lui.yaml --out validation_results/physics_v1.1.0
rocketsim validate validation_data/ndrt_2020.yaml --out validation_results/physics_v1.1.0
rocketsim validate validation_data/prometheus.yaml --out validation_results/physics_v1.1.0
python validation_data/crosscheck_rocketpy.py
```

## 1. Analytic and unit validation

`tests/` (160 tests) checks against closed-form solutions, not against the simulator itself: free fall, vacuum
projectile range/apogee, the Tsiolkovsky rocket equation with gravity (burnout velocity and altitude to 1e-8 relative),
quadratic-drag fall (`tanh`/`ln cosh`), wind through relative velocity (algebraic relaxation law), torque-free
axisymmetric precession (Euler equations, 1e-6), constant-rate rotation, small-angle weathercock oscillation frequency
(`omega_d`, 1 %), CP-ahead-of-CG divergence, tail-first instability, TVC torque, ISA vs the published 1976 table at all
layer boundaries, motor impulse by independent quadrature, integrator order of accuracy (1, 2, 4), quaternion/DCM
identities, convergence in dt. Details: [testing.md](testing.md).

## 2. Independent cross-check: RocketPy

`validation_data/crosscheck_rocketpy.py` runs RocketPy 1.13 and this simulator on an identical vehicle (Prometheus
geometry/mass, M1520 thrust curve, Cd = 0.45 constant, standard atmosphere, variable gravity, no wind, ends at apogee):

| launch elevation | quantity | this sim | RocketPy | diff |
|---|---|---|---|---|
| 90 deg | apogee [m] | 3768.81 | 3775.17 | -0.17 % |
| 90 deg | apogee time [s] | 27.30 | 27.34 | -0.18 % |
| 90 deg | max speed [m/s] | 319.09 | 319.18 | -0.03 % |
| 90 deg | rail-exit speed [m/s] | 27.06 | 27.15 | -0.32 % |
| 80 deg | apogee [m] | 3613.58 | 3620.09 | -0.18 % |
| 80 deg | horizontal range at apogee [m] | 1299.6 | 1307.1 | -0.57 % |

Both codes agree to < 0.6 % on everything including the weathercocking trajectory; static margin agrees (1.0 cal).
Residual differences are consistent with different gravity/atmosphere implementations and integrator tolerances. This
shows the *dynamics* agree; it says nothing about drag-model accuracy because Cd is prescribed. (OpenRocket was not
available in this environment; RocketPy is an independent established tool, and neither is assumed to be correct.)

## 3. Real flight data

Sources, selected for having both telemetry and published vehicle parameters. All three come from RocketPy's public
example data (MIT-licensed repository); the original teams gave permission for the data to be used. Assessed quality:
**university-team amateur flights, altimeter-grade telemetry, vehicle parameters from a secondary source (RocketPy
documentation)**. They are *not* NASA-grade ground truth.

| flight | rocket / motor | vehicle | telemetry | sampling | known uncertainties |
|---|---|---|---|---|---|
| EPFL Rocket Team *Bella Lui*, Kaltbrunn, 22 Feb 2020 | AeroTech K828FJ (54 mm) | 156 mm dia, 2.68 m, liftoff 19.6 kg, 3 fins, drogue only | altitude AGL + vertical velocity (filtered), 767 samples | ~15 Hz (irregular) | atmosphere (ERA5 not available), wind, boat-tail not modelled, filtered velocity lag |
| Notre Dame Rocket Team, Three Oaks MI, 23 Feb 2020 | Cesaroni L1395 (75 mm) | 203 mm dia, 3.39 m, liftoff 23.3 kg, 3 fins, drogue + main | altitude (ft AGL) ~20 Hz + axial accelerometer 400 Hz (burn only, 4.5 s); team-reported apogee 4320 ft | 20 Hz / 400 Hz | **two-diameter airframe (203 -> 155 mm) not representable**, wind/atmosphere unknown, accelerometer scale unknown (reads 0.8-0.9 g at rest) |
| *Prometheus*, Spaceport America, 24 Jun 2022 | Cesaroni M1520 (98 mm) | 139.7 mm dia, 2.23 m, liftoff 20.65 kg, 3 fins, drogue + main | AltOS TeleMetrum height AGL (barometric, lags in boost) + *total* speed (Kalman-filtered), pad pressure 86444 Pa | ~25 Hz | wind and temperature profile unknown; Von Karman nose approximated by an ogive |

Reconstruction rules (documented in each `validation_data/*_sim.yaml`): geometry/mass/chutes from the team parameters;
motor = ThrustCurve.org thrust curve of the actual motor; atmosphere = ISA with the site elevation and a temperature
offset that is a *climatological estimate* (-8 K, -12 K, +15 K; not tuned), except Prometheus whose sea-level pressure is
derived from the *measured* pad pressure (verified: ISA with the +15 K offset and this sea-level pressure gives 86444 Pa at 1401 m; an earlier version of this file used a value that gave 87180 Pa, found by the critic review); published inertias were implausible for two vehicles (e.g. Iyy 0.78 kg m^2 for a
2.7 m, 19.6 kg rocket) so a thin-tube estimate is used there; unknown fin thickness assumed 3-4 mm. Time alignment: the
simulation is shifted so both cross 15 m altitude at the same instant (removes only the launch-detect offset).

### Altitude methodology (important; found by the second critic review)

All three real altimeters are **barometric**. Earlier versions of this document compared them with the simulation's
*geometric* height, which hides the effect of the real atmosphere's temperature on a pressure altimeter. The comparison
now converts the simulated static pressure to the altitude a standard-atmosphere barometer would report
(`validation/telemetry.py::_baro_equivalent_altitude`, pad pressure as reference) -- like with like. Because the day's
temperature profile is unknown, the temperature offsets (-8 / -12 / +15 K) are *estimated inputs that act as hidden free
parameters*: +-10 K moves the barometric apogee by roughly 2-3.5 points per side (table below). Geometric-height comparisons are available
with `telemetry.altitude_reference: geometric` and gave -2.3 / +8.7 / +0.03 %.

### Results, physics v1.1.0 (nominal inputs, no fitted constants; model structure changed after seeing Prometheus)

| metric | Bella Lui | NDRT 2020 | Prometheus |
|---|---|---|---|
| apogee, real / sim (barometric) [m] | 459.0 / 461.3 | 1320.4 / 1497.7 | 3903.8 / 3712.6 |
| **apogee error** | **+0.50 %** | **+13.43 %** | **-4.90 %** |
| apogee time error | -1.02 % (-0.10 s) | +5.25 % (+0.85 s) | -5.60 % (-1.62 s) |
| max velocity error | -5.73 % | +13.11 % (derived from altitude) | -0.26 % (vertical velocity) |
| max axial acceleration error | n/a | +9.68 % (sensor scale unknown) | n/a |
| burnout time error | n/a | -1.87 % (-0.06 s) | n/a |
| landing time error | +1.29 % | +16.74 % | -4.42 % |
| altitude RMSE / MAE / max / bias [m] | 3.6 / 3.0 / 9.0 / +1.2 | 130.8 / 117.3 / 218.9 / +117.3 | 141.1 / 122.4 / 415.5 / -45.8 |
| altitude NRMSE (of range) | 0.77 % | 9.89 % | 3.61 % |
| ascent-only altitude RMSE [m] | 3.3 | 116.5 | 146.8 |
| velocity RMSE / bias [m/s] | 3.4 / -2.3 | 14.3 / +10.2 | 10.7 / -9.4 |

RMS apogee error over the three flights: **8.3 %** (mean absolute 6.3 %). "Barometric" is certain only for AltOS (documented); the Bella Lui and NDRT altimeters are *assumed* barometric, as nearly all hobby altimeters are. Plots: `validation_results/physics_v1.1.0/*.png`.
The Prometheus time alignment uses the 50 m/s crossing of the velocity channel because AltOS height is barometric and
lags during boost (avionics-bay pressure lag); this choice was made after seeing the altitude lag (a judgement call).
The AltOS `speed` column is **signed vertical velocity** (it reaches -38 m/s on descent and 0 at apogee; an earlier
version of this document wrongly called it total speed, found by the third critic review). Its time integral to apogee is
4175 m versus a barometric apogee of 3904 m. Together with the -9.4 m/s velocity bias this means the simulated
coast decelerates too fast and the real *geometric* apogee was probably ~4.1-4.2 km, i.e. ~6 % above the simulation: the
earlier geometric "+0.03 %" for Prometheus was a coincidence of two errors, not agreement.

### Baseline (v1.0.0) and what changed

Baseline numbers (geometric altitude comparison, the methodology of the time): Bella Lui -4.37 %, NDRT +0.19 %, Prometheus
-11.14 %. v1.1.0 under the same geometric comparison: -2.29 / +8.71 / +0.03 % (corrected Prometheus pressure). With the
barometric comparison the model-change evidence is weaker: see the table above (+0.5 / +13.4 / -4.9 %).

Calibration loop (spec 37):

1. **Simulate / compare**: Prometheus apogee 11 % low (geometric) and 3.5 s early although max velocity was right.
2. **Identify**: too much drag after burnout at high subsonic speed.
3. **Physical cause**: the team's RASAero Cd(M) falls 0.42 -> 0.30 over M 0.1-0.85 while this model's rose 0.45 -> 0.55;
   the Barrowman subsonic base-drag term `0.12 + 0.13 M^2` accounted for the whole rise; Hoerner's relation
   `0.029/sqrt(Cd_forebody)` is Mach-independent subsonically; the roughness floor had omitted compressibility.
4. **Modify**: replace those terms with literature models (no fitted constant), for all vehicles.
5. **Re-run / measure**: tables above.
6. **Do not overfit**: the change was motivated by Prometheus, so Prometheus is **in-sample**; Bella Lui improved (geometric)
   and NDRT worsened (+0.2 -> +8.7 % geometric). Three flights cannot establish generalisation, and the barometric
   comparison shows errors of +0.5 / +13.4 / -4.9 %, i.e. no flight is predicted better than ~5 % except Bella Lui.

**Reading the NDRT result.** NDRT is over-predicted by 13 % (geometric 8.7 %). The vehicle has a two-diameter airframe
this model cannot represent, the thrust-curve sensitivity (below) shows a -5 % total impulse would nearly remove the
error (within certification tolerance, but **untestable here**: no motor impulse measurement exists), and the winter
temperature offset is a guess. These are candidate explanations, not findings.

### Sensitivity of apogee error (barometric comparison)

| input change | Bella Lui | NDRT | Prometheus |
|---|---|---|---|
| nominal | +0.50 % | +13.43 % | -4.90 % |
| temperature offset -10 K / +10 K | +3.86 / -2.66 % | +16.51 / +10.53 % | -3.07 / -6.65 % |
| thrust x0.95 / x1.05 | -9.42 / +10.75 % | +4.76 / +22.11 % | -10.01 / -0.13 % |

A +-5 % impulse variation moves apogee by ~+-10 %, and +-10 K by 3-6 points: the unknown inputs are comparable to the
model error, so individual flight errors cannot be attributed to the drag model.

### Leave-one-out calibration of a global drag multiplier (`validation_results/loo_drag_scale.txt`)

| held-out flight | fitted on | fitted `drag_scale` | apogee error: nominal | calibrated |
|---|---|---|---|---|
| Bella Lui | NDRT, Prometheus | 1.133 | +0.50 % | -0.75 % |
| NDRT | Bella Lui, Prometheus | 0.879 | +13.43 % | +17.67 % |
| Prometheus | Bella Lui, NDRT | 1.430 | -4.90 % | -17.08 % |

RMS held-out error: nominal 8.26 %, calibrated 14.19 %. **Verdict: not supported; the model stays uncalibrated** (the
protocol in `rocket_sim.validation.calibrate` only accepts a calibration that improves held-out flights).

## 4. What this validation does and does not show

* Shows: an uncalibrated first-principles model predicts apogee of three 20 kg-class rockets (459 m to 3.9 km, Mach up to
  ~0.9) to +0.5 / +13 / -5 % (barometric, RMS 8 %), burnout timing to ~2 % (one flight), and agrees with an independent simulator to < 1 %.
* Does not show: supersonic accuracy (no flight exceeded Mach 0.95), small model rockets, descent-rate accuracy (chute
  data are assumed, landing time errors up to 17 %), accelerometer-level fidelity (only one burn-phase record, unknown
  scale), 6-DOF attitude fidelity (no attitude telemetry available), wind response (no wind data), sensor models,
  estimator performance on real data, or anything about TVC/control (no real TVC flight data was found or used).
* Quality of ground truth: secondary-source parameters, filtered altimeter outputs, unknown atmosphere. Errors of a few
  percent cannot be attributed to the model with certainty.

## 5. Next validation steps

Flights with measured atmosphere (radiosonde/weather station) and wind, weighed-in vehicles and motors, raw
(unfiltered) IMU data including attitude, a supersonic flight, and ideally a TVC flight; a second independent code
(OpenRocket/RASAero) on the same Cd tables; a laminar/turbulent roughness study.
