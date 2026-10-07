# Validation

Validation hierarchy followed: analytic physics -> unit tests -> known-rocket calculations -> independent simulator ->
real flight data -> multiple real flights **with a held-out set**. Everything below is reproducible:

```bash
rocketsim validate-registry --split development --out validation_results/registry_dev
rocketsim validate-registry --split calibration --out validation_results/registry_cal
rocketsim validate-registry --split holdout --confirm-frozen          # logged; see section 3
python validation_data/crosscheck_rocketpy.py                         # trajectory cross-check (RocketPy)
python validation_data/crosscheck_geometry.py                         # CP / CNalpha cross-check (RocketPy)
```

## 0. How to read the numbers (read this first)

* **Every flight carries a split label.** *Development* flights shaped the model (structure was adjusted after seeing
  them); agreement on them is **in-sample** and is never quoted as evidence. *Calibration* flights may be used to fit
  physically bounded shared parameters. *Holdout* flights were never used for either and are evaluated once, with the
  model frozen, through a logged protocol (section 3).
* **All ground truth is Tier 2 or 3** (university-team flight computers / COTS altimeters, parameters from a secondary
  source). None of the flights has measured wind, a measured temperature profile, verified masses or motor impulse. An
  apogee error of a few percent is **not** attributable to the physics (section 6 quantifies how much the unmeasured
  inputs alone move apogee: 3-7 % one sigma, depending on the vehicle).
* **No small-rocket (<1 kg) validation data exist in this repository.** This is an open gap (section 7), not a result.
* The simulator is **not** validated for supersonic flight (no flight exceeded ~Mach 0.9 in the simulation), attitude
  dynamics, sensors on real data, or TVC/fin control on real data.

## 1. Flight registry, tiers and splits

`validation_data/registry.yaml` is the single source of truth. Source tiers: 1 = instrument-grade, measured inputs;
2 = team flight computer / COTS altimeter with team-supplied parameters; 3 = sparse or undocumented sensor; 4 = prose or
summary only (not usable for metrics). **No Tier 1 data were available.**

| flight | split | tier | motor | liftoff mass | telemetry | notes |
|---|---|---|---|---|---|---|
| EPFL *Bella Lui* 2020 | development | 2 | AeroTech K828FJ | 19.6 kg | altitude + vertical velocity, ~15 Hz | boat-tail not modelled |
| Notre Dame 2020 | development | 2 | Cesaroni L1395 | 23.3 kg | altitude 20 Hz + axial accel 400 Hz | two-diameter airframe not representable by the single-diameter config; accel scale unknown |
| *Prometheus* 2022 | development | 2 | Cesaroni M1520 | 20.7 kg | AltOS TeleMetrum height + signed vertical velocity | model structure (base drag, roughness) was changed after seeing this flight |
| *Genesis* (EuRoC 2023) | calibration | 2 | Cesaroni L995 | 11.2 kg | team flight computer altitude, 100 Hz | |
| *Astra* (EuRoC 2022) | calibration | 3 | Cesaroni L1350 | 12.7 kg | altitude, ~1 Hz, sensor undocumented | timing and RMS are not meaningful at 1 Hz |
| *Erebus 11* (EuRoC 2022) | **holdout** | 2 | ProK54 (curve from RocketPy file) | 7.0 kg | COTS altimeter altitude, 20 Hz | propellant mass from grain geometry; boat-tail modelled |
| *Cavour* (Polito, EuRoC 2023) | **holdout** | 2 | Cesaroni L995 | 10.2 kg | altimeter altitude, ~6 Hz (the file's velocity column is not used; v_max is derived from altitude) | the YAML had a quoting error in its column name (`altitude[m]`) fixed minutes before this flight's holdout run; no model input changed |

Excluded (documented in the registry): hybrid/liquid flights (Defiance, Halcyon, Hedy), Lince (mid-ascent mass-change event),
and four larger flights not yet reconstructed. The arXiv 1708.01970 Estes paper has no usable time series (Tier 4).

**Reconstruction conventions, fixed before any calibration/holdout flight was simulated**: geometry/mass/inertia from the
RocketPy notebook of each flight; RocketPy's `mass` is taken to *include* the empty motor casing (so the motor `.eng`
copies used here have total weight = propellant weight); von Karman noses are approximated by tangent ogives; fin sweep =
root - tip; fin thickness 3 mm; no wind; ISA +8 K; launch 84 deg, rail 12 m (the RocketPy example values - not measured);
ascent-only comparison for flights with undocumented parachutes.
The order of events (conventions and split written down, then calibration flights, then the calibration record, then the holdout
runs) is attested by the process and by the timestamps in `holdout_log.jsonl`; it is **not independently timestamped** (the
repository state is uncommitted), so a reader has to trust it. The calibration flights were seen before the holdout ones, but
no model change was made in between.

**The mass convention matters as much as any model choice**: adding the .eng casing mass on top of the reported mass moves
apogee by **-12 % (Genesis), -9.6 % (Cavour), -10 % (Astra)** (`validation_results/sensitivity_error_budget.json`).

Time alignment: the simulation is shifted so both cross 15 m (barometric-equivalent altitude); this removes only the
launch-detect offset. The applied shift is printed in every report and is <= 0.5 s for all flights. Altitude comparison:
simulated static pressure -> standard-atmosphere altimeter reading (like-for-like with a barometric altimeter).

**Alignment diagnostic** (reported by every comparison; scan of +-1.5 s extra shift on ascent altitude RMSE). Best extra shift /
RMSE change: Bella Lui +0.03 s (3.3 -> 3.3 m); NDRT +0.83 s (116 -> 85 m); Prometheus +0.60 s (147 -> 113 m); Genesis +0.13 s
(50 -> 46 m); Astra -0.63 s (102 -> 28 m, 1 Hz data). The applied alignment is not tuned to this; the residuals show that for
NDRT and Prometheus part of the altitude RMSE is timing/shape error of the boost phase, not a pure offset. (Not run for the
holdout flights: that would be a logged re-evaluation, and the diagnostic does not change any apogee number.)

## 2. Results (physics 1.2.0 numbers; physics 1.2.1 changed only fin-controlled flights, see physics.md change log; nominal inputs, nothing fitted)

| flight | split (label) | apogee real / sim [m] | **apogee error** | t_apogee error | v_max error | ascent altitude RMSE / bias [m] |
|---|---|---|---|---|---|---|
| Bella Lui | development (in-sample) | 459 / 461 | **+0.50 %** | -1.02 % | -5.7 % | 3 / +2 |
| NDRT 2020 | development (in-sample) | 1320 / 1497 | **+13.41 %** | +5.25 % | +13.1 % | 116 / +102 |
| Prometheus | development (in-sample) | 3904 / 3712 | **-4.90 %** | -5.60 % | -0.3 % | 147 / +17 |
| Genesis | calibration | 2917 / 2843 | **-2.54 %** | -2.26 % | -14.0 % | 50 / -8 |
| Astra | calibration | 3249 / 3219 | **-0.93 %** | +2.82 % | n/a | (1 Hz data) |
| Erebus 11 | **HOLDOUT** | 3002 / 2810 | **-6.40 %** | -1.95 % | **-25.0 %** | 124 / -110 |
| Cavour | **HOLDOUT** | 2789 / 2915 | **+4.52 %** | -2.39 % | -8.1 % | 107 / +92 |

RMS apogee error: development (in-sample) 8.25 %; calibration 1.91 %; **holdout 5.54 % (n = 2)**; the four flights not
used for development (calibration + holdout) 4.1 %. Two holdout flights cannot establish a generalisation claim; they
show only that the nominal model is not wildly off on two new vehicles and motors, with errors of both signs. The v_max
errors for Genesis/Erebus/Cavour use a velocity derived from altitude (0.6 s local fit), not an independent measurement.

**Open issue (Erebus 11, holdout).** The real altitude record implies a burnout speed of roughly Mach 1 (derived v_max
about 385 m/s); the simulation peaks at 289 m/s even though the apogee is only 6 % low. A burnout-speed gap of that size
points at an input/convention problem for this flight (reported mass vs. propellant mass, thrust-curve impulse) rather than
the drag model, but this was **not** investigated by re-tuning, because re-running the flight under changed conventions
after seeing the result would make it a development flight (the protocol would label it a re-evaluation). It is reported
as unexplained. It also means the transonic/supersonic code paths (section 5 of `aerodynamics.md`) remain unvalidated.

## 3. Holdout protocol and log

`rocketsim validate-registry --split holdout` refuses to run holdout flights without `--confirm-frozen`; each run
appends to `validation_results/holdout_log.jsonl`: flight, UTC time, physics version, **fingerprint of the physics source**
(SHA-256 over the parsed syntax trees of `physics/ vehicle/ environment/ motor/ simulation/ config/ constants.py`, so
reformatting does not matter and any executable change does), overrides, calibration id, results. If the fingerprint later
changes, earlier results show as **STALE** and a re-run is labelled "RE-EVALUATION ... no longer a clean holdout".
A behaviour-preserving change (the code was reformatted and the fingerprint definition changed after the holdout run) is
declared with `--migrate-fingerprint`, which re-simulates every logged entry and writes a migration record **only if all
reproduce** (they did, to 1e-9; recorded in the log).

The log (`validation_results/holdout_log.jsonl`) contains the two nominal first evaluations, the two evaluations of the
pre-declared secondary hypothesis below (calibration cal-001; labelled "first holdout evaluation" because they were written by
separate runs - they are *not* independent tests of the same flight) and migration records. **Hardening after the first review:**
the CLI now writes to this log by default (`--log`), refuses holdout evaluation without a log or when the registry's
`frozen_physics_version` differs from the code, `validate` refuses holdout flights, new entries carry a SHA-256 hash chain
(`verify_log`), and `validate-registry --status` prints each holdout flight's status and the chain check. Entries written from 1.2.1 on also record SHA-256 hashes of the flight definition, sim config, telemetry, motor file and the
comparison code (`inputs_sha256`). **Known gaps:** the chain does not protect the *last* line or a truncated tail (an external
anchor, e.g. committing the log, is needed), and nothing is committed to git. The first four entries
pre-date the chain (they were assembled from two per-run logs) and are therefore **trust-based**, like the order of events.
**Status rule: after any edit to the physics source, run `--status`; if it says `stale`, run `--migrate-fingerprint` (which
re-simulates the logged entries and records equivalence only if they reproduce) or accept that the holdout is no longer clean.**

## 4. Calibration record (cal-001: global drag multiplier)

`validation_data/calibration_records.yaml` stores old value, proposed value, reason, evidence and effect. Fitted only on
the calibration flights (Genesis, Astra) within physical bounds [0.7, 1.3]: `drag_scale = 0.957`, which improves those two
flights (-2.5 -> -0.8 %, -0.9 -> +0.8 %). The leave-one-out criterion technically passes (1.50 % vs 1.91 %) but with two
flights (one Tier 3) and one fold getting worse this is not credible evidence; **it was not adopted.** As a declared
secondary test, the holdout flights were also evaluated with it:

| holdout flight | nominal | with drag_scale 0.957 |
|---|---|---|
| Erebus 11 | -6.40 % | -4.48 % |
| Cavour | +4.52 % | +6.65 % |
| RMS | 5.54 % | 5.67 % |

The calibration does not improve the holdout RMS (and moves the two flights in opposite directions): the data do not
support a global drag correction. The development flight NDRT (+13.4 %) wants *more* drag, the others less: one number
cannot absorb the residuals, which look like input/convention error rather than a drag-level error.

## 5. Independent cross-checks (RocketPy 1.13)

*Trajectory* (`crosscheck_rocketpy.py`, identical vehicle, constant Cd, standard atmosphere, no wind): apogee -0.18 %,
apogee time -0.18 %, max speed -0.03 %, rail-exit speed -0.32 %, 80-deg launch range -0.5 % - still < 1 % after the V1.1
geometry/aero refactor. This checks the dynamics, not the drag model.

*Geometry/stability* (`crosscheck_geometry.py`, new in V1.1): the same nose/fin/boat-tail geometry built in RocketPy for
four EuRoC vehicles:

| vehicle | CNa RocketPy | CNa here | CP RocketPy [m from nose] | CP here | CP diff |
|---|---|---|---|---|---|
| Erebus 11 (with boat-tail) | 8.075 | 8.075 | 1.5752 | 1.5719 | -0.036 cal |
| Genesis | 13.657 | 13.657 | 1.8997 | 1.8984 | -0.014 cal |
| Cavour | 10.505 | 10.505 | 2.1569 | 2.1535 | -0.032 cal |
| Astra | 16.061 | 16.061 | 1.4930 | 1.4918 | -0.012 cal |

CNa is identical (same Barrowman equations, independent implementation); the CP offset is the von Karman (RocketPy,
0.5 L) vs tangent-ogive (here, 0.466 L) nose CP. OpenRocket was **not** used: a jar would have to be downloaded and
executed from the internet, which was not authorised; RocketPy is the independent code. Neither tool is assumed correct.

## 6. Input-uncertainty Monte Carlo

`validation_results/input_uncertainty_mc.json` (module `validation/input_mc.py`; **150 samples per flight**, seed 1; rerun
with n = 150 after the first review found n = 40 too noisy: the 5-95 % edges moved by up to 60 m). Priors, 1-sigma: wind speed
|N(0,3)| m/s with uniform direction; air temperature +N(0,4) K; launch elevation +N(0,1.5) deg; mass x N(1, 0.02); thrust
x N(1, 0.03); optionally drag x N(1, 0.07). (The holdout flights are re-simulated here with perturbed inputs; this is an
uncertainty analysis, not a new logged holdout evaluation, and nothing was tuned.)

| flight (split) | real [m] | MC mean +- sd, inputs only | real inside 90 % band? | z | with drag +-7 % | inside? |
|---|---|---|---|---|---|---|
| Bella Lui (dev) | 459 | 462 +- 32 | yes | -0.09 | 458 +- 33 | yes |
| NDRT (dev) | 1320 | 1503 +- 78 | **no** | -2.35 | 1495 +- 83 | **no** (z -2.10) |
| Prometheus (dev) | 3904 | 3711 +- 144 | yes (p95 = 3908, at the edge) | +1.34 | 3688 +- 176 | yes |
| Genesis (cal) | 2917 | 2841 +- 115 | yes | +0.65 | 2828 +- 147 | yes |
| Astra (cal) | 3249 | 3216 +- 120 | yes | +0.28 | 3201 +- 154 | yes |
| Erebus 11 (**holdout**) | 3002 | 2808 +- 96 | **no** | +2.02 | 2798 +- 138 | yes |
| Cavour (**holdout**) | 2789 | 2911 +- 91 | yes | -1.35 | 2902 +- 137 | yes |

With inputs only, 5 of 7 observed apogees fall inside the 90 % band (NDRT and Erebus 11 do not); with an added +-7 % drag
uncertainty 6 of 7 do (NDRT still does not, z = -2.1). For 7 flights, ~6 of 7 inside a 90 % band is what a calibrated
model would give, so **the result is consistent with calibrated uncertainty, not evidence of accuracy**; but the 7 % was chosen
*after* seeing that inputs alone missed some flights, so it is a fitted nuisance term, and the bands (+-4-5 %) are wide enough
that covering a flight is a weak test. NDRT (two-diameter airframe the model cannot represent) and Erebus 11 (unexplained
burnout speed) are the outliers, which points at model-form/convention problems, not at the sampled inputs.

## 7. Small-rocket investigation (gap, not a result)

No public telemetry for a <1 kg rocket with documented mass, motor and altimeter was found (altimeter-cloud flights lack
mass and motor; the Estes paper is prose). Therefore **the simulator is unvalidated below ~5 kg**. What can be said
analytically (`error_budget.md`, section 3): for the 0.45 kg, 41 mm example vehicle the first-order apogee uncertainty
from unmeasured inputs is 8.5 % (vs. 6.5 % for the 11 kg Genesis), dominated by fin thickness (5.1 % per mm), surface
roughness (4.5 %) and drag level (3.8 %) - the same terms that dominate small real rockets, plus launch-rail and
tip-off effects that are not modelled. Required next step: a weighed Estes/Aerotech-class flight with a logged altimeter.

## 8. Earlier (V1) material that still stands

* Analytic and unit validation (`tests/`, ~430 tests): free fall, vacuum projectile, Tsiolkovsky with gravity, quadratic
  drag fall, torque-free precession, constant-rate quaternion integration, weathercock frequency, TVC torque, ISA table,
  motor impulse, integrator order, quaternion identities, and since V1.1: Barrowman by hand, inertia tensor vs independent
  summation, Euler equations for asymmetric bodies, control-surface force/moment analytics, mixer decoupling, actuator
  rate/lag/delay exactness, 5-step convergence ladder, conservation limits ([testing.md](testing.md)).
* The AltOS `speed` column of the Prometheus log is signed *vertical* velocity (found by an earlier critic review); its
  integral to apogee (4175 m) exceeds the barometric apogee (3904 m), so the real geometric apogee was probably 4.1-4.2 km
  and the V1 geometric "+0.03 %" was a coincidence of two errors. Barometric-equivalent comparison is used throughout.
* V1 leave-one-out drag calibration on the three development flights was rejected (RMS 8.3 % -> 14.2 %).
* The V1 model-change history (Hoerner base drag, Raymer roughness cut-off) is in `physics.md` (change log 1.1.0): it was
  motivated by Prometheus, so Prometheus is in-sample.

## 9. What this validation shows and does not show

* **Shows**: an uncalibrated first-principles model predicts apogee of seven 7-24 kg solid-motor rockets (459 m to 3.9 km)
  with errors of -6.4 to +13.4 % (RMS 6.2 % over all seven; 5.5 % over the two holdout flights), agrees with RocketPy to
  < 1 % on dynamics and exactly on CNa, and is consistent with 6 of the 7 flights when a (post hoc) +-7 % drag uncertainty is allowed (NDRT is not).
* **Does not show**: supersonic accuracy (unresolved Erebus 11 burnout-speed gap), anything below ~5 kg, wind response,
  descent accuracy (chute data assumed), attitude fidelity (no attitude telemetry), sensor or estimator performance on real
  data, control-surface or TVC behaviour on real data (none exists in the registry; those subsystems are verified by
  analytic tests and closed-loop simulation only).
* **Statistical power**: n = 2 holdout flights. Treat all generalisation statements as provisional.

## 10. Next validation steps

Flights with a measured atmosphere and wind, weighed vehicles and motors, raw unfiltered IMU including attitude, at least
one supersonic flight, one sub-1-kg flight, a TVC/fin-controlled flight; reconstruct the four remaining RocketPy flights
(Andromeda, Camoes, Juno3, Valkyrie) as an additional holdout pool; resolve the Erebus 11 burnout-speed discrepancy with
the team's weights; a second independent code with the same Cd tables.
