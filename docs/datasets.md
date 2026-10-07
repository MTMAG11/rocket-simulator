# Data generation, datasets and ML readiness

## Telemetry schema

One row per recorded step. The column list, units, dtypes and meaning are in [schema.md](schema.md) (generated from
`rocket_sim/data/schema.py`; versioned by `SCHEMA_VERSION`). Groups: `time`, `flight`, `state` (truth), `mass`,
`environment`, `control` (>= fidelity 3), `sensors` (>= 4, `meas_*`), `estimator` (>= 5, `est_*`). Changing, renaming or
removing a column requires bumping `SCHEMA_VERSION`; a dataset's manifest records the exact column list.

## Exports of a single flight

`rocketsim simulate config.yaml --out dir --format csv,parquet,npz,json` writes any of

* CSV (inspection; metadata in `<name>.meta.json`), JSON (everything incl. schema and data), NPZ (`columns`, `data`, `meta`),
  Parquet (typed columns, ZSTD, metadata in the file footer under `rocket_sim.meta`).
* Readers: `read_record_csv/npz/parquet` round-trip losslessly (CSV to 9 significant digits).

## Reproducibility

Every run is a pure function of (config, seed). The record stores: simulation id, random seed, config hash **and the
full config**, simulator/physics/schema versions, fidelity, fast flag, dt, integrator, creation time, warnings and the
fidelity overrides applied. Seeds: `SeedSequence([master_seed, split_namespace, run_index])`; parameter draws use
`spawn(1)[0]`, the simulation seed is `generate_state(1, uint64)[0] >> 1`. Consequently any run can be regenerated from
`(master_seed, split, index)` without generating its neighbours, independent of worker count or chunking
(`tests/test_data.py::test_batch_is_independent_of_worker_count_and_resumable`). `rocketsim export DATASET --run ID`
re-simulates a run from the manifest, its parameter draws and its seed.

## Batch / dataset generation

```bash
rocketsim batch configs/batch_example.yaml --runs 1000          # full telemetry shards (Parquet)
rocketsim generate-dataset configs/dataset_example.yaml --runs 100000
```

Headless, multi-process (`workers: 0` = all cores - 1, `1` = serial), streaming (one shard per `shard_runs`
simulations, memory bounded by one chunk), checkpointed. Spec reference: [config_reference.md](config_reference.md).

```
output/<name>/
  manifest.json      dataset id, versions, base config + hash, spec, counts, seeds, schemas, leakage checks, file checksums
  runs.parquet       one row per attempted run: id, split, seed, accepted, reject_reason, param.* draws, res.* results
  rejected.jsonl     every rejected run with its reasons and parameters
  shards/<split>/    telemetry|tabular .parquet or windowed .npz shards
  chunks/            per-chunk completion markers (the checkpoint) ; state.json guards against changed specs
```

**Checkpointing.** A chunk is complete when its JSON marker exists (written last, atomically). Re-running the same command
skips completed chunks; the final manifest and shard checksums are identical to an uninterrupted run (tested by deleting
a chunk and resuming). A different spec in the same directory is refused.

**Quality gate (`data/quality.py`).** A run enters a shard only if: it did not raise (config/numerical errors are caught
and logged), all values are finite, status is `ok` (not timeout / never left the pad), speed/altitude/angular rate are
below sanity limits, altitude never below ground, quaternion norm intact, required columns present, landing recorded,
apogee above a minimum. Everything else goes to `rejected.jsonl` with the reason.

**Splits without leakage.** `splits: {train, val, test}` use different `split_namespace` seed streams (verified disjoint
for up to 200k runs), may use *different parameter distributions* (e.g. a test set with stronger wind and heavier
vehicles = out-of-distribution hold-out, see `configs/dataset_example.yaml`), and the manifest reports any identical
parameter vectors shared across splits (`leakage_checks`). Remember that train/val/test drawn from the *same*
distribution measure interpolation, not robustness.

## Dataset kinds and feature/label configuration

`dataset.kind`:

* `telemetry` - all schema columns, every step (analysis, EKF development, plotting).
* `tabular` - one row per resampled step with the requested features and labels.
* `windowed` - `X (N, history, n_features)` float32 and labels per window.

Features are declared in YAML: `{column: meas_baro_altitude}`, `{column: vel_z, lag: 5}` (value 5 resampled steps ago),
`{derived: time_since_ignition}` or `{derived: time_since_launch_detected}` (needs an estimator). Targets are plain truth
columns. `window.target`: `current` (label at window end), `future` (label `horizon_s` later), `trajectory` (next K
samples, `Y (N, K, L)`). Flights are trimmed to liftoff..landing (`trim: flight`) and resampled to a uniform grid
(`sample_dt_s`). **Every input column is zero-order held** (the value of the latest native sample at or before the grid time;
V1.1: truth inputs used to be linearly interpolated, which reads one native sample *after* the grid time - a small non-causal
leak). Targets are labels and are interpolated. Nothing in the simulator is specific to a network architecture; choosing deployable (non-truth) inputs
is the user's responsibility and is documented in the manifest (`feature_schema`, `label_schema`).

Windowed shard (`.npz`): `x`, `y`, `t_end`, `run_ref` (index into `simulation_ids`), `simulation_ids`.

## Manifest contents

dataset id (`<name>-<spec hash>`), creation time, simulator/physics/schema/config versions, fidelity level and fast flag,
base config (+hash), full spec, counts (requested / accepted / rejected / samples / per split), seed information and the
derivation rule, leakage checks, feature and label schemas, window definition, telemetry columns, quality limits, the
list of shard files with SHA-256, worker count. V1.1 additions: `dataset_version`, `statistics` (per-split parameter and
result statistics), `sampler_checks` (KS tests of sampled vs declared marginals), `leakage_checks` (near-duplicates, run
disjointness, resampling rule), `statistics_report` (`dataset_report.md`), and per-row `dataset_id`/`dataset_version`/`config_hash`
in `runs.parquet` for traceability.

## Inspecting datasets without loading telemetry

* `rocketsim inspect DATASET` / GUI "Data browser": reads only `manifest.json` and `runs.parquet`.
* `rocketsim inspect DATASET --run ID`: parameter draws and results of one run.
* `data.browser.load_telemetry(dataset, id)`: one run's rows (Parquet row-group per run => no other run is decoded).

## Loading for training (example)

```python
import json, numpy as np
m = json.load(open("output/est-demo/manifest.json"))
train = [np.load(f"output/est-demo/{f['path']}") for f in m["files"] if f["split"] == "train"]
X = np.concatenate([z["x"] for z in train])   # (N, 50, 8) float32
Y = np.concatenate([z["y"] for z in train])   # (N, 1)
```

## Limitations to keep in mind

* Fast-mode data (fidelity <= 2) lack attitude dynamics and sensors; use fidelity >= 4 for estimator/controller learning.
* The simulator was compared (docs/validation.md) with seven amateur flights (no flight below ~7 kg); sim-to-real gaps (aerodynamics, sensor
  noise structure, vibration, wind spatial structure) are real. Domain randomisation over the relevant parameters is
  provided precisely because the nominal model is only accurate to ~10 % in apogee for the vehicles validated (and unvalidated beyond that).
* Sensor defaults are representative orders of magnitude, not a datasheet.


## V1.1: domain randomisation

`configs/domain_randomization.yaml` is the reference spec. Rules (all stored verbatim in the manifest's `spec`):

1. **Explicit distributions only**: `normal`, `truncnormal`, `uniform`, `lognormal`, `choice`, or `linked`. Nothing is
   randomised implicitly; `rocketsim generate-dataset` fails if a path does not exist.
2. **Physically linked quantities are not sampled independently.**
   * `dist: linked` - `value = base * (sampled_ref / base_ref) ** exponent * (1 + N(0, rel_std))`; e.g. inertia follows the
     sampled mass (`ref: rocket.dry_mass_kg`, exponent 1). The ref must appear earlier in the list.
   * `correlations:` - a Gaussian copula over a group of continuous parameters (`paths`, symmetric positive-definite
     `matrix`). Each marginal is *exactly* the declared distribution (verified by KS tests in the tests and per dataset); only
     the dependence changes. Example: temperature offset vs sea-level pressure (-0.4), mean wind vs turbulence (+0.7).
     Parameters outside groups draw exactly as before, so existing specs reproduce bit-for-bit.
3. **Per-flight sensor imperfections**: noise, bias, misalignment, GPS dropout probability (Bernoulli per fix), GPS start-up
   delay, GPS velocity noise - all ordinary parameters (`sensors.*`).
4. **Splits are whole flights with disjoint seed namespaces.** A `test` split can use a *shifted* distribution
   (the example uses stronger wind, heavier vehicles and lower launch angles than training). The dataset report records how far
   the splits are (median nearest-neighbour distance train->test vs train->train).

## V1.1: leakage tests and quality gates

* *Trajectory leakage*: windows never span two flights; ids and seeds appear in exactly one split (`check_run_disjoint`,
  stored in the manifest).
* *Near-duplicate leakage*: `near_duplicate_report` standardises the varying numeric parameters on the pooled std and
  reports, per non-train split, the nearest-neighbour distance to the training runs; a run is a near-duplicate if it is closer
  than `max(eps, 0.05 x the median within-train nearest-neighbour distance)` (relative, so it works in 20 dimensions) and raises a
  manifest warning. The V1 check only caught bit-identical parameter vectors; the tests show 40 of 40 copies perturbed by
  0.02 sigma in 20-D are found and 0 of 40 independent draws are flagged. It detects *copying*, not general similarity.
* *Temporal leakage*: `temporal_causality_check` corrupts everything after many cut times and verifies that every resampled
  input row (and, for windowed datasets, every window ending before the cut) is bit-identical (tested on truth inputs, measured inputs, lags and the derived clock feature;
  with a negative control: switching the pipeline to interpolation makes the detector fire).
* *Label leakage* is the user's choice (truth vs measured inputs) and is recorded, not policed.
* *Sampler correctness*: every declared normal/uniform/lognormal marginal is tested against its CDF (KS, p < 1e-3 flags a
  warning) once a split has >= 200 accepted runs.
* *Quality gates* (`data/quality.py`, tested per failure class): non-finite values, speed/altitude/rate bounds, quaternion
  drift, status, missing columns, missing landing, apogee floor. Rejected runs are written to `rejected.jsonl` with the
  seed and the parameter draw; a rejection rate above 5 % raises a bias warning.

## V1.1: statistics report

Every dataset directory contains `dataset_report.md`: per-split acceptance, parameter statistics (mean, std, min, p05, p95,
max), result statistics (apogee, max speed/acceleration, ...), the leakage report, the KS checks and the rejection reasons.

## V1.1: experiments (versioned, reproducible)

```bash
rocketsim experiment configs/experiment_example.yaml     # writes experiments/<name>-<spec hash>-<UTC stamp>/
rocketsim experiment --list
rocketsim experiment --verify experiments/<id>           # regenerates and compares every shard SHA-256
```

`experiment.json` stores the experiment id, date, git commit and dirty flag, simulator/physics/schema/config/dataset versions,
master seed, spec hash, base-config hash, dataset id, environment (Python/NumPy/PyArrow), command line, counts, warnings, and the
SHA-256 of the manifest and every shard; the spec is copied next to it. Same spec hash => bit-identical data (tested:
`--verify` reproduces all shards; a different seed changes hash and shards).

## V1.1: benchmarks

`rocketsim benchmark` measures single runs and a scaling ladder of **1 / 100 / 1000 / 10000** simulations (`--scales` to change).
Numbers and the machine they were measured on are in [performance.md](performance.md).


## V1.2: column roles, provenance and state-source flags

* Every column has a **role** (`truth`, `measurement`, `estimate`, `command`, `actual`; `schema.md`, `rocket_sim.data.schema.columns_by_role`).
  Choose ML inputs from `measurement`/`estimate` columns to model a flight computer; use `truth` columns for labels. New truth columns:
  the full inertia tensor (`izz`, `ixy`, `ixz`, `iyz`), lateral CG offsets, **true specific force** (`fsp_*`, what an ideal accelerometer
  reads); new measurement flags `meas_{accel,gyro,mag}_new`; estimate `est_gyro_bias_*`; per-fin commands `fin_dcmd_*`. Schema version 1.2.0.
* `runs.parquet` carries `aero_provenance_kind` (`estimate` / `mixed`), `aero_model` and `controller_state_source`
  (`none` / `estimate` / `truth` / `measurements_only`) for every run, and each flight record carries the full `aero_provenance` in its metadata.
  **A dataset whose controller_state_source is `truth` was generated with a controller that saw the true state: do not present it as
  representing a flight computer.** Estimated and imported aerodynamics are never labelled alike.
