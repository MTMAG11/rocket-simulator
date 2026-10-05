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
(`sample_dt_s`). Nothing in the simulator is specific to a network architecture; choosing deployable (non-truth) inputs
is the user's responsibility and is documented in the manifest (`feature_schema`, `label_schema`).

Windowed shard (`.npz`): `x`, `y`, `t_end`, `run_ref` (index into `simulation_ids`), `simulation_ids`.

## Manifest contents

dataset id (`<name>-<spec hash>`), creation time, simulator/physics/schema/config versions, fidelity level and fast flag,
base config (+hash), full spec, counts (requested / accepted / rejected / samples / per split), seed information and the
derivation rule, leakage checks, feature and label schemas, window definition, telemetry columns, quality limits, the
list of shard files with SHA-256, worker count.

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
* The simulator is validated (docs/validation.md) against three amateur flights; sim-to-real gaps (aerodynamics, sensor
  noise structure, vibration, wind spatial structure) are real. Domain randomisation over the relevant parameters is
  provided precisely because the nominal model is only accurate to ~10 % in apogee for the vehicles validated (and unvalidated beyond that).
* Sensor defaults are representative orders of magnitude, not a datasheet.
