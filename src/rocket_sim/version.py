"""Version identifiers. Bump deliberately: they are stored in every exported dataset.

SIM_VERSION      software release (semantic versioning).
PHYSICS_VERSION  changes whenever any equation/model changes results. Old datasets stay
                 interpretable because the version is stored with them; see docs/physics.md
                 for the change log.
SCHEMA_VERSION   telemetry/dataset column schema (see rocket_sim.data.schema).
CONFIG_VERSION   configuration-file format.
DATASET_VERSION  dataset directory / manifest layout.
"""

SIM_VERSION = "1.2.0"  # the single source of the release number (pyproject.toml reads it; GUI and executable display it)
PHYSICS_VERSION = "1.2.1"
SCHEMA_VERSION = "1.2.0"  # 1.2.0: full inertia tensor, true specific force, CG offsets, estimated gyro bias, per-fin commands, roles
# (1.1.0: control-surface columns, estimator columns from fidelity 3)
CONFIG_VERSION = 1
DATASET_VERSION = "1.1.0"  # manifest/dataset layout
