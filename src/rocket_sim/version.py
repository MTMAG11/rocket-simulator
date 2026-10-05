"""Version identifiers. Bump deliberately: they are stored in every exported dataset.

SIM_VERSION      software release (semantic versioning).
PHYSICS_VERSION  changes whenever any equation/model changes results. Old datasets stay
                 interpretable because the version is stored with them; see docs/physics.md
                 for the change log.
SCHEMA_VERSION   telemetry/dataset column schema (see rocket_sim.data.schema).
CONFIG_VERSION   configuration-file format.
DATASET_VERSION  dataset directory / manifest layout.
"""

SIM_VERSION = "0.2.0"
PHYSICS_VERSION = "1.2.1"
SCHEMA_VERSION = "1.1.0"  # 1.1.0: control-surface columns, estimator columns from fidelity 3
CONFIG_VERSION = 1
DATASET_VERSION = "1.1.0"  # manifest/dataset layout (V1.1: statistics, leakage report, traceability ids)
