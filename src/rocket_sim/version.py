"""Version identifiers. Bump deliberately: they are stored in every exported dataset.

SIM_VERSION      software release (semantic versioning).
PHYSICS_VERSION  changes whenever any equation/model changes results. Old datasets stay
                 interpretable because the version is stored with them; see docs/physics.md
                 for the change log.
SCHEMA_VERSION   telemetry/dataset column schema (see rocket_sim.data.schema).
CONFIG_VERSION   configuration-file format.
"""

SIM_VERSION = "0.2.0"
PHYSICS_VERSION = "1.1.0"
SCHEMA_VERSION = "1.0.0"
CONFIG_VERSION = 1
