# Autonomous Rocket Project — Weekly Logbook

**Entry date:** November 16, 2026  
**Reporting window:** November 9–16, 2026  
**Project areas:** Rocket Simulator, avionics, GNC architecture

## Work completed this week

No new hands-on work for November 9–16 was provided in the available project notes. I therefore cannot honestly record new coding, testing, CAD, soldering, or hardware integration as completed this week.

A repository audit was completed for this report. The latest visible commit in `MTMAG11/rocket-simulator` is:

- Commit: `e05f4e99b3e98f62a692ce2167862d8fd88fe865`
- Commit message: `V1.2.1`
- Commit date: October 9, 2026

No newer commit appeared in the repository's recent commit history at the time of this audit. This establishes the visible GitHub state, not whether uncommitted local work exists.

## Verified simulator state

The repository is substantially beyond the older V0.1 UI milestone. The README describes a 6-DOF solid-motor flight simulator and dataset generator for autonomous-rocket GNC development.

The documented system includes:

- 6-DOF rigid-body flight dynamics, with 3-DOF and 1-D fidelity levels.
- Component-based `vehicle.json` descriptions, deriving mass properties, center of gravity, inertia, center of pressure, and static margin.
- Aerodynamic model interfaces for drag, lift, and pitching moment.
- TVC and control-fin actuator models.
- Sensor simulation, a linear Kalman filter, and controller interfaces.
- Separation of truth, measurement, estimate, command, and actual state.
- HIL protocol and loopback/pipe/record-replay foundations.
- Bundled motor curves and ThrustCurve.org motor discovery/download.
- Reproducible Monte Carlo dataset generation and a real-flight validation protocol.

The UI package contains a main window, vehicle/motor catalog, motor browser, graphing, channels, launcher, styles, and a 3D view. The simulation package contains a builder, phase handling, recording, simulator, and timing modules.

## Validation status and limitations

The repository's validation documentation reports comparisons against seven real flights in the 7–24 kg class. The documented holdout set contains two flights. The holdout apogee RMS error is reported as 5.54% for those two flights.

This is not broad validation. The repository explicitly says that:

- No small-rocket (under 1 kg) validation data are available.
- Supersonic flight is not validated.
- No vehicle, actuator, sensor, estimator, or HIL model has been validated against real hardware.
- The estimator is a linear Kalman filter, not an EKF.
- No hardware has been connected to the HIL interface.
- Several validation inputs are uncertain or reconstructed from secondary sources.

These are important limitations, not reasons to discard the work. They define what evidence is still needed.

## Avionics status carried forward from earlier notes

The last known hardware configuration in project notes was:

- ESP32 XH-32S flight-computer development board.
- BMP388 barometer using I²C address `0x77`.
- ICM42688-P IMU using I²C address `0x69` in the successful manual-read test.
- I²C pins: SDA 21, SCL 22.
- SD interface: CS 5, SCK 18, MISO 19, MOSI 23, previously tested at 1 MHz.
- Intended logging interval: 20 ms (50 Hz).
- Initial prototype was breadboard-based.

Earlier tests showed successful BMP388 readings and successful manual ICM42688 register/configuration reads. The library `begin()` result was inconsistent with the successful manual reads. This is historical context; it is not confirmation that the current hardware is still assembled or working.

The last known inert avionics fit-check envelope was 1.25 in internal diameter and 5.5 in available length. A Fusion 360 fit-check model was being planned. No newer CAD or integration result is confirmed here.

## Decisions / conclusions

1. Treat the GitHub repository as the authoritative simulator reference unless newer local code is supplied.
2. Do not use the old V0.1 blocker list as the current simulator roadmap; it is outdated relative to the repository.
3. Keep simulator implementation, simulation validation, avionics bench testing, and flight validation as separate evidence categories.
4. Do not describe simulated sensors or HIL software as hardware-validated.
5. Preserve validation provenance and the holdout protocol. Do not tune against holdout results and continue calling them independent holdouts.

## Current blockers / unknowns

- Whether local uncommitted simulator work exists after October 9.
- Whether the latest GitHub commit has been pulled and run locally.
- Current test-suite status on the user's machine.
- Current physical state of the ESP32/BMP388/ICM42688/SD prototype.
- Whether the avionics fit-check CAD has been completed.
- Whether any new bench measurements or flight data exist.

## Next actions

1. Compare the local repository with GitHub and record any uncommitted changes.
2. Run the documented test suite and simulator smoke check; record exact commands and outputs.
3. Review the V1.2.1 physics and validation status before making further model changes.
4. Confirm the current avionics assembly and repeat a controlled sensor + SD logging test.
5. Resume the Fusion 360 fit-check only after confirming the actual board, connector, cable, and mounting dimensions.
6. Record real work completed during the next reporting window with screenshots, commit hashes, test outputs, or bench logs.

## Evidence classification

- **Verified from GitHub:** repository structure, README capabilities and limitations, status matrix, validation summary, latest visible commit.
- **Historical project context:** ESP32/BMP388/ICM42688/SD configuration and earlier sensor test results.
- **Not verified for November 9–16:** new code, local tests, hardware work, CAD work, or flight testing.
