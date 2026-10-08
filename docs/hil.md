# Hardware-in-the-loop (HIL) foundation and timing model

Status: software only. The boundary, protocol, transports, timing model and a headless loop exist and are tested. No physical
flight computer has been connected; there is no serial/UDP transport, real-time pacing or firmware.

```
SIMULATION PHYSICS (truth) -> sensor models -> [ protocol v2 ] -> FLIGHT COMPUTER -> command -> [ + compute/downlink latency ]
          ^                                                                                              |
          +------------------------------ physics <- ACTUAL actuator state <- actuator model <-----------+
```

## 1. The boundary

Only two things cross: **timestamped sensor samples** (and design data at start-up) go to the flight computer; **commands** come
back. No truth **state** crosses: not position, velocity, attitude, rates, instantaneous mass or CG, nor the flight phase. **Design data does cross, and it is derived from the as-built vehicle**: the gimbal-authority schedule in `design` encodes the true (Monte-Carlo-dispersed) `I_yy / (T x lever arm)`, i.e. the flight computer is assumed to know its own as-built mass properties (the nominal thrust curve, not the motor's actual thrust scale). `controller.design_inertia_scale` (default 1) models imperfect knowledge by scaling that schedule; the in-process controllers' `authority` callback has the same property and the same knob. This is enforced by
construction (`HilBridgeController.consumes_state = False`: the simulator hands it a `ControlInput` with `valid = False` and zeroed
state fields) and by tests that inspect every message.

## 2. Protocol v2 (`rocket_sim.hil.protocol`)

Newline-delimited JSON, one message per line, deterministic encoding (sorted keys, no whitespace, NaN/Infinity rejected: the same
message is the same bytes, which is what makes recordings replayable). Every message carries `"v": 2`.

| direction | message |
|---|---|
| sim -> FC | `{"type":"init", "controller_rate_hz", "sensors": {name: {rate_hz, noise_std, latency_s}}, "design": {...}, "estimator": {alignment_time_s}}` |
| FC -> sim | `{"type":"ready"}` (the handshake; anything else aborts the run) |
| sim -> FC | `{"type":"tick", "seq": n, "t": s, "samples": [{"sensor", "t_sample", "t_visible", "value"}]}` every control period |
| FC -> sim | `{"type":"command", "seq": n, "cmd": {"tvc_y","tvc_z","fin_pitch","fin_yaw","fin_roll"}}` (missing fields = 0; unknown or non-finite fields are errors; the `seq` must match) |

`samples` contains **every** sample that became visible since the previous tick (not only the latest), each with when it was taken
(`t_sample`) and when it became visible (`t_visible`, sensor latency already included). Values: `accel` [m/s^2] and `gyro` [rad/s] in
body axes, `baro` [Pa] (1-vector), `gps` [x, y, z, vx, vy, vz] in the launch frame, `mag` [T] in body axes: all **measurements**
(noise, bias, scale error, quantisation, saturation, dropout applied).

`design` is as-designed knowledge the flight software legitimately has: nominal gimbal-authority schedule against time since ignition,
gravity, site elevation, launch axis, alignment time, reference magnetic field. In a Monte Carlo run it is derived from the dispersed
vehicle (the flight computer is assumed to know its as-built mass properties and the nominal motor curve, *not* the motor's actual
thrust scale), which is a stated simplification.

## 3. Transports (`rocket_sim.hil.transport`)

A transport is anything with `send(bytes)` and `recv() -> bytes` carrying one line (lock-step: the simulation waits for the reply).

| transport | status |
|---|---|
| `LoopbackTransport(handler)` | implemented: an in-process Python flight computer |
| `PipeTransport([cmd...], timeout_s)` | implemented: a child process on stdin/stdout (a firmware host build, a C++ binary, `python -m rocket_sim.hil.flight_computer`); a dead process is an error and a silent one is killed after `timeout_s` (default 60 s) with an error, not a hang |
| `RecordingTransport(inner)` / `ReplayTransport(log)` | implemented: JSONL session log; replay needs **no** flight computer and fails loudly if the simulator's requests ever differ |
| serial port, UDP, shared memory | **not implemented.** Each needs only `send`/`recv` of one line; the lock-step schedule would not change. Real-time pacing, timeouts and retries are also absent |

## 4. Flight-computer side

`FlightComputer` is the contract: `reset(init)` once, `on_tick(tick) -> command dict`. `handle_message` and `serve` turn any
implementation into a byte-line server. `ReferenceFlightComputer` is a pure-Python flight-software loop built **only** from packets
(pad alignment, `NavigationFilter`, `TVCAttitudeController`, launch detection from its own estimate): the executable specification of
what the ESP32 firmware has to do and the test double for the whole path. **It is not validated flight software**; it rebuilds its estimator configuration from the `init` message plus library defaults (so a simulator configured with non-default bias walk or estimator flags is *not* mirrored exactly), emits TVC commands only, and uses the same
algorithms as the in-process estimator and controller (so it cannot show that those algorithms are good, only that the boundary
works). Run: `rocketsim hil CONFIG [--replay LOG] [--log LOG] [--uplink S] [--rate HZ] [--command PROG ARGS...]` (`--command` takes everything after it, so give it last, e.g. `--command python -m rocket_sim.hil.flight_computer`; `--uplink` overrides `controller.uplink_latency_s`, which is honoured otherwise).

Measured in the test suite: the example unstable vehicle (static margin -0.7 cal, CG aft) is held within ~3.5 degrees during the burn
through the protocol (3.5 degrees; the same loop in process gives the same 3.5 degrees since both index the gimbal-authority schedule by time since detected launch; the vehicle tumbles to ~180 degrees without control). A child
process gives a bit-identical flight to the in-process reference computer. Command latency degrades the hold quickly and then loses it (tests): 0 ms -> 3.5 degrees, 10 ms -> 7.6, 20 ms -> 17, **40 ms -> control lost**.

## 5. Timing model

The loop does **not** run at one frequency. `rocketsim timing CONFIG` prints (and tests check) all of this:

| element | schedule |
|---|---|
| physics | variable step <= `simulation.dt_s` (`descent_dt_s` under a parachute); steps are **cut at every sensor sample time, controller tick, thrust-curve corner and event**, so the physics is advanced exactly to each of them (consequence: sensor rates slightly alter the integration grid, a ~1e-6 relative effect on truth) |
| each sensor | own `rate_hz`; sample released `latency_s` later; first sample after `startup_delay_s`; Bernoulli dropout |
| estimator (in-process) | **event-driven**: updates when a new sample is visible; there is no fixed estimator rate |
| controller / flight computer | ticks at `controller.rate_hz` (the tick period) |
| `controller.uplink_latency_s` | HIL bridge only: sample available to the flight computer `uplink` after `t_visible` (an explicit `params.uplink_latency_s` overrides it) |
| `controller.compute_time_s` + `controller.downlink_latency_s` | any controller: delay before the command reaches the actuator (applied by the simulator before the actuator's own `delay_s`) |
| actuator | advanced every physics step: transport delay `delay_s` -> first-order lag -> rate limit -> angle limit; its state is **held constant over the step** (left-endpoint hold, an effective extra lag of ~h/2 in closed-loop runs; not changed) |

Derived numbers (command-to-actuator latency, worst-case sensor-to-actuator latency, IMU samples per control tick) and warnings
(command latency over two periods, a controller that cannot see all IMU samples, a step coarse relative to the actuator lag, an
uplink latency set where it has no effect) are in the report.

Not modelled: clock offset and drift between simulator and flight computer, jitter, dropped/reordered messages, serial baud-rate
limits, interrupt latencies. A real ESP32 will expose these.

## 6. What is validated

| item | evidence |
|---|---|
| protocol encoding, validation, malformed-reply rejection | unit tests |
| every sample delivered once, ordered, timestamps consistent, equals the logged measurements | integration tests |
| no truth across the boundary | tests inspect every init/tick message against a whitelist, require every delivered sample value to be bit-identical to a logged **noisy** measurement (with a negative control that tampers with the samples), and inspect the `ControlInput` the bridge receives |
| uplink and compute/downlink latency act exactly as stated (to one step) | tests |
| bit-identical live runs, bit-identical replay without a flight computer, divergence detection | tests |
| pipe process == in-process computer | test |
| closed-loop attitude hold through the boundary | simulation test (not flight data) |
| anything about real hardware | **none** |
