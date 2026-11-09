# SegwayCode

Firmware for the self-balancing Segway on an Arduino Nano ESP32 (ESP32-S3). It is written in Rust (std) on ESP-IDF v5.5.3, the same stack as `esp32-s3-testing/hello_esp32`.

## Architecture

```
          CORE 0 - networking, best effort              CORE 1 - control, hard real time
 ┌──────────────────────────────────────────┐   ┌──────────────────────────────────────────┐
 │ ESP-IDF: WiFi, lwIP, esp_timer, events   │   │ "control" thread, prio 20, alone on core │
 │                                          │   │                                          │
 │ main thread: comm::setpoints             │   │ every MPU INT pulse (200 Hz):            │
 │   long-poll GET /device/segway ──────┐   │   │   read IMU -> Estimator -> State         │
 │                    SHARED.commands ══╪═══╪══▶│   latest Command (try_lock, never waits) │
 │                                      │   │   │   safety FSM -> Controller -> TB6612     │
 │ "net-tx" thread: comm::telemetry     │   │   │                                          │
 │   POST /device/telemetry ◀═ channel ═╪═══╪═══│   every 4th tick: try_send(Telemetry)    │
 │   logs control status ◀═ SHARED.status ══╪═══│   once a second: Status                  │
 │   any HTTP success ═▶ SHARED.link ═══════╪══▶│   link stale > 1 s -> lean/turn = 0      │
 └──────────────────────────────────────────┘   └──────────────────────────────────────────┘
```

- **Core 0** runs everything network related, plus all ESP-IDF system tasks (pinned there in `sdkconfig.defaults`). Errors on this core are logged and retried, never returned, so a network problem can't reboot the chip and drop the robot.
- **Core 1** runs only the control thread. It creates its own drivers (I2C, GPIO interrupt, LEDC), so their interrupts also land on core 1. Inside the loop it never blocks on core 0, never logs and never allocates. Core 0 does the logging, from the `Status` that the control thread publishes.
- **The cores share only [`src/shared.rs`](src/shared.rs)** (two latest-value mailboxes and an atomic heartbeat) plus the telemetry channel.

### Safety modes (`shared::Mode`)

| Mode | Motors | Leaves when |
|---|---|---|
| `Calibrating` | standby | gyro bias measured (2 s, keep the robot still) |
| `Disabled` | standby | server sends `enabled: true` |
| `Armed` | standby | robot lifted to within 5° of the target → `Balancing` |
| `Balancing` | PID | \|pitch\| > 45° → `Fallen`; `enabled: false` → `Disabled` |
| `Fallen` | standby | lifted upright again → `Balancing` |
| `Fault` | standby | IMU silent or a motor write failed; latched until `enabled: false` |

When the server link is lost, the robot **keeps balancing** with lean and turn set to zero. Turning `enabled` off always stops the motors, and the robot will fall.

## Files

| Path | What |
|---|---|
| `src/main.rs` | Boot, the **PIN BLOCK** (all GPIOs), and core assignment |
| `src/config.rs` | Every tuning constant: loop rate, IMU mounting, limits, timeouts |
| `src/shared.rs` | Everything the cores share: `Command`, `Telemetry`, `Status`, `Mailbox`, `Link` |
| `src/control/` | `mod.rs` loop and FSM, `imu.rs` MPU-6050, `motors.rs` TB6612, `estimator.rs`, `controller.rs` |
| `src/comm/` | `wifi.rs`, `setpoints.rs` (long-poll in), `telemetry.rs` (batched POST out) |

The **EKF and nonlinear controller** plug in as new implementations of the `Estimator` and `Controller` traits. You swap them in `control::run`; nothing else changes.

## Before the first run

1. **Pins:** set the real GPIOs in the PIN BLOCK in `src/main.rs`. They are placeholders now.
2. **IMU mounting:** set `IMU_FORWARD` / `IMU_LEFT` / `IMU_UP` in `src/config.rs`. Tilt the robot forward and check that the logged pitch goes positive.
3. **Motor direction:** with the wheels in the air, positive duty must drive the robot forward. Use `MOTOR_LEFT_INVERT` / `MOTOR_RIGHT_INVERT` to correct it.
4. **WiFi:** `cp wifi.env.example wifi.env` and fill it in, or copy the one from `hello_esp32`.

## Build and flash

Tool setup is the same as for `hello_esp32` (`cargo install espup ldproxy espflash`, then `espup install`).

```sh
cargo build --release
cargo run --release     # flash and open the monitor
```

## Server contract

The server side is not implemented yet.

**`GET {SERVER_URL}/device/segway[?since=<version>]`** is a long-poll: when `since` equals the current version, the server holds the request for up to 10 s. It returns:

```json
{"version": 3, "enabled": true, "trim_deg": 1.5, "lean_deg": 0.0, "turn": 0.0,
 "gains": {"kp": 6.0, "ki": 0.0, "kd": 0.3}}
```

- Every field is optional and defaults to 0 / false.
- Gains act on radians: `kp` is duty per rad, `kd` is duty per rad/s.
- `turn` is a differential duty; positive turns left.

**`POST {SERVER_URL}/device/telemetry`** has the body `{"samples": [...]}`, sent every 250 ms. Samples come at 50 Hz, and each one looks like:

```json
{"t_us": 12345678, "mode": "balancing", "pitch_deg": 0.4, "pitch_rate_dps": -2.1,
 "yaw_rate_dps": 0.0, "target_deg": 1.5, "u_left": 0.12, "u_right": 0.12,
 "accel": [0.0, 0.0, 1.0], "gyro": [0.1, -2.0, 0.0], "loop_us": 180, "dropped": 0, "link_ok": true}
```
