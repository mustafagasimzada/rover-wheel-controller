# RP2040 Distributed Wheel Controller

Over the past year I've been working on a large-scale autonomous rover project as part of a team. The full system — central navigation, sensor fusion, communication stack, everything — lives in a private team repository that I can't share publicly.

This is one module from that system. I extracted it, cleaned it up, and put it here as a standalone piece because it represents a real engineering problem I had to think through and solve, not just code I wrote following a spec.

---

## The problem it solves

Early in the project we had one central computer handling everything — reading sensors, computing paths, sending PWM signals to all four wheels. It worked at first. Then we added more sensors, more wheels, more logic, and it started falling apart. Latency crept up. One bug in the motor code could freeze the whole system. Wiring became a nightmare.

The fix was to stop treating wheels as dumb actuators and turn each one into its own controller. The central computer now just sends high-level commands over the local network. Each wheel handles its own feedback loop, its own safety logic, its own motor control. If one wheel has a problem, the others keep running.

This firmware is what runs on each wheel.

---

## What it actually does

**Steering** is a closed-loop PID running at 50Hz against an AS5600 magnetic encoder. One thing that took real tuning time was integral anti-windup — on rough terrain, static friction causes the integral term to wind up and the wheel overshoots significantly. Getting that right took a few iterations of field testing.

**Drive speed** ramps instead of jumping. We were getting brownouts on the power distribution board from hard acceleration current spikes. The ramp rate is a single tunable define now, but finding the right value came from actually watching the system fail.

**Deadman's switch** — if no command arrives for 2 seconds, motors halt immediately. This wasn't in the original design. We added it after a test run where a Wi-Fi dropout left a wheel spinning with nobody noticing for a few seconds. On a rover that can weigh 30+ kg, that's not acceptable.

**Network identity** — the same binary runs on all wheels. `WHEEL_NUMBER` at compile time sets the IP, MAC, and port automatically. Deploying a new wheel is a one-line change and a flash.

---

## Hardware

| Component | Details |
|---|---|
| MCU | Raspberry Pi Pico (RP2040) @ 133MHz |
| Ethernet | WIZnet W5500 (hardwired TCP/IP over SPI) |
| Motor driver | RoboClaw (simple serial, non-blocking UART) |
| Steering feedback | AS5600 magnetic encoder (I2C) |

---

## Network layout

```
Wheel 1 → 10.42.0.30 : 8000
Wheel 2 → 10.42.0.32 : 8001
Wheel 3 → 10.42.0.34 : 8002
Wheel 4 → 10.42.0.36 : 8003
```

---

## Protocol

Plain JSON over TCP, newline-terminated.

```json
{"topic": "steering_angle", "angle": "45"}
{"topic": "drive_speed", "speed": "75"}
{"command": "get_status"}
```

Full reference: [`docs/PROTOCOL.md`](docs/PROTOCOL.md)

---

## Build

```bash
mkdir build && cd build
cmake ..
make -j4
```

Set `WHEEL_NUMBER` in `main.c` before flashing.

---

## What I'd fix next

**JSON parser** — hand-rolled string scanner, fast but brittle. Would replace with `jsmn` or a proper state machine.

**Encoder calibration in flash** — zero-offsets are currently hardcoded. Every mechanical realignment means a recompile. The RP2040 has plenty of flash, this should be stored and updated at runtime.

**UART cold boot drop** — intermittently the RoboClaw drops the first byte at 38400 baud on power-on. Likely a timing issue between the UART init and the RoboClaw's startup sequence. A dummy byte or a short delay before the first real command would probably fix it. Haven't had the hardware in front of me to test it properly yet.

---

## Docs

- [`docs/HARDWARE.md`](docs/HARDWARE.md) — pin assignments, wiring
- [`docs/PROTOCOL.md`](docs/PROTOCOL.md) — JSON command reference  
- [`docs/CALIBRATION.md`](docs/CALIBRATION.md) — encoder calibration
