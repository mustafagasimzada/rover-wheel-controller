RP2040 Distributed Wheel Controller
📌 Context & Background
Over the past year, I've been developing embedded software for rover control systems. In large-scale autonomous rovers, routing all sensory feedback and motor PWM signals to a single central computer creates massive processing bottlenecks and single points of failure.

To solve this, we moved to a distributed architecture. This repository contains the independent wheel controller module I developed. I have extracted, sanitized, and refactored this specific subsystem from our private team repository to serve as a standalone portfolio piece. It demonstrates how I handle real-time hardware control, networking, and safety failsafes on bare-metal silicon.

⚙️ Hardware Stack
Microcontroller: Raspberry Pi Pico (RP2040) overclocked to 133MHz

Network: WIZnet W5500 Ethernet Controller (Hardwired TCP/IP)

Motor Driver: RoboClaw (via Non-blocking UART)

Feedback: AS5600 Magnetic Encoder (via I2C)

🏗️ What This Code Actually Does
Instead of dumb PWM forwarding, this firmware turns each wheel into an independent "smart node" on the rover's local network:

High-Throughput JSON Server: Runs a custom, lightweight TCP server that parses incoming JSON trajectory commands with near-zero latency.

Hardware-Level PID: The central computer doesn't micromanage the steering. It just sends {"topic": "steering_angle", "angle": "45"}. The RP2040 handles the closed-loop PID calculations locally at 50Hz, including integral anti-windup (crucial for overcoming static friction on rough terrain).

Smooth Drive Ramping: Sudden speed changes pull massive current spikes. I implemented algorithmic ramp-rates (DRIVE_RAMP_RATE) to protect the mechanical drivetrain and power distribution board.

The "Deadman's Switch" (Watchdog): In autonomous systems, connection loss is fatal. If the TCP socket drops or no commands are received for 2000ms, the failsafe triggers, hardware-halting the RoboClaw immediately.

🚀 Network Scalability
The architecture is designed so the same exact firmware runs on all 4 (or 6) wheels. By simply changing #define WHEEL_NUMBER 1 at compile time, the node dynamically calculates its own static IP, MAC address, and specific TCP Port based on the base offsets.

Wheel 1 -> 10.42.0.30:8000

Wheel 2 -> 10.42.0.32:8001

...and so on.

🛠️ Build Instructions
Standard Pico C/C++ SDK build process:

Bash
mkdir build
cd build
cmake ..
make -j4

📝 Lessons Learned & Future TODOs
JSON Parser: The current string-manipulation JSON parser is fast but a bit brittle. Planning to migrate to a more robust state-machine parser like jsmn in the future.

Calibration Offsets: Currently, the AS5600 encoder offsets are hardcoded in the header. They need to be moved to the RP2040's flash memory so we don't have to recompile every time a wheel gets mechanically realigned.

UART Initialization: Discovered that occasionally the RoboClaw drops the first byte at 38400 baud on a cold boot. Future fix will include a minor hardware delay check before the first transmission.