# JSON Protocol Reference

All communication is plain JSON over TCP, newline-terminated (`\n`).  
Connect to the wheel's IP and port, send a JSON object, receive a JSON object back.

---

## Commands

### Set steering angle

```json
{"topic": "steering_angle", "angle": "90"}
```

`angle` — target position in degrees, 0–360.

Response:
```json
{"target_steering_angle": 90, "current_steering_angle": 88}
```

---

### Set drive speed

```json
{"topic": "drive_speed", "speed": "75"}
```

`speed` — percentage, -100 to 100. Negative = reverse.

```json
{"topic": "drive_speed", "speed": "-50"}
```

Response:
```json
{"drive_speed": 48}
```

---

### LED control

```json
{"topic": "led_state", "state": "on"}
{"topic": "led_state", "state": "off"}
```

---

### Status / heartbeat

```json
{"command": "get_status"}
{"command": "heartbeat"}
```

Response:
```json
{
  "status": "running",
  "led_state": "off",
  "drive_speed": 48,
  "target_drive_speed": 48,
  "steering_active": "true",
  "target_steering_angle": 90,
  "current_steering_angle": 89,
  "current_steering_speed": 12
}
```

---

## Watchdog

If no message is received for **2000ms**, motors halt automatically.  
Send `{"command": "heartbeat"}` periodically to keep the connection alive.
