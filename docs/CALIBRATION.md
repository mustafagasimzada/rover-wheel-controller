# Encoder Calibration

The AS5600 is a magnetic encoder — it has no absolute zero. Each wheel needs a calibration offset so that 0° corresponds to the straight-ahead position.

## Current offsets

```c
const int encoder_offsets[4] = {136, 129, 157, 150};
```

These are defined in `main.c`. `WHEEL_NUMBER` selects the correct offset at compile time.

## How to recalibrate a wheel

1. Physically align the wheel to straight-ahead
2. Connect to the wheel over TCP and send:
   ```json
   {"command": "get_status"}
   ```
3. Note the `current_steering_angle` value in the response
4. That value is your new offset for this wheel
5. Update `encoder_offsets[WHEEL_NUMBER - 1]` in `main.c` and reflash

## Known limitation

Offsets are hardcoded — any mechanical realignment requires a recompile and reflash.  
Planned fix: store offsets in RP2040 flash memory so they can be updated at runtime without recompiling.
