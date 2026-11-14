# VR gimbal

This section controls Robot AR's two-axis camera gimbal through the Waveshare
**Bus Servo Adapter (A)** and two Feetech STS3215 servos. This is the direct
USB/UART adapter commonly used with SO-101 robot arms; it is not the Waveshare
ESP32 servo driver. The first test uses the keyboard. The same controller also
accepts streamed VR yaw/pitch values.

## Safety first

- Test without a camera or payload first.
- Support the gimbal by hand and keep cables clear of both axes.
- The adapter has no voltage regulator. Supply the voltage specified on your
  exact STS3215 servo label (normally a suitable 2S/7.4 V-class supply), with
  enough current for two stalled servos. Never apply 12 V to a 7.4 V servo.
- USB-C is the data connection; power the servo bus through either the DC jack
  or green screw terminal. Do not connect both external power inputs at once.
- Match the servo cable to `D` (data), `V` (positive) and `G` (ground).
- Confirm that yaw and pitch have **different servo IDs**. The example uses
  yaw `1` and pitch `2`; two factory-ID-1 servos cannot share the bus safely.
- Set conservative `min`/`max` values in `config/gimbal.json` before enabling
  torque. The supplied values are examples, not measured mechanical limits.
- The software connects with torque off and holds the measured pose only after
  you press `E`. `SPACE` and program exit both disable torque.
- Use a suitable external servo supply and common ground. Do not power two
  STS3215 servos from the Mini PC's USB port.

## Configuration

### Set the adapter for direct USB control

1. With power disconnected, move the yellow control-mode jumper to **B
   (USB)**. Position A is only for control from the separate `TX/RX/GND` UART
   header.
2. Connect yaw and pitch servos to either of the identical `D/V/G` sockets.
3. Connect the correctly rated external servo supply to the DC jack **or** the
   green screw terminal. Check polarity before applying power.
4. Connect USB-C to the Mini PC. The onboard CH343 appears as a serial device;
   no ESP32 firmware, web page or serial-forwarding mode is involved.

The serial baud must match the connected servos. Factory STS3215 servos are
normally `1000000` baud, which is the default in `gimbal.json`. If your SO-101
setup deliberately changed the servo baud, put that same value in the config.

Find the stable Linux port after plugging in the adapter:

```bash
ls -l /dev/serial/by-id/
```

Copy that path into `config/gimbal.json`. Adjust:

- `yaw.id` / `pitch.id`: physical servo IDs.
- `center`: desired forward-looking position.
- `min` / `max`: measured collision-free position limits (`0..4095`).
- `direction`: set to `-1` when an axis moves opposite to the VR input.
- `speed`, `acceleration`, and `keyboard_step`: begin slowly.

If the CH343 port exists but cannot be opened, add the user running the program
to the Linux `dialout` group, then sign out and back in. Do not run the entire
robot stack as root just to access a serial port.

## Keyboard test

First verify the software without hardware:

```bash
python3 -m vr_gimbal.keyboard_control --dry-run
```

Then connect the adapter and run:

```bash
python3 -m vr_gimbal.keyboard_control \
  --port /dev/serial/by-id/your-waveshare-adapter
```

| Key | Action |
|---|---|
| `E` | Enable torque and hold the current pose |
| `A` / `D` | Yaw left / right |
| `W` / `S` | Pitch up / down |
| `C` | Move to the configured center (only after limits are verified) |
| `Space` | Torque off immediately |
| `Q` | Torque off and quit |

If an axis moves the wrong way, change its `direction`; this applies to both
keyboard and VR control.

## VR input stream

`stream_control.py` reads one JSON object per line from standard input. The
first pose becomes the neutral headset pose and maps to the gimbal's current
position, avoiding a startup jump.

Euler input:

```json
{"yaw_deg": 12.4, "pitch_deg": -3.0}
```

Quaternion input (common Z-yaw/Y-pitch conversion):

```json
{"quaternion": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}
```

Test a recorded stream without hardware:

```bash
printf '%s\n' \
  '{"yaw_deg":0,"pitch_deg":0}' \
  '{"yaw_deg":10,"pitch_deg":-5}' | \
  python3 -m vr_gimbal.stream_control --dry-run
```

Control records are also accepted:

```json
{"command":"recenter"}
{"command":"stop"}
{"command":"enable"}
```

The VR headset's coordinate convention may differ. Keep the gimbal on a stand,
move one headset axis at a time, then use `direction` or a small input adapter
to swap/invert axes. Never remove the hard servo limits.
