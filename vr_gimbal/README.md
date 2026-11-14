# VR gimbal

This section controls Robot AR's two-axis camera gimbal through the Waveshare
**Servo Driver with ESP32** and two Feetech STS3215 servos. The first test uses
the keyboard. The same controller also accepts streamed VR yaw/pitch values.

## Safety first

- Test without a camera or payload first.
- Support the gimbal by hand and keep cables clear of both axes.
- Confirm that yaw and pitch have **different servo IDs**. The example uses
  yaw `1` and pitch `2`; two factory-ID-1 servos cannot share the bus safely.
- Set conservative `min`/`max` values in `config/gimbal.json` before enabling
  torque. The supplied values are examples, not measured mechanical limits.
- The software connects with torque off and holds the measured pose only after
  you press `E`. `SPACE` and program exit both disable torque.
- Use a suitable external servo supply and common ground. Do not power two
  STS3215 servos from the Mini PC's USB port.

## Configuration

### Put the Waveshare board in USB forwarding mode

The Waveshare example firmware normally controls the servos itself. For this
Python controller, open the board's web page and start **Serial Forwarding**.
The OLED should show `SERIAL_FORWARDING`. In the vendor example this is command
14; alternatively set `SERIAL_FORWARDING = true` in `ServoDriver.ino` and
upload the firmware again.

The ESP32 USB serial side is `115200` baud, as set in `gimbal.json`. Its
`Serial1` servo side remains at the STS bus rate internally. If you later
replace the ESP32 board with a direct Waveshare bus-servo USB adapter, change
the configured baud to the rate used by that adapter/servo bus (commonly
`1000000`).

Find the stable Linux port after plugging in the Waveshare adapter:

```bash
ls -l /dev/serial/by-id/
```

Copy that path into `config/gimbal.json`. Adjust:

- `yaw.id` / `pitch.id`: physical servo IDs.
- `center`: desired forward-looking position.
- `min` / `max`: measured collision-free position limits (`0..4095`).
- `direction`: set to `-1` when an axis moves opposite to the VR input.
- `speed`, `acceleration`, and `keyboard_step`: begin slowly.

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
