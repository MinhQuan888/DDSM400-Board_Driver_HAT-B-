# DDSM400 / DDSM Driver HAT (B) — ESP32 firmware

The project now has four build environments. `main.c` contains the shared
UART, Wi-Fi and web application. `main_open.c`, `main_current.c`,
`main_speed.c` and `main_position.c` implement the four motor modes. Only
`main.c` defines `app_main()`. The default build is open loop.

| Environment | Mode | Web target | Stop behavior |
| --- | --- | --- | --- |
| `ddsm_open` | `0x00` | signed raw value, up to 30000 | sends raw zero |
| `ddsm_current` | `0x01` | signed current, up to 1.00 A | sends zero current |
| `ddsm_speed` | `0x02` | signed speed, up to 380 RPM | sends zero RPM |
| `ddsm_position` | `0x03` | direct motor angle, 0–180 degrees | holds target; Disable removes torque |

Current uses the DDSM400 command conversion, where raw 32767 represents
4 A; this build caps the target at 1 A. Position maps
0–32767 directly to 0–360 motor degrees and sends one target frame only.
The web position limit is 180° because 360° is the same absolute target as 0°.
Switch to
position only while stopped. The mirrored ID1 receives the opposite sign.
`/api/status` reports the position target that was sent. If a full physical
wheel turn requires a different calibration, the motor's mechanical reduction
must be measured separately; the position command itself is an absolute motor
position command.
The web page measures distance continuously while the motors are enabled. The
**Zero odometry** button captures the current two-wheel odometry as the new zero.
The mode ranges and position behavior follow the
[Waveshare DDSM Driver HAT (B) guide](https://www.waveshare.com/wiki/DDSM_Driver_HAT_%28B%29)
and the [DDSM400 motor protocol](https://www.waveshare.com/wiki/DDSM400).

## Two-wheel speed balancing

When **both** motors run in open, current or speed mode, `wheel_sync.c` reads
the measured RPM returned by each drive command every 250 ms. It gradually lowers the command to
the faster wheel (never raises either command above the selected target).
This keeps the two wheel speeds closer despite different load or motor
response. Selecting only ID1 or ID2 bypasses balancing. A persistent case
where one wheel is stationary and the other moves causes both motors to be
disabled. `/api/status` reports `actual_id1_rpm`, `actual_id2_rpm`,
`sync_error_rpm`, and each wheel's command scale; ID1 RPM is inverted into
vehicle-forward coordinates.

Position mode sends one absolute target to each motor. It does not guarantee
identical arrival time between wheels. The balancing gains are conservative and
should be checked on the actual robot under load.

Firmware for the ESP32 on the Waveshare DDSM Driver HAT (B), using ESP-IDF 5.x through PlatformIO. The driver protocol implementation is in `main/DDSM400.c` and is independent of ESP-IDF. The application in `main/main.c` owns UART2 and exposes a small local Wi-Fi control page.

## Important hardware setup

1. The Waveshare DDSM Driver HAT (B) schematic routes ESP32 **GPIO19 as UART TX** and **GPIO18 as UART RX** for the switched motor communication path. These are also the defaults in this project. Re-check the schematic if your PCB revision or wiring differs.
2. `DDSM_TX` and `DDSM_RX` remain configurable in PlatformIO menuconfig under `DDSM400 motor UART`; normally leave them at 19 and 18 for the HAT (B).
3. Connect the motor to the HAT, apply the correct motor supply, and set the HAT switch to the **ESP32** path when using the onboard ESP32 firmware. The USB path is for direct USB/UART control.
4. Fix the motor shaft / mounting points securely to a rigid test fixture and keep the wheel clear of people and obstructions. Do not simply place the hub motor on a tabletop and let its body rotate. Secure the board and keep hands clear of the wheel.

The motor UART is configured as **38400 baud, 8 data bits, no parity, 1 stop bit (8N1)**. This UART is separate from the ESP32 USB programming/monitor connection, which typically uses 115200 baud.

## Build and flash

Open the extracted `DDSM400_ESP32` folder (the one containing `platformio.ini`) in VS Code / PlatformIO. For example, build and upload speed mode:

```sh
pio run -e ddsm_speed
pio run -e ddsm_speed -t upload
pio device monitor -b 115200
```

Replace `ddsm_speed` with `ddsm_open`, `ddsm_current` or `ddsm_position`.
Plain `pio run` builds the default `ddsm_open` environment. Each environment
has its own `sdkconfig.ddsm_*` copied from the tested `sdkconfig.ddsm400` so
the four builds retain the same UART pins and 4 MB flash configuration.

## Android USB-C control build

`ddsm_mobile` builds [main_mobile_control.c](main/main_mobile_control.c).
Connect an Android phone to the ESP32 board with a USB-OTG Type-C data cable
and use a USB serial terminal at **115200 baud, 8N1**. The phone talks to
ESP32 UART0; the motors remain on UART2 at 38400 baud.
For the Android arrow controls (`U`/`D`), the firmware reads each motor's
measured RPM from its `0x65` response every 250 ms and reduces the open-loop
command to whichever wheel is faster. `L`/`R` turns bypass this correction.
If feedback is lost or one wheel stalls while the other moves, it commands
both wheels to stop. `FB 1` and `FB 2` lines report measured RPM in 0.1 RPM
units, so compare both values while driving to check the correction on the
actual vehicle. Equal motor RPM alone cannot correct unequal wheel diameter,
slip, or a mounting alignment problem.

```text
HELP
ENABLE speed
DRIVE speed 20
DRIVE speed -20
STOP
DISABLE
```

`DRIVE` accepts `open` (raw -30000..30000), `current` (-1..1 A), `speed`
(-380..380 RPM), or `position` (0..180 degrees). For both wheels, ID1 is
automatically sign-inverted for the mirrored mounting. The Android app must
send each command followed by `Enter`/newline and read the `OK` or `ERR`
response. Build and upload with:

```sh
pio run -e ddsm_mobile -t upload --upload-port COM9
```

GPIO19 TX / GPIO18 RX are preconfigured for the HAT (B); use menuconfig only if your revision or wiring differs. Flashing this project replaces the HAT's existing ESP32 application firmware. Keep a copy of the original firmware if you may need to restore it.

## Web control

After boot, the ESP32 starts a Wi-Fi access point:

- SSID: `DDSM400_ESP32`
- Password: `12345678`
- Web page: `http://192.168.4.1`
- Motor IDs: `2` and mirrored `1`

The motor **does not start automatically at boot**. The page gets its active
mode, target units and slider range from `/api/status`. It sends
`POST /api/drive?value=10&motor=both` (motor may also be `1` or `2`). The
value is raw in open loop, amperes in current loop, RPM in speed loop, or
degrees in position loop. Forward sends a positive value; Reverse sends a
negative value, with ID1 inverted for its mirrored mounting. Open, current
and speed targets refresh while Run is active. Position is a one-shot target;
Hold leaves the last target in the motor. Disable removes torque. A 360°
request is intentionally rejected by the web limit because it aliases to 0°.

## Driver API currently implemented

### Frame and protocol layer

- `ddsm_crc8()` — CRC-8/MAXIM over the first 9 bytes.
- `ddsm_mode()` — build `0xA0` mode / enable / disable frames.
- `ddsm_speed()` and `ddsm_drive_raw()` — build `0x64` drive frames.
- `ddsm_query()` — build mode or odometry query frames.
- `ddsm_set_id_frame()` — build the ID-change frame. Use only with one powered motor and send the same frame five times as specified by Waveshare.
- `ddsm_decode()` — validate expected ID, opcode and CRC, then decode supported feedback frames.

### Speed Loop API

```c
servo_init(exchange);             // bind the UART transport callback
servo_on(2);                      // enable and select Open Loop (default)
servo_on_mode(2, DDSM_SPEED);     // used by the ddsm_speed mode module
set_speed(2, 50.0f, true);         // +50 RPM, default acceleration
set_speed_soft(2, 50.0f, 10, true); // +50 RPM with configured ramp
set_speed(2, 50.0f, false);        // -50 RPM
stop(2);                          // 0 RPM, remains enabled
stop_emergency(2);                // protocol DISABLE; wheel may coast
```

Speed arguments are magnitudes from 0 to 380 RPM; `forward=true` encodes positive RPM and `forward=false` negative RPM. The drive setpoint uses 0.1 RPM units. `accel_time` is written to DATA[6] and is documented as milliseconds per RPM; `0` selects the motor default.

### Feedback and diagnostics

`get_feedback()`, `get_speed()`, `get_current()`, `get_temperature()`, `get_fault_flags()` and `GetAlarmType()` read the latest valid `0x65` actual feedback cached by a successful drive transaction. They do not issue an independent status query. Current conversion follows the supplied DDSM400 document's scale of raw `-32767..32767` to `-4..4 A`.

`GetAlarmType()` returns: `0` none, `1` overcurrent, `2` overload/stall, `3` overtemperature, `4` other motor/hardware fault. Raw fault bits remain available through `get_fault_flags()` after the driver requests a Disable, so the application can display the reason.

### Odometry

`get_distance()` sends the documented odometry query and combines the signed
32-bit mileage-lap counter with the 0..32767 position inside the current lap.
Maintain one `ddsm_distance_state` per motor. The web controller normalizes the
reversed ID1 direction, accumulates the mean travel of both wheels during normal
driving, and continues sampling after Stop while the motors remain enabled.
`WHEEL_DIAMETER_ID1_M` and `WHEEL_DIAMETER_ID2_M` are effective rolling
diameters and can be calibrated separately from a measured straight run.

Distance conversion uses:

```text
counts = mileage_laps * 32768 + position
metres = delta_counts * pi * wheel_diameter /
         (32768 * DDSM_ODOM_LAPS_PER_WHEEL_TURN)
```

`DDSM_ODOM_LAPS_PER_WHEEL_TURN` is set to `1.92`, calibrated from two manual
push tests: 0.060 m reported for 1.000 m actual and 0.030 m reported for
0.500 m actual. Confirm it on the real motor: disable the motors, press **Zero
odometry**, rotate ID2 exactly 10 physical wheel revolutions, then divide the
displayed ID2 counts by `10 * 32768`. Put that value in `main/DDSM400.h`.
After the lap ratio is correct, calibrate each effective wheel diameter with a
measured straight run: `new_diameter = old_diameter * real_distance /
displayed_distance`.

## Architecture and safety behavior

- Only `motor_control_task()` calls the DDSM library/UART. HTTP handlers put commands into a one-element queue, so old setpoints do not accumulate if UART replies are slow.
- `set_speed_soft()` returns false on missing/invalid feedback or non-zero fault bits. On a lost drive response, the library makes a best-effort Disable request. The application also requests Disable after a failed drive command.
- At startup, firmware requests Disable and does not automatically run a direction test.
- A `DISABLE` acknowledgement failure means the physical motor state is **not confirmed**. Do not interpret a software return value as a physical emergency-stop guarantee.

## Known scope limits

Automatic ID assignment, network authentication beyond the Wi-Fi AP password,
and hardware safety circuitry are outside this revision.
