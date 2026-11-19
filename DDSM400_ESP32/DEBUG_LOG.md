# Open-loop diagnostic build

Select the `ddsm_open` build environment for these diagnostics. The web Run
controls then switch both DDSM400 motors to open-loop mode `0x00`.
The command is a signed **raw drive value**, not RPM. ID1 is mirrored relative
to ID2. The slider starts at raw 2000 and is limited to 0..30000; the API
accepts integers from -30000 to 30000 at `/api/drive?value=2000&motor=1`.
Stop sends raw zero. Distance moves are unavailable in this mode.

While a continuous Run command is active, the ESP32 re-sends the same raw
command every 500 ms with `set_open_loop()` to obtain fresh `0x65` feedback.
Each `LIVE` line shows the signed raw command, measured RPM, current,
temperature, and the raw fault code for the selected motor. `fault=0x00`
means no reported fault. If a command fails, the log prints the last known
fault code, when available, plus any UART response error. A last known fault
may be stale after a missing reply.

From the project directory, close any Serial Monitor using COM9, then run:

```sh
pio run -e ddsm_open -t upload --upload-port COM9
pio device monitor -p COM9 -b 115200
```

Run ID1, ID2, then both for at least ten seconds each. Note when jitter
occurs and send the monitor output from before Run until after Stop. Do not
compare a raw value numerically with an RPM target from the previous build.
The DDSM400 wiki gives open-loop examples at raw 2000, 5000, and 10000.
Try 2000 first, then 5000 if the motor does not start.
