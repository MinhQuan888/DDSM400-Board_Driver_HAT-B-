# Host-side protocol tests

These tests exercise the platform-independent `main/DDSM400.c` without an ESP32 or motor.

With GCC installed, from the project root run:

```sh
gcc -std=c11 -Wall -Wextra -Wpedantic -I main main/DDSM400.c main/main_open.c main/main_current.c main/main_speed.c main/main_position.c tests/test_driver.c -lm -o test_driver
./test_driver
gcc -std=c11 -Wall -Wextra -Wpedantic -I main main/wheel_sync.c tests/test_wheel_sync.c -lm -o test_wheel_sync
./test_wheel_sync
```

The tests check frame encoding, the four mode handshakes, current and angle
conversions, calibrated 96-step wheel-turn planning, position feedback querying,
feedback conversion, and fault-to-disable behavior. They do **not**
test the UART electrical layer, HAT switch, motor mechanics, or a real DDSM400 motor.
