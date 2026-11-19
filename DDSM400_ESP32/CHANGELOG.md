# Changes in this revision

- Restored the earlier bench-test settings: motor ID 2, 10 RPM web limit, 200 ms/RPM acceleration, and four Wi-Fi clients.
- Fixed ESP-IDF 5.4 compilation by sending HTTP 503 with an explicit status string.
- Matched the ESP32-WROOM-32E-N4 board configuration to 4 MB flash.
- Moved UART initialization and the startup Disable request ahead of Wi-Fi initialization.
- Kept the protocol/frame layer platform-independent and added full feedback/fault getters.
- Added best-effort Disable after a missing drive response or non-zero motor fault feedback.
- Made UART ownership explicit: only the motor-control task accesses the DDSM bus.
- Replaced the automatic forward/reverse boot test with an explicit web control page.
- Added a 750 ms dead-man timeout while a drive command is being held/refreshed.
- Added HTTP endpoints for drive, stop, enable, disable, and status.
- Configured the HAT (B) UART defaults as TX GPIO19 / RX GPIO18, with menuconfig overrides.
- Added host-side tests for the protocol library.
