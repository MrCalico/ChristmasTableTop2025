## Purpose
Help AI coding agents be immediately productive in this repository: an ESP32/PlatformIO project that animates WS2812 LEDs and plays sounds from a DFPlayer Mini over UART.

Keep guidance short and actionable. When in doubt, change only `src/main.cpp` behavior in a conservative way and prefer preserving existing wiring, pin choices, and timing constants.

## Big picture
- Single microcontroller target (ESP32) using PlatformIO. Build config: `platformio.ini` in repo root.
- Main application is a single-file program in `src/main.cpp` that:
  - drives a long WS2812 LED strip (NUM_LEDS = 1088) using FastLED
  - controls a DFPlayer Mini via HardwareSerial (UART2) to play SD card tracks
  - reads two ADC pots for volume and speed control (GPIO34/35)
  - uses a BUSY pin (GPIO27) to monitor DFPlayer playback state

## Key files & locations
- `src/main.cpp` — primary implementation; contains all behavior and hardware pin mappings. Use this file as the canonical source for wiring, pin choices, and timing.
- `platformio.ini` — build/board settings (target platform, envs). Always consult it before changing build-related code.
- `assets/mp3/` — audio files expected on the DFPlayer Mini SD card; filenames are numeric (e.g. `0011.mp3`) and tracks are referenced by numeric index in code (e.g. `QueueTrack(11)` plays track 11).
- `.vscode/extensions.json` — recommended editor extensions (non-blocking) for local dev.

## Build / Flash / Debug workflows
- Use PlatformIO commands in VS Code or the terminal from repo root. Typical commands:
  - Build: `platformio run`
  - Upload/Flash: `platformio run --target upload`
  - Monitor serial: `platformio device monitor` (baud 9600 in code)
- The project uses ESP32-specific APIs (analogReadResolution, analogSetPinAttenuation). Ensure the selected PlatformIO board in `platformio.ini` matches your ESP32 board variant.

## Project-specific conventions and patterns
- Single-file program: prefer minimal invasive edits. If adding features, factor new code into `lib/` and `include/` and update `platformio.ini` if extra libraries are required.
- Hardware serial usage: `HardwareSerial ExtSerial(2)` with RX=16, TX=17 is chosen to reduce interference with ADC pins. Preserve UART2 usage unless you update pin comments and hardware wiring.
- ADC pots: code documents known ADC loading/noise issues. If changing ADC pins or resistor values, also update the mapping comments and the read mapping code in `loop()`.
- Audio tracks: audio is queued by numeric track index. Do not rename files in `assets/mp3` unless you also update the SD card and code references.

## Safety and non-functional constraints
- The code expects a long LED array (NUM_LEDS = 1088). Avoid changes that assume fewer LEDs without updating all indexing (wrapping arithmetic uses NUM_LEDS heavily).
- DFPlayer initialization blocks on failure (infinite loop). If modifying startup behavior, preserve a safe fallback (e.g., log and halt) rather than leaving silent failures.
- Timing: many delays (delay(trainSpeedMs), pot read intervals, and DFPlayer polling) influence behavior. Keep timing changes conservative and test on device.

## Examples of targeted edits an agent may perform
- Change default train speed constant: update `TRAIN_SPEED` near top of `src/main.cpp` and keep variable `trainSpeedMs` usage intact.
- Add a new audio cue: add numbered mp3 to `assets/mp3/` and call `QueueTrack(n)` with appropriate duration/volume.
- Extract LED drawing into a helper: create `lib/leds/` or `src/leds.cpp` and move the LED drawing loop there; keep public API compatible (advance position, show).

## Testing notes
- No automated tests in repo. Verify changes by building and running on hardware. Use the serial monitor (9600 baud) to inspect debug prints (`Serial.println` calls are present throughout).
- Quick smoke test after edits: build + upload + monitor serial; observe LED behavior and DFPlayer tracks.

## When to ask the human
- Any change touching pin assignments, ADC attenuation, or NUM_LEDS should be verified with the human before merge.
- If you need to add new libraries to `platformio.ini`, confirm the target board and desired framework (Arduino) with the human.

## Minimal checklist for PRs from AI agents
1. Build passes locally with `platformio run`.
2. No change to wiring/pin comments unless also updated in code and PR description.
3. If audio files are added, list exact filenames and intended track indexes in PR description.
4. Run basic smoke test on device and paste key serial logs (startup, DFPlayer init, one loop iteration).

-- End of file
