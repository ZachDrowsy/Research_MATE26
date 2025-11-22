+47
-0

# Motor Control and Telemetry Code Walkthrough

This document annotates `motor_res.cpp` line-by-line for a novice reader, explains why each library is used, and suggests alternative options where appropriate. Section references point to line numbers in the source file.

## Library choices
- **HardwareSerial / Arduino core (`<HardwareSerial.h>`, `<Arduino.h>`)**: Provide direct access to hardware UARTs and core MCU utilities like `pinMode` and timing. Chosen because ESP32 boards expose multiple hardware UARTs for reliable LoRa communication without CPU-heavy bit banging. *Alternative*: `SoftwareSerial` (or `AltSoftSerial`) emulates UART on GPIO but is less reliable at higher baud rates and consumes more CPU, making it inferior for simultaneous motor control and sensor sampling.
- **`<ArduinoJson.h>`**: Used to assemble compact JSON packets for LoRa (lines 35–63). Static allocation avoids heap fragmentation on microcontrollers. *Alternatives*: Manual string concatenation (harder to maintain, easier to break JSON syntax) or CBOR/MessagePack libraries (smaller payloads but require custom decoding on the receiver).
- **`<Wire.h>`**: I²C bus utilities for the MS5837 pressure sensor (lines 7–8). Arduino’s core `Wire` is standard and well-tested; alternatives like TinyWire are for smaller AVR chips and offer no advantage on ESP32.
- **`"MS5837.h"`**: Vendor driver for the Blue Robotics barometric pressure sensor (lines 5–8). It abstracts calibration and depth conversion. Writing a custom driver would be risky and time-consuming given the sensor’s calibration coefficients and fluid-density compensation.
- **`<PID.h>`**: Provides a lightweight PID controller (lines 5–8, 90–109). Useful for smoothing depth control without reinventing control math. *Alternatives*: Rolling your own PID loop is possible but the library includes anti-windup and tuning helpers.
- **`<TMCStepper.h>`**: Configures the TMC2209 stepper driver over UART (lines 21–38, 87–109). This library exposes driver features like microstepping, spreadCycle, and current limits. An alternative is basic step/dir pulse generation without UART setup, but that would forfeit current limiting, stall detection, and smoothness controls.

## Global configuration and data structures
- GPIO and driver configuration (lines 10–38) define LoRa UART pins, stepper control pins, and TMC driver parameters such as sense resistor value and UART address. This centralization makes hardware changes easier.
- `SensorData` and `jsonString` buffers (lines 40–58) collect pressure/depth samples and serialized JSON messages. Fixed-size arrays prevent dynamic allocation issues during mission runs.

## Helper functions for JSON packing
- `writeSensorData` (lines 60–74) and `writeData` (lines 76–89) serialize structured data into short JSON strings. They use `StaticJsonDocument` so that buffer sizes are compile-time–checked and no heap is used. The small capacities (1–4 keys) keep packets minimal for LoRa bandwidth.

## Run-time limits
- `check_json` (lines 91–98) stops motor operations once 198 JSON packets are queued, preventing buffer overruns and physically backing out the actuator by reversing direction. This is a safety guard against memory misuse and mechanical over-travel.

## `setup()` overview
- LoRa initialization (lines 102–140) configures the REYAX RYLR897 module with address, network ID, frequency, and modulation parameters. Using hardware serial at 115200 baud ensures timely link setup. Commands are spaced with `delay` to allow the module to respond.
- GPIO setup and TMC2209 configuration (lines 145–168) set `step`, `dir`, and `enable` pins as outputs, then program the driver for full-step mode, spreadCycle, and a ~675 mA RMS current. UART configuration is why `TMCStepper` is pulled in; without it, these driver features cannot be tuned at runtime.
- Pressure sensor initialization (lines 187–195) retries until the MS5837 responds, then sets the model and fluid density so depth readings are accurate for fresh water.

## `loop()` control flow
- LoRa pass-through (lines 199–208) lets a laptop/phone send AT commands directly to the module for debugging—useful during field tests.
- Command reception and motor start (lines 210–222): upon a `+RCV` payload with control byte `1`, the motor routine begins. Motor enable is deasserted (`LOW`) to allow motion, the LED is lit, and direction is set for water intake.
- Sensor sampling and PID bookkeeping (lines 229–254, 276–302, 319–341): the code reads pressure/depth, logs them to JSON, and runs `pid(current_depth, previous_error)` to compute a corrective term. The PID library manages proportional/integral/derivative math to smooth depth changes.
- Initial purge (lines 256–263): on the first run, the code reports a connection and fully intakes water to sink the vehicle. This establishes neutral buoyancy before the main cycle.
- Dive and climb phases (lines 272–318): motor direction is toggled with `dirPin` and motion is commanded via `step(...)`, a helper that generates timed step pulses. Timing values (e.g., `7362.5` steps at `1000 µs`) were empirically chosen for desired displacement.
- Telemetry burst (lines 304–316): collects 10 successive sensor readings and queues each as JSON, spaced one second apart, to track depth evolution during the dive.
- Uplink and shutdown (lines 343–365): when control byte `2` is seen, the queued JSON frames are transmitted over LoRa, then the motor is disabled and buffers reset. The LED and `enPin` are also set to safe states when a stop command (`0`) arrives.

## Step pulse helper
- `step` (lines 377–384) toggles the step pin with a configurable period, producing square pulses that the TMC2209 interprets as motion commands. Keeping this logic in a helper clarifies intent and avoids repeated timing code.

## Alternatives and potential improvements
- **Motor driver control**: If advanced TMC features are unnecessary, a simpler driver like the A4988 could be used with only `step/dir` signaling (no `TMCStepper`). The trade-off is losing current control, spreadCycle smoothing, and stall detection.
- **Control loop**: A more full-featured PID library (e.g., `PID_v1`) offers auto-tuning and output limits, but the current lightweight library likely minimized code size. Alternatively, a simple proportional controller could reduce complexity at the cost of responsiveness.
- **Messaging format**: To cut LoRa airtime, CBOR or a custom binary frame could replace JSON. That would shrink packets but make debugging harder without specialized tooling.
- **Memory management**: The fixed 200-element buffers are safe but inflexible. Using a ring buffer or streaming uploads could avoid the hard cap enforced by `check_json` while preventing overruns.
- **Timing**: Replacing busy-wait `delay` calls with non-blocking state machines or timers (e.g., `esp_timer`) would allow smoother multitasking during long waits.

Overall, the chosen libraries favor reliability and rapid development on ESP32 hardware: hardware UARTs for LoRa, vendor drivers for the MS5837 sensor, and TMCStepper for rich motor control. Alternatives exist, but most would sacrifice robustness or require significant rework.