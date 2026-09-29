# Bench Watcher, Garden Spine

A small robot that rides along a park bench and measures **how many people are sitting on it and for how long**, without a camera. It runs on a single ESP32 and sends its results to an online dashboard.

Built during the Garden Spine summer programme at the **University of Oulu** (ITEE), funded by the **European Union**.

---

## At a glance

| | |
|---|---|
| **Problem** | City planners want to know how public benches are used, but cameras raise privacy concerns. |
| **Solution** | An ultrasonic distance sensor on a moving robot scans the bench. It measures distance only, so it never records a face or an image. |
| **Output** | Number of people on the bench and their average sitting time, published every 30 s to a live dashboard. |
| **Hardware** | ESP32, HC-SR04 ultrasonic sensor, 2 × 28BYJ-48 stepper motors, 2 × servo motors |
| **Software** | C++ (Arduino framework), MQTT over TLS, state machines, non-blocking timing |

---

## How it works

1. **Scan.** Two stepper motors drive the robot back and forth along a track beside the bench. An ultrasonic sensor points at the seats and takes a distance reading every 50 ms.
2. **Locate.** The robot keeps track of where it is on the track by combining its known speed with elapsed time (dead reckoning). Every reading is therefore tied to a position on the bench.
3. **Detect.** When the sensor sees something closer than the bench's back (under 200 cm), the robot marks the start of an object. When the reading goes back to normal, it marks the end. The distance between the two is the object's **width**.
4. **Classify.** The width tells the robot what it found:
   - under 25 cm → small object (for example a bag)
   - 40 cm or more → a person; wider groups are counted as roughly one person per 50 cm
5. **Track.** Each detection is stored with its position and the time it was first seen. On the next pass, a detection at the same spot is matched to the same occupant, so the robot knows how long they have been there. If a spot stays empty for 5 seconds, that occupant is marked as having left.
6. **React.** When someone new sits down, the robot stops for 5 seconds and a bird mounted on top flaps its wings (two mirrored servos). This makes the device visible and a bit playful in a public space.
7. **Report.** Every 30 seconds the ESP32 publishes two values over secure MQTT to the Garden Spine dashboard:
   - `bench_occupant_count` : people currently on the bench
   - `bench_avg_sit_time` : their average sitting time in seconds

---

## Two versions

### Version 1: Simple ([Bench-WatcherV1/simpleVersion1](Bench-WatcherV1/simpleVersion1/simpleVersion1.ino))

The first prototype, focused on the mechanical bird. The two wing servos move as a mirrored pair: a slow flap most of the time and a fast "alarm" flap for 5 seconds every 10 seconds. It has no sensing or network code, which kept it simple to build and test the moving parts.

### Version 2: Advanced ([Bench-WatcherV2/advancedVersion](Bench-WatcherV2/advancedVersion/advancedVersion.ino))

The full system described above: moving robot, ultrasonic scanning, per-person tracking, sit-time statistics and dashboard reporting.

---

## Technical highlights

- **Everything runs at once on one small chip.** The code never pauses to wait. Motors, wing servos, sensor readings and network traffic are all handled in the same loop using timers (`millis()`), so no single task blocks the others.
- **State machines.** The robot's behaviour is split into clear states: *running → stopped (new person) → cooldown → running*. A separate state machine handles *forward → pause → backward → pause*. This makes the behaviour predictable and easy to debug.
- **Live segmentation.** A person is confirmed as soon as 25 cm of them has been scanned, rather than waiting for the full pass. This makes the reaction feel immediate. The width estimate keeps updating as the scan continues.
- **Noise handling.** Short gaps in the signal (under 400 ms) are ignored so one bad reading does not split a person in two. Very narrow blips (under 15 cm) are discarded.
- **Privacy by design.** The sensor only measures distance. No images or personal data are collected or sent.
- **Secure communication.** Data is sent over MQTT with TLS encryption using the programme's `GardenSpine` library ([src/GardenSpine.h](src/GardenSpine.h)).

---

## Hardware and wiring (Version 2)

| Component | ESP32 pins |
|---|---|
| HC-SR04 ultrasonic sensor | Trigger 2, Echo 4 |
| Wing servo 1 / servo 2 | 5 / 15 |
| Stepper motor 1 (IN1–IN4) | 26, 27, 14, 25 |
| Stepper motor 2 (IN1–IN4) | 23, 22, 21, 19 |

---

## Running it yourself

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) with ESP32 board support.
2. Install the libraries **ESP32Servo**, **PubSubClient** and **Stepper**, and add this repository as a library so `GardenSpine.h` is found.
3. Copy [config.h.example](config.h.example) to `config.h` and fill in your Wi-Fi and MQTT credentials. Do not commit this file.
4. Open one of the sketches, select your ESP32 board and upload.
5. Open the Serial Monitor at **115200 baud** to see detections as they happen, for example:

   ```
   New occupant tracked at 142.50 cm, width 52.00 cm (medium, person)
   Reported occupancy: 1 people, avg sit time 34.20 s
   ```

---

## Limitations and next steps

- Position is estimated from speed and time, so small errors add up over long runs. Wheel encoders or end-stop switches would fix this.
- Width-based classification cannot tell a large bag from a small person. A second sensor at a different height could help.
- Someone standing in front of the bench is currently counted like someone sitting. A minimum-distance filter is planned.

---

## Acknowledgements

Funded by the European Union in collaboration with the University of Oulu. The `GardenSpine` communication library and the example sketches in [examples/](examples/) come from the programme's [starter repository](https://github.com/ITEE-IKAPO/ladybug-starter).
