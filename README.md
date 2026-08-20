# Garden Spine, The Bench Watcher

We made a Bench Watcher that can detect privatly on how much a bench is occupied, made with only one ESP32 kit.

`GardenSpine.h` handles Wi-Fi, the clock, TLS, MQTT reconnection, topic construction, and JSON
formatting. You write the line that says what you measured. Everything else — sensing, timing,
movement, enclosure, interpretation — is yours.

## What you can call

```cpp
#include <GardenSpine.h>

GardenSpine spine;

void setup() {
  Serial.begin(115200);
  spine.begin();          // Wi-Fi, clock, TLS, MQTT. Call once.
}

void loop() {
  spine.loop();           // Keeps the connection alive. Call every loop, never inside a delay.
  spine.publish("temperature", 23.4, "celsius");   // number
  spine.publish("status", "ok", "enum");           // text
}
```

| Call | Returns | Notes |
|---|---|---|
| `spine.begin()` | — | Connects. Call once in `setup()`. |
| `spine.loop()` | — | Call at the top of every `loop()`. Reconnects and resubscribes by itself. |
| `spine.connected()` | `bool` | True when Wi-Fi and MQTT are both up. |
| `spine.publish(measurement, value, unit)` | `bool` | `value` may be a number or text. Returns false and says why if the sensor returned no number. |
| `spine.subscribe(topic, handler)` | `bool` | Up to 8 topics. Works before or after `begin()`. |

Your handler receives readable text, not raw bytes:

```cpp
void showCounter(const char* topic, const char* payload) {
  Serial.println(payload);
}

spine.subscribe("garden/entrance/counter/gk-01/count", showCounter);
```

The topic is built for you from `config.h` plus the measurement you pass:
`garden/<zone>/<device-type>/<device-id>/<measurement>`. Every payload carries the five contract
fields, so you never format JSON by hand. See the
[data contract](https://gardenspine.ikapo.fi/spec) for what the backbone accepts.

## No board yet?

`laptop/` publishes from your laptop with Python, so a team without hardware can still reach the
dashboard on day one.

```bash
cd laptop
python -m pip install paho-mqtt
cp config.py.example config.py     # then fill in your credentials
python first_message.py
```

Put the downloaded CA certificate beside `config.py` as `garden-spine-ca.crt`.

## What is in here

| Path | What it is |
|---|---|
| `src/GardenSpine.h` | the library. You never edit this |
| `src/spine_ca.h` | the programme CA, baked in so nobody has to paste a certificate. Public, not a secret |
| `config.h.example` | template for your credentials. Copy it into your sketch folder as `config.h` |
| `examples/` | the five tutorial sketches |
| `laptop/` | the Python publisher, for teams without hardware |
| `docs/HARDWARE.md` | kit inventory and electrical safety. **Read before wiring motors or the relay** |
| `BUILD_LOG.md` | your fortnightly evidence log, a programme deliverable |

## Team deliverables

- A working project sketch in your fork.
- [`BUILD_LOG.md`](BUILD_LOG.md), updated twice each week.
- A short calibration or engineering evaluation.
- A documented cross-team subscription.
- One creative measurement under your own device topic.
- A final README with your wiring, limitations, and evidence from the dashboard.

## Rules

- Never commit `config.h` or `laptop/config.py`. They hold your password.
- Never use `client.setInsecure()`. A certificate error is a real error.
- Never publish over plaintext port `1883`.
- Never publish under another team's device prefix. The broker refuses it anyway.

Stuck? [Troubleshooting](https://gardenspine.ikapo.fi/troubleshooting) lists every reject reason
with the fix.
