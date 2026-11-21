# Getting started

## Prerequisites

- Arduino UNO Q
- PZEM-004T (or compatible) energy meter
- 4-channel 5 V relay module
- 5 V logic supply appropriate for relays and sensors
- Optional: I²C TFT HMI, MAX485, panel E-STOP

## Software

1. Flash `firmware/sketch/sketch.ino` to the MCU side (Arduino IDE / App Lab as appropriate for UNO Q).
2. On the Linux side:
   ```bash
   cd ai/python   # or python/
   pip install -r requirements.txt
   # ensure weights.pkl, scaler.pkl, model.eim are present
   python main.py
   ```
3. Open the web dashboard served by the AI process (see App Lab / `app.yaml` port notes).

## Bring-up order (recommended)

1. AI engine alone with synthetic or serial test vectors  
2. Serial telemetry path  
3. Relay outputs (no load)  
4. Local HMI  
5. RS-485 TX  
6. PZEM measurement hardware  
7. Full closed loop with limited loads  

Do **not** wire everything at once on first power-up.

## Synthetic / offline testing

See `scripts/generate_synthetic_samples.py` for physics-informed sample generation used to expand a seed laboratory set during development.
