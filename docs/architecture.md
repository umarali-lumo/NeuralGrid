# Architecture

## Dual-processor split (Arduino UNO Q)

| Side | Role |
|------|------|
| **STM32 MCU** | Real-time: sensor UART/telemetry, feature helpers, relay GPIO, HMI I²C, RS-485 TX, E-STOP, policy execution |
| **Linux MPU** | Analytics: feature assembly, MLP inference, Edge Impulse anomaly, fusion, health, web dashboard |

Bridge: UNO Q RouterBridge (`Bridge.call("predict_json")` pattern in firmware/AI).

## Data path

1. PZEM-004T provides V, I, P, E, PF, f.  
2. STM32 validates and may compute local derivatives.  
3. Feature vector (14-D) is presented to the Linux AI.  
4. Primary NN → 4-class state + confidence.  
5. Edge Impulse model → anomaly score/flag.  
6. Safety engine applies hard limits.  
7. Fusion produces policy command.  
8. STM32 maps command to relay channels (Critical / High / Low / Auxiliary).

## Why separation matters

If the AI process stalls, returns nonsense, or is offline, the MCU can still enforce limits and E-STOP. This is the core safety argument of the architecture versus "model directly drives GPIO."
