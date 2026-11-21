# Feature vector (14-D)

| Index | Name | Notes |
|-------|------|--------|
| 0 | Voltage | Instantaneous |
| 1 | Current | Instantaneous |
| 2 | Power | |
| 3 | Energy | Accumulated |
| 4 | Frequency | |
| 5 | Power Factor | |
| 6 | Load Priority | Context / policy hint |
| 7 | Voltage Deviation | vs nominal (e.g. 230 V) |
| 8 | PF Deviation | e.g. 1 − PF |
| 9 | Utilization | Current vs reference rating |
| 10 | dI/dt | Rate of change of current |
| 11 | dV/dt | Rate of change of voltage |
| 12 | Current Variance | Short-window instability |
| 13 | Thermal Index | I²t-style accumulated stress (calculated, not a thermometer) |

Scaling: `scaler.pkl` (mean / scale) applied before the MLP.

## Model artefacts

| File | Purpose |
|------|---------|
| `weights.pkl` | MLP layer weights & biases (14→32→16→4) |
| `scaler.pkl` | Standardisation parameters |
| `model.eim` | Edge Impulse anomaly runner package |
