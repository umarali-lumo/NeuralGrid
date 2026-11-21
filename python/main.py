# ============================================================
# 
# Arduino UNO Q 
#
# STM32 MCU
#     |
#     | Bridge.call("predict_json")
#     v
# Qualcomm Linux MPU
#     |
#     +-- Primary AI
#     |     14-Feature Neural Network
#     |
#     +-- Secondary AI
#     |     Edge Impulse Anomaly Detector
#     |
#     +-- Safety Engine
#     |
#     +-- Decision Fusion
#     |
#     +-- Emergency Stop State
#     +-- WebUI / REST API
#           |
#           +-- /api/status
#           |
#           v
#     Industrial NeuralGrid Dashboard
#
# ============================================================

import os
import sys
import json
import time
import pickle
import subprocess
import threading

import numpy as np

from collections import deque

from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI

from edge_impulse_linux.runner import ImpulseRunner


# ============================================================
# SYSTEM INFORMATION
# ============================================================

APP_NAME = "NeuralGrid"
APP_VERSION = "3.0"
SYSTEM_NAME = "NeuralGrid Edge AI Industrial Grid Intelligence"


# ============================================================
# STARTUP CLEANUP
#
# Prevent accidental duplicate main.py processes.
# ============================================================

CURRENT_PID = str(os.getpid())

try:

    running_pids = subprocess.check_output(
        ["pgrep", "-f", "main.py"]
    ).decode().split()

    for pid in running_pids:

        if pid != CURRENT_PID:

            try:
                os.kill(int(pid), 9)
            except Exception:
                pass

except Exception:
    pass


# ============================================================
# BASE DIRECTORY
# ============================================================

BASE_DIR = os.path.dirname(
    os.path.abspath(__file__)
)


# ============================================================
# MODEL PATHS
# ============================================================

WEIGHTS_PATH = os.path.join(
    BASE_DIR,
    "weights.pkl"
)

SCALER_PATH = os.path.join(
    BASE_DIR,
    "scaler.pkl"
)

EI_MODEL_PATH = os.path.join(
    BASE_DIR,
    "model.eim"
)


# ============================================================
# CLASS DEFINITIONS
# ============================================================

CLASS_MAPPING = {

    0: "NORMAL",

    1: "LOAD_INCREASE_WARNING",

    2: "OVERLOAD_RISK",

    3: "CRITICAL_CONDITION"

}


CLASS_SHORT = {

    0: "NORMAL",

    1: "WARNING",

    2: "OVERLOAD",

    3: "CRITICAL"

}


# ============================================================
# SYSTEM PARAMETERS
# ============================================================

SAMPLE_TIME = 0.20

HISTORY_LENGTH = 60

AXES_COUNT = 8

EI_ANOMALY_THRESHOLD = 75.0


# ============================================================
# HISTORY BUFFERS
#
# Used by:
#   dI/dt
#   dV/dt
#   variance
#   thermal index
#   dashboard graphs
# ============================================================

voltage_history = deque(
    maxlen=HISTORY_LENGTH
)

current_history = deque(
    maxlen=HISTORY_LENGTH
)

power_history = deque(
    maxlen=HISTORY_LENGTH
)

pf_history = deque(
    maxlen=HISTORY_LENGTH
)

frequency_history = deque(
    maxlen=HISTORY_LENGTH
)

thermal_history = deque(
    maxlen=HISTORY_LENGTH
)

timestamp_history = deque(
    maxlen=HISTORY_LENGTH
)


# ============================================================
# EDGE IMPULSE BUFFER
# ============================================================

ei_window_buffer = deque()


WINDOW_SAMPLES = 1


# ============================================================
# WEB UI INITIALIZATION
# ============================================================

print()
print("==================================================")
print(" NEURALGRID — WEBUI INITIALIZATION")
print("==================================================")

web_ui = WebUI()

print("[WebUI] WebUI object initialized.")


# ============================================================
# LATEST SYSTEM STATE
#
# This MUST exist before the first telemetry packet.
#
# Therefore:
#
# http://UNO_Q_IP:7000/api/status
#
# works immediately.
# ============================================================

latest_state = {

    # --------------------------------------------------------
    # SYSTEM
    # --------------------------------------------------------

    "status": "ONLINE",

    "system": APP_NAME,

    "version": APP_VERSION,

    "timestamp": time.time(),

    "uptime": 0,

    # --------------------------------------------------------
    # EMERGENCY STOP
    # --------------------------------------------------------
    "emergency_stop": False,
    "emergency_stop_status": "CLEAR",
    "emergency_stop_source": "UNO_Q_GPIO_5",

    # --------------------------------------------------------
    # TELEMETRY
    # --------------------------------------------------------

    "voltage": 230.0,

    "current": 0.0,

    "power": 0.0,

    "power_factor": 0.95,

    "frequency": 50.0,

    "energy": 0.0,

    # --------------------------------------------------------
    # HEALTH
    # --------------------------------------------------------

    "health": 100,

    "health_status": "HEALTHY",

    # --------------------------------------------------------
    # PRIMARY AI
    # --------------------------------------------------------

    "ai_class": "NORMAL",

    "ai_class_id": 0,

    "ai_confidence": 0.0,

    # Individual probabilities

    "ai_probabilities": [
        0.25,
        0.25,
        0.25,
        0.25
    ],

    # --------------------------------------------------------
    # EDGE IMPULSE
    # --------------------------------------------------------

    "anomaly": False,

    "anomaly_score": 0.0,

    "anomaly_threshold": EI_ANOMALY_THRESHOLD,

    "anomaly_status": "CLEAR",

    # --------------------------------------------------------
    # DECISION FUSION
    # --------------------------------------------------------

    "final_class": "NORMAL",

    "final_class_id": 0,

    "policy_command": 0,

    "policy_name": "SYSTEM_HEALTHY",

    "action": "SYSTEM_HEALTHY",

    "relay_state": "ALL_ON",

    # --------------------------------------------------------
    # RELAY STATES
    #
    # true  = ON
    # false = OFF
    # --------------------------------------------------------

    "relays": {

        "r1": True,

        "r2": True,

        "r3": True,

        "r4": True

    },

    # --------------------------------------------------------
    # DERIVED SIGNALS
    # --------------------------------------------------------

    "dI_dt": 0.0,

    "dV_dt": 0.0,

    "variance": 0.0,

    "thermal_index": 0.0,

    "utilization": 0.0,

    # --------------------------------------------------------
    # 14 FEATURES
    # --------------------------------------------------------

    "features_14": [

        230.0,
        0.0,
        0.0,
        0.0,
        50.0,
        0.95,
        1.0,
        0.0,
        0.05,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0

    ],

    # --------------------------------------------------------
    # FEATURE NAMES
    # --------------------------------------------------------

    "feature_names": [

        "Voltage",

        "Current",

        "Power",

        "Energy",

        "Frequency",

        "Power Factor",

        "Load Priority",

        "Voltage Deviation",

        "PF Deviation",

        "Utilization",

        "dI/dt",

        "dV/dt",

        "Current Variance",

        "Thermal Index"

    ],

    # --------------------------------------------------------
    # SYSTEM METADATA
    # --------------------------------------------------------

    "models": {

        "primary": "14-Feature Neural Network",

        "secondary": "Edge Impulse Time-Series",

        "fusion": "Deterministic Safety Engine"

    },

    "hardware": {

        "platform": "Arduino UNO Q",

        "controller": "STM32 MCU",

        "processor": "Qualcomm Linux MPU"

    }

}


# ============================================================
# STATE LOCK
#
# Prevent simultaneous API / AI updates from corrupting
# dashboard state.
# ============================================================

state_lock = threading.Lock()


# ============================================================
# SYSTEM START TIME
# ============================================================

SYSTEM_START_TIME = time.time()


# ============================================================
# WEB API — GET /api/status
# ============================================================

def get_status():

    """
    Return complete live NeuralGrid state.
    """

    with state_lock:

        state = dict(latest_state)

        state["uptime"] = round(
            time.time() - SYSTEM_START_TIME,
            1
        )

        state["timestamp"] = time.time()

        return state


# ============================================================
# REGISTER WEB API
# ============================================================

web_ui.expose_api(
    "GET",
    "/api/status",
    get_status
)


print("[WebUI] GET /api/status registered.")


# ============================================================
# EMERGENCY STOP STATE
#
# GPIO 5 on the STM32 is the physical E-stop input. The MCU is the
# authority for the physical stop; this function mirrors that state
# into the Linux dashboard and forces the reported relays OFF.
# ============================================================

def set_emergency_stop_state(active):
    global latest_state

    with state_lock:
        latest_state["emergency_stop"] = bool(active)
        latest_state["emergency_stop_status"] = (
            "ACTIVE" if active else "CLEAR"
        )
        latest_state["emergency_stop_source"] = "UNO_Q_GPIO_5"

        if active:
            latest_state["status"] = "EMERGENCY_STOP"
            latest_state["health"] = 0
            latest_state["health_status"] = "CRITICAL"
            latest_state["policy_command"] = 5
            latest_state["policy_name"] = "HARDWARE_EMERGENCY_STOP"
            latest_state["action"] = "EMERGENCY_STOP_ACTIVE"
            latest_state["relay_state"] = "EMERGENCY_STOP"
            latest_state["relays"] = {
                "r1": False,
                "r2": False,
                "r3": False,
                "r4": False
            }
        else:
            # Releasing the button does not automatically turn loads back on.
            # The next valid telemetry packet must produce a fresh AI decision.
            latest_state["status"] = "ONLINE"
            latest_state["policy_command"] = 5
            latest_state["policy_name"] = "WAITING_FOR_AI"
            latest_state["action"] = "E_STOP_RELEASED_WAITING_FOR_AI"
            latest_state["relay_state"] = "ALL_OFF"
            latest_state["relays"] = {
                "r1": False,
                "r2": False,
                "r3": False,
                "r4": False
            }

        latest_state["timestamp"] = time.time()


def get_emergency_stop():
    with state_lock:
        return {
            "emergency_stop": latest_state["emergency_stop"],
            "status": latest_state["emergency_stop_status"],
            "source": latest_state["emergency_stop_source"],
            "timestamp": time.time()
        }


web_ui.expose_api(
    "GET",
    "/api/emergency-stop",
    get_emergency_stop
)

print("[WebUI] GET /api/emergency-stop registered.")


# ============================================================
# WEB API — GET /api/history
#
# Provides historical graph data.
# ============================================================

def get_history():

    with state_lock:

        count = min(

            len(timestamp_history),

            len(voltage_history),

            len(current_history),

            len(power_history),

            len(frequency_history)

        )

        timestamps = list(
            timestamp_history
        )[-count:]

        voltages = list(
            voltage_history
        )[-count:]

        currents = list(
            current_history
        )[-count:]

        powers = list(
            power_history
        )[-count:]

        frequencies = list(
            frequency_history
        )[-count:]

    return {

        "timestamps": timestamps,

        "voltage": voltages,

        "current": currents,

        "power": powers,

        "frequency": frequencies

    }


# ============================================================
# REGISTER HISTORY API
# ============================================================

web_ui.expose_api(
    "GET",
    "/api/history",
    get_history
)


print("[WebUI] GET /api/history registered.")


# ============================================================
# WEB API — GET /api/health
# ============================================================

def get_health():

    with state_lock:

        return {

            "status": latest_state[
                "status"
            ],

            "health": latest_state[
                "health"
            ],

            "health_status": latest_state[
                "health_status"
            ],
            "emergency_stop": latest_state[
                "emergency_stop"
            ],

            "timestamp": time.time()

        }


web_ui.expose_api(
    "GET",
    "/api/health",
    get_health
)


print("[WebUI] GET /api/health registered.")


# ============================================================
# LOAD PRIMARY AI
# ============================================================

print()
print("==================================================")
print(" LOADING PRIMARY AI")
print("==================================================")


if not os.path.exists(
    SCALER_PATH
):

    print(
        "[FATAL ERROR] scaler.pkl not found:"
    )

    print(
        SCALER_PATH
    )

    sys.exit(1)


if not os.path.exists(
    WEIGHTS_PATH
):

    print(
        "[FATAL ERROR] weights.pkl not found:"
    )

    print(
        WEIGHTS_PATH
    )

    sys.exit(1)


# ============================================================
# LOAD SCALER
# ============================================================

try:

    with open(
        SCALER_PATH,
        "rb"
    ) as f:

        scaler_data = pickle.load(f)


    scaler_mean = np.array(

        scaler_data["mean"],

        dtype=np.float32

    )


    scaler_scale = np.array(

        scaler_data["scale"],

        dtype=np.float32

    )


    print(
        "[SUCCESS] scaler.pkl loaded."
    )

    print(
        "[INFO] Scaler features:",
        len(scaler_mean)
    )

except Exception as e:

    print(
        "[FATAL ERROR] Scaler loading failed:"
    )

    print(
        str(e)
    )

    sys.exit(1)


# ============================================================
# LOAD NEURAL NETWORK WEIGHTS
# ============================================================

try:

    with open(
        WEIGHTS_PATH,
        "rb"
    ) as f:

        model_data = pickle.load(f)


    weights = [

        np.array(
            w,
            dtype=np.float32
        )

        for w
        in model_data["weights"]

    ]


    biases = [

        np.array(
            b,
            dtype=np.float32
        )

        for b
        in model_data["biases"]

    ]


    print(
        "[SUCCESS] weights.pkl loaded."
    )

    print(
        "[INFO] NN layers:",
        len(weights)
    )


except Exception as e:

    print(
        "[FATAL ERROR] Neural network loading failed:"
    )

    print(
        str(e)
    )

    sys.exit(1)


# ============================================================
# LOAD EDGE IMPULSE
# ============================================================

print()
print("==================================================")
print(" LOADING SECONDARY AI")
print("==================================================")


if not os.path.exists(
    EI_MODEL_PATH
):

    print(
        "[FATAL ERROR] model.eim not found:"
    )

    print(
        EI_MODEL_PATH
    )

    sys.exit(1)


try:

    os.system(
        f"chmod +x '{EI_MODEL_PATH}'"
    )


    ei_runner = ImpulseRunner(
        EI_MODEL_PATH
    )


    ei_model_info = (
        ei_runner.init()
    )


    EXPECTED_FEATURES = int(

        ei_model_info[
            "model_parameters"
        ][
            "input_features_count"
        ]

    )


    if EXPECTED_FEATURES % AXES_COUNT != 0:

        print(
            "[WARNING] EI feature count "
            "is not divisible by AXES_COUNT."
        )


    WINDOW_SAMPLES = max(

        1,

        EXPECTED_FEATURES //
        AXES_COUNT

    )


    ei_window_buffer = deque(

        maxlen=WINDOW_SAMPLES

    )


    print(
        "[SUCCESS] Edge Impulse model loaded."
    )


    print(
        "[INFO] EI Project:",
        ei_model_info[
            "project"
        ][
            "name"
        ]
    )


    print(
        "[INFO] EI Features:",
        EXPECTED_FEATURES
    )


    print(
        "[INFO] EI Timesteps:",
        WINDOW_SAMPLES
    )


except Exception as e:

    print(
        "[FATAL ERROR] Edge Impulse initialization failed:"
    )

    print(
        str(e)
    )

    sys.exit(1)


print()
print("==================================================")
print(" NEURALGRID AI STACK READY")
print("==================================================")


# ============================================================
# MATHEMATICAL FUNCTIONS
# ============================================================

def relu(x):

    return np.maximum(
        0.0,
        x
    )


# ============================================================

def softmax(x):

    x = x - np.max(x)

    exp_values = np.exp(x)

    total = np.sum(
        exp_values
    )

    if total <= 0:

        return (
            np.ones_like(x)
            /
            len(x)
        )

    return (
        exp_values /
        total
    )


# ============================================================
# PRIMARY NEURAL NETWORK
# ============================================================

def predict_primary_nn(
    features
):

    x = features

    for i in range(
        len(weights) - 1
    ):

        x = relu(

            np.dot(
                x,
                weights[i]
            )

            +

            biases[i]

        )


    logits = (

        np.dot(
            x,
            weights[-1]
        )

        +

        biases[-1]

    )


    probabilities = softmax(
        logits
    )[0]


    return probabilities


# ============================================================
# SCALING
# ============================================================

def scale_input(
    features
):

    safe_scale = np.where(

        scaler_scale == 0,

        1.0,

        scaler_scale

    )


    return (

        features -
        scaler_mean

    ) / safe_scale


# ============================================================
# FEATURE ALIGNMENT
# ============================================================

def align_features(
    features
):

    expected = len(
        scaler_mean
    )

    current = (
        features.shape[1]
    )


    if current < expected:

        padding = np.zeros(

            (
                1,
                expected - current
            ),

            dtype=np.float32

        )


        features = np.concatenate(

            [
                features,
                padding
            ],

            axis=1

        )


    elif current > expected:

        features = features[
            :,
            :expected
        ]


    return features


# ============================================================
# HISTORY UPDATE
# ============================================================

def update_history(

    voltage,
    current,
    pf,
    power,
    frequency

):

    # --------------------------------------------------------
    # Detect major load reset
    # --------------------------------------------------------

    if (

        len(current_history) > 0

        and current < 3.0

        and current_history[-1] >= 4.0

    ):

        voltage_history.clear()

        current_history.clear()

        power_history.clear()

        pf_history.clear()

        frequency_history.clear()

        thermal_history.clear()

        timestamp_history.clear()

        ei_window_buffer.clear()


    # --------------------------------------------------------
    # Append telemetry
    # --------------------------------------------------------

    voltage_history.append(
        voltage
    )

    current_history.append(
        current
    )

    power_history.append(
        power
    )

    pf_history.append(
        pf
    )

    frequency_history.append(
        frequency
    )

    thermal_history.append(
        current * current
    )

    timestamp_history.append(
        time.time()
    )


# ============================================================
# dI/dt
# ============================================================

def calculate_dI_dt():

    if len(
        current_history
    ) < 2:

        return 0.0


    return (

        current_history[-1]
        -
        current_history[-2]

    ) / SAMPLE_TIME


# ============================================================
# dV/dt
# ============================================================

def calculate_dV_dt():

    if len(
        voltage_history
    ) < 2:

        return 0.0


    return (

        voltage_history[-1]
        -
        voltage_history[-2]

    ) / SAMPLE_TIME


# ============================================================
# CURRENT VARIANCE
# ============================================================

def calculate_variance():

    if len(
        current_history
    ) < 5:

        return 0.0


    return float(

        np.var(
            current_history
        )

    )


# ============================================================
# THERMAL INDEX
# ============================================================

def calculate_thermal_index():

    if not thermal_history:

        return 0.0


    return float(

        np.sum(
            thermal_history
        )

        *

        SAMPLE_TIME

    )


# ============================================================
# HEALTH CALCULATION
# ============================================================

def calculate_health(

    voltage,
    current,
    pf

):

    score = (

        100.0

        -

        abs(
            voltage - 230.0
        ) * 2.0

        -

        max(
            current - 8.0,
            0
        ) * 6.0

        -

        max(
            0.90 - pf,
            0
        ) * 80.0

    )


    return int(

        np.clip(
            score,
            0,
            100
        )

    )


# ============================================================
# HEALTH LABEL
# ============================================================

def get_health_status(
    health
):

    if health >= 85:

        return "HEALTHY"

    if health >= 65:

        return "DEGRADED"

    if health >= 40:

        return "WARNING"

    return "CRITICAL"


# ============================================================
# SAFETY OVERRIDE
# ============================================================

def safety_override(

    voltage,
    current,
    pf,
    primary_class

):

    severity = primary_class


    # --------------------------------------------------------
    # CRITICAL
    # --------------------------------------------------------

    if (

        current >= 14.0

        or voltage < 205.0

        or pf < 0.70

    ):

        severity = 3


    # --------------------------------------------------------
    # OVERLOAD RISK
    # --------------------------------------------------------

    elif (

        current >= 10.0

        or voltage < 220.0

        or voltage > 245.0

        or pf < 0.85

    ):

        severity = max(
            severity,
            2
        )


    # --------------------------------------------------------
    # WARNING
    # --------------------------------------------------------

    elif current >= 6.0:

        severity = max(
            severity,
            1
        )


    return severity


# ============================================================
# DECISION FUSION
# ============================================================

def perform_decision_fusion(

    final_class,
    is_anomaly

):

    # --------------------------------------------------------
    # CRITICAL
    # --------------------------------------------------------

    if final_class == 3:

        return (
            4,
            "EMERGENCY_SHUTDOWN",
            "TRIP_PROTECTED"
        )


    # --------------------------------------------------------
    # OVERLOAD
    # --------------------------------------------------------

    if final_class == 2:

        return (
            3,
            "SHED_LOAD_HIGH",
            "TRIP_LOW_AUX"
        )


    # --------------------------------------------------------
    # ANOMALY
    # --------------------------------------------------------

    if is_anomaly:

        return (
            2,
            "PRE_FAULT_WARNING",
            "TRIP_AUX_ONLY"
        )


    # --------------------------------------------------------
    # WARNING
    # --------------------------------------------------------

    if final_class == 1:

        return (
            1,
            "MONITOR_LOAD",
            "ALL_ON"
        )


    # --------------------------------------------------------
    # NORMAL
    # --------------------------------------------------------

    return (
        0,
        "SYSTEM_HEALTHY",
        "ALL_ON"
    )


# ============================================================
# RELAY STATE CALCULATION
#
# Mirrors the policy implemented on STM32.
# ============================================================

def calculate_relay_states(
    cmd_policy
):

    if cmd_policy in (
        0,
        1
    ):

        return {

            "r1": True,
            "r2": True,
            "r3": True,
            "r4": True

        }


    if cmd_policy == 2:

        return {

            "r1": True,
            "r2": True,
            "r3": True,
            "r4": False

        }


    if cmd_policy == 3:

        return {

            "r1": True,
            "r2": True,
            "r3": False,
            "r4": False

        }


    if cmd_policy in (4, 5):

        return {

            "r1": False,
            "r2": False,
            "r3": False,
            "r4": False

        }


    return {

        "r1": False,
        "r2": False,
        "r3": False,
        "r4": False

    }


# ============================================================
# EDGE IMPULSE WINDOW BUILDER
# ============================================================

def build_ei_window(
    frame
):

    ei_window_buffer.append(
        frame
    )


    if len(
        ei_window_buffer
    ) < WINDOW_SAMPLES:

        missing = (

            WINDOW_SAMPLES
            -
            len(ei_window_buffer)

        )


        padding = (
            frame * missing
        )


        actual = [

            value

            for sample
            in ei_window_buffer

            for value
            in sample

        ]


        return (
            padding +
            actual
        )


    return [

        value

        for sample
        in ei_window_buffer

        for value
        in sample

    ]
    # ============================================================
# WEB DASHBOARD STATE UPDATE
# ============================================================

def update_web_state(

    voltage,
    current,
    power,
    pf,
    frequency,
    energy,

    dI_dt,
    dV_dt,
    variance,
    thermal,

    nn_features,

    primary_class,
    primary_confidence,
    nn_probabilities,

    raw_anomaly,
    is_anomaly,

    final_class,

    cmd_policy,
    fused_action,
    relay_command,

    health_score

):

    global latest_state

    relay_states = calculate_relay_states(
        cmd_policy
    )

    health_status = get_health_status(
        health_score
    )

    anomaly_status = (
        "ANOMALY DETECTED"
        if is_anomaly
        else "CLEAR"
    )

    # --------------------------------------------------------
    # Create a clean copy of probability values
    # --------------------------------------------------------

    probabilities = [
        round(float(x), 4)
        for x in nn_probabilities
    ]

    # --------------------------------------------------------
    # Make sure exactly four probabilities exist
    # --------------------------------------------------------

    while len(probabilities) < 4:

        probabilities.append(0.0)

    probabilities = probabilities[:4]

    # --------------------------------------------------------
    # Update state atomically
    # --------------------------------------------------------

    with state_lock:

        latest_state = {

            # =================================================
            # SYSTEM
            # =================================================

            "status": "ONLINE",

            "system": APP_NAME,

            "version": APP_VERSION,

            "timestamp": time.time(),

            "uptime": round(
                time.time() - SYSTEM_START_TIME,
                1
            ),

            # =================================================
            # TELEMETRY
            # =================================================

            "voltage": round(
                float(voltage),
                2
            ),

            "current": round(
                float(current),
                3
            ),

            "power": round(
                float(power),
                2
            ),

            "power_factor": round(
                float(pf),
                3
            ),

            "frequency": round(
                float(frequency),
                2
            ),

            "energy": round(
                float(energy),
                2
            ),

            # =================================================
            # HEALTH
            # =================================================

            "health": int(
                health_score
            ),

            "health_status": health_status,

            # =================================================
            # PRIMARY AI
            # =================================================

            "ai_class": CLASS_MAPPING.get(
                int(primary_class),
                "UNKNOWN"
            ),

            "ai_class_short": CLASS_SHORT.get(
                int(primary_class),
                "UNKNOWN"
            ),

            "ai_class_id": int(
                primary_class
            ),

            "ai_confidence": round(
                float(primary_confidence),
                4
            ),

            "ai_probabilities": probabilities,

            # =================================================
            # SECONDARY AI
            # =================================================

            "anomaly": bool(
                is_anomaly
            ),

            "anomaly_score": round(
                float(raw_anomaly),
                2
            ),

            "anomaly_threshold": (
                EI_ANOMALY_THRESHOLD
            ),

            "anomaly_status": anomaly_status,

            # =================================================
            # DECISION FUSION
            # =================================================

            "final_class": CLASS_MAPPING.get(
                int(final_class),
                "UNKNOWN"
            ),

            "final_class_short": CLASS_SHORT.get(
                int(final_class),
                "UNKNOWN"
            ),

            "final_class_id": int(
                final_class
            ),

            "policy_command": int(
                cmd_policy
            ),

            "policy_name": fused_action,

            "action": fused_action,

            "relay_state": relay_command,

            # =================================================
            # RELAYS
            # =================================================

            "relays": relay_states,

            # =================================================
            # DERIVED SIGNALS
            # =================================================

            "dI_dt": round(
                float(dI_dt),
                3
            ),

            "dV_dt": round(
                float(dV_dt),
                3
            ),

            "variance": round(
                float(variance),
                5
            ),

            "thermal_index": round(
                float(thermal),
                5
            ),

            "utilization": round(
                float(
                    np.clip(
                        (current / 15.0) * 100.0,
                        0,
                        100
                    )
                ),
                2
            ),

            # =================================================
            # 14 FEATURES
            # =================================================

            "features_14": [

                round(
                    float(x),
                    5
                )

                for x in nn_features

            ],

            "feature_names": [

                "Voltage",

                "Current",

                "Power",

                "Energy",

                "Frequency",

                "Power Factor",

                "Load Priority",

                "Voltage Deviation",

                "PF Deviation",

                "Utilization",

                "dI/dt",

                "dV/dt",

                "Current Variance",

                "Thermal Index"

            ],

            # =================================================
            # AI ARCHITECTURE
            # =================================================

            "models": {

                "primary":
                    "14-Feature Neural Network",

                "secondary":
                    "Edge Impulse Time-Series",

                "fusion":
                    "Deterministic Safety Engine"

            },

            # =================================================
            # HARDWARE
            # =================================================

            "hardware": {

                "platform":
                    "Arduino UNO Q",

                "controller":
                    "STM32 MCU",

                "processor":
                    "Qualcomm Linux MPU"

            }

        }




        # Never let an AI update clear an active hardware E-stop.
        if latest_state.get("emergency_stop", False):
            latest_state["status"] = "EMERGENCY_STOP"
            latest_state["health"] = 0
            latest_state["health_status"] = "CRITICAL"
            latest_state["policy_command"] = 5
            latest_state["policy_name"] = "HARDWARE_EMERGENCY_STOP"
            latest_state["action"] = "EMERGENCY_STOP_ACTIVE"
            latest_state["relay_state"] = "EMERGENCY_STOP"
            latest_state["relays"] = {
                "r1": False,
                "r2": False,
                "r3": False,
                "r4": False
            }
# ============================================================
# HISTORY API STATE
#
# The browser can use this to initialize charts immediately.
# ============================================================

def get_dashboard_history():

    with state_lock:

        return {

            "timestamps":
                list(timestamp_history),

            "voltage":
                list(voltage_history),

            "current":
                list(current_history),

            "power":
                list(power_history),

            "power_factor":
                list(pf_history),

            "frequency":
                list(frequency_history),

            "thermal":
                list(thermal_history)

        }


# ============================================================
# REGISTER DASHBOARD HISTORY API
# ============================================================

web_ui.expose_api(
    "GET",
    "/api/dashboard/history",
    get_dashboard_history
)

print(
    "[WebUI] GET /api/dashboard/history registered."
)


# ============================================================
# MODEL INFORMATION API
# ============================================================

def get_model_info():

    return {

        "system":
            SYSTEM_NAME,

        "version":
            APP_VERSION,

        "primary_ai":
            "14-Feature Deep Neural Network Classifier",

        "secondary_ai":
            "Edge Impulse Time-Series Anomaly Detector",

        "decision_layer":
            "Deterministic Safety Engine + Multi-Model Fusion",

        "primary_classes":
            CLASS_MAPPING,

        "ei_threshold":
            EI_ANOMALY_THRESHOLD,

        "ei_features":
            EXPECTED_FEATURES,

        "ei_timesteps":
            WINDOW_SAMPLES,

        "nn_features":
            len(scaler_mean),

        "hardware":
            {

                "platform":
                    "Arduino UNO Q",

                "mcu":
                    "STM32",

                "mpu":
                    "Qualcomm Linux"

            }

    }


web_ui.expose_api(
    "GET",
    "/api/models",
    get_model_info
)

print(
    "[WebUI] GET /api/models registered."
)


# ============================================================
# PRIMARY AI PREDICTION
# ============================================================

def run_primary_ai(
    nn_features
):

    # --------------------------------------------------------
    # Align feature dimensions
    # --------------------------------------------------------

    aligned_features = align_features(
        nn_features
    )

    # --------------------------------------------------------
    # Scale exactly as training
    # --------------------------------------------------------

    scaled_features = scale_input(
        aligned_features
    )

    # --------------------------------------------------------
    # Neural network forward pass
    # --------------------------------------------------------

    probabilities = predict_primary_nn(
        scaled_features
    )

    # --------------------------------------------------------
    # Class
    # --------------------------------------------------------

    predicted_class = int(
        np.argmax(
            probabilities
        )
    )

    # --------------------------------------------------------
    # Confidence
    # --------------------------------------------------------

    confidence = float(
        np.max(
            probabilities
        )
    )

    return (
        predicted_class,
        confidence,
        probabilities
    )


# ============================================================
# EDGE IMPULSE PREDICTION
# ============================================================

def run_edge_impulse(
    voltage,
    current,
    power,
    pf,
    frequency,
    dI_dt,
    dV_dt,
    variance
):

    # --------------------------------------------------------
    # 8-channel EI frame
    # --------------------------------------------------------

    frame = [

        float(voltage),

        float(current),

        float(power),

        float(pf),

        float(frequency),

        float(dI_dt),

        float(dV_dt),

        float(variance)

    ]

    # --------------------------------------------------------
    # Build rolling window
    # --------------------------------------------------------

    ei_full_window = build_ei_window(
        frame
    )

    # --------------------------------------------------------
    # Run Edge Impulse
    # --------------------------------------------------------

    result = ei_runner.classify(
        ei_full_window
    )

    result_data = result.get(
        "result",
        {}
    )

    # --------------------------------------------------------
    # Extract anomaly score
    # --------------------------------------------------------

    raw_anomaly = float(
        result_data.get(
            "anomaly",
            0.0
        )
    )

    # --------------------------------------------------------
    # Edge Impulse anomaly scores are commonly
    # represented on a 0-100 scale in this project.
    # --------------------------------------------------------

    is_anomaly = (
        raw_anomaly >=
        EI_ANOMALY_THRESHOLD
    )

    return (
        raw_anomaly,
        is_anomaly
    )


# ============================================================
# TELEMETRY PARSER
# ============================================================

def parse_telemetry(
    raw_input
):

    # --------------------------------------------------------
    # Defaults
    # --------------------------------------------------------

    voltage = 230.0

    current = 0.0

    pf = 0.95

    frequency = 50.0

    energy = 0.0

    # --------------------------------------------------------
    # Bytes
    # --------------------------------------------------------

    if isinstance(
        raw_input,
        (
            bytes,
            bytearray
        )
    ):

        raw_input = raw_input.decode(
            "utf-8"
        )

    # --------------------------------------------------------
    # String cleanup
    # --------------------------------------------------------

    if isinstance(
        raw_input,
        str
    ):

        raw_input = raw_input.strip()

    # ========================================================
    # JSON
    # ========================================================

    if (

        isinstance(
            raw_input,
            str
        )

        and raw_input.startswith(
            "{"
        )

    ):

        data = json.loads(
            raw_input
        )

        voltage = float(
            data.get(
                "voltage",
                230.0
            )
        )

        current = float(
            data.get(
                "current",
                0.0
            )
        )

        pf = float(
            data.get(
                "power_factor",
                data.get(
                    "pf",
                    0.95
                )
            )
        )

        frequency = float(
            data.get(
                "frequency",
                50.0
            )
        )

        energy = float(
            data.get(
                "energy",
                0.0
            )
        )

    # ========================================================
    # CSV
    #
    # V,I,PF,F
    #
    # or
    #
    # V,I,PF,F,Energy
    # ========================================================

    elif (

        isinstance(
            raw_input,
            str
        )

        and ","
        in raw_input

    ):

        parts = [

            x.strip()

            for x
            in raw_input.split(",")

        ]

        if len(parts) >= 1:

            voltage = float(
                parts[0]
            )

        if len(parts) >= 2:

            current = float(
                parts[1]
            )

        if len(parts) >= 3:

            pf = float(
                parts[2]
            )

        if len(parts) >= 4:

            frequency = float(
                parts[3]
            )

        if len(parts) >= 5:

            energy = float(
                parts[4]
            )

    else:

        raise ValueError(
            "Telemetry must be CSV or JSON"
        )

    # ========================================================
    # SANITY LIMITS
    # ========================================================

    voltage = float(
        np.clip(
            voltage,
            0,
            500
        )
    )

    current = float(
        np.clip(
            current,
            0,
            100
        )
    )

    pf = float(
        np.clip(
            pf,
            0,
            1
        )
    )

    frequency = float(
        np.clip(
            frequency,
            0,
            100
        )
    )

    energy = float(
        max(
            energy,
            0
        )
    )

    return (
        voltage,
        current,
        pf,
        frequency,
        energy
    )


# ============================================================
# MAIN AI RPC
#
# Called by STM32:
#
# Bridge.call(
#     "predict_json",
#     telemetry
# )
#
# ============================================================

def predict_json(
    *args
):

    try:

        # Hardware E-stop state is mirrored by the STM32. If it is active,
        # do not run AI inference and do not return a command that can enable loads.
        with state_lock:
            if latest_state.get("emergency_stop", False):
                return json.dumps({
                    "cmd": 5,
                    "ai": "EMERGENCY_STOP",
                    "conf": 1.0,
                    "anom": True,
                    "raw": 100.0,
                    "act": "EMERGENCY_STOP_ACTIVE",
                    "rly": "EMERGENCY_STOP",
                    "hlth": 0,
                    "estop": True
                }, separators=(",", ":"))

        # ====================================================
        # VALIDATION
        # ====================================================

        if not args:

            return json.dumps({

                "error":
                    "No telemetry payload"

            })


        # ====================================================
        # PARSE
        # ====================================================

        (
            voltage,
            current,
            pf,
            frequency,
            energy

        ) = parse_telemetry(
            args[0]
        )


        # ====================================================
        # POWER
        # ====================================================

        power = (

            voltage
            *
            current
            *
            pf

        )


        # ====================================================
        # LOAD PRIORITY
        # ========================================================

        if current < 6.0:

            priority = 1

        elif current < 12.0:

            priority = 2

        else:

            priority = 3


        # ====================================================
        # UPDATE HISTORY
        # ====================================================

        with state_lock:

            update_history(

                voltage,
                current,
                pf,
                power,
                frequency

            )


            # ================================================
            # DERIVED SIGNALS
            # ================================================

            dI_dt = calculate_dI_dt()

            dV_dt = calculate_dV_dt()

            variance = calculate_variance()

            thermal = calculate_thermal_index()


        # ====================================================
        # FEATURE ENGINEERING
        #
        # EXACT 14 FEATURE VECTOR
        # ====================================================

        voltage_deviation = abs(
            voltage - 230.0
        )

        pf_deviation = (
            1.0 - pf
        )

        utilization = (

            current /
            15.0

        ) * 100.0


        nn_features = np.array(

            [

                voltage,

                current,

                power,

                energy,

                frequency,

                pf,

                priority,

                voltage_deviation,

                pf_deviation,

                utilization,

                dI_dt,

                dV_dt,

                variance,

                thermal

            ],

            dtype=np.float32

        ).reshape(
            1,
            -1
        )


        # ====================================================
        # PRIMARY AI
        # ====================================================

        (
            primary_class,
            primary_confidence,
            nn_probabilities

        ) = run_primary_ai(
            nn_features
        )


        # ====================================================
        # SECONDARY AI
        # ====================================================

        (
            raw_anomaly,
            is_anomaly

        ) = run_edge_impulse(

            voltage,
            current,
            power,
            pf,
            frequency,
            dI_dt,
            dV_dt,
            variance

        )


        # ====================================================
        # SAFETY OVERRIDE
        # ====================================================

        final_class = safety_override(

            voltage,

            current,

            pf,

            primary_class

        )


        # ====================================================
        # DECISION FUSION
        # ====================================================

        (
            cmd_policy,
            fused_action,
            relay_command

        ) = perform_decision_fusion(

            final_class,

            is_anomaly

        )


        # ====================================================
        # HEALTH
        # ====================================================

        health_score = calculate_health(

            voltage,

            current,

            pf

        )


        # ====================================================
        # UPDATE WEB DASHBOARD
        # ====================================================

        update_web_state(

            voltage,

            current,

            power,

            pf,

            frequency,

            energy,

            dI_dt,

            dV_dt,

            variance,

            thermal,

            nn_features[0].tolist(),

            primary_class,

            primary_confidence,

            nn_probabilities,

            raw_anomaly,

            is_anomaly,

            final_class,

            cmd_policy,

            fused_action,

            relay_command,

            health_score

        )


        # ====================================================
        # MCU RESPONSE
        #
        # KEEP THIS COMPATIBLE WITH YOUR EXISTING
        # Arduino RouterBridge SKETCH.
        # ====================================================

        response = {

            "cmd":
                int(cmd_policy),

            "ai":
                CLASS_MAPPING[
                    primary_class
                ],

            "conf":
                round(
                    primary_confidence,
                    2
                ),

            "anom":
                bool(
                    is_anomaly
                ),

            "raw":
                round(
                    raw_anomaly,
                    1
                ),

            "act":
                fused_action,

            "rly":
                relay_command,

            "hlth":
                int(
                    health_score
                )

        }


        # ====================================================
        # CONSOLE LOG
        # ====================================================

        print(
            "[NEURALGRID]",
            json.dumps(
                response,
                separators=(
                    ",",
                    ":"
                )
            ),
            flush=True
        )


        # ====================================================
        # RETURN TO STM32
        # ====================================================

        return json.dumps(

            response,

            separators=(
                ",",
                ":"
            )

        )


    # ========================================================
    # ERROR HANDLING
    # ========================================================

    except Exception as e:

        error_message = str(
            e
        )

        print(
            "[AI ERROR]",
            error_message,
            flush=True
        )


        return json.dumps({

            "error":
                error_message

        })
        # ============================================================
# REGISTER MCU RPC
# ============================================================

print()
print("==================================================")
print(" REGISTERING NEURALGRID RPC")
print("==================================================")


Bridge.provide(
    "predict_json",
    predict_json
)


print(
    "[Bridge] predict_json RPC registered."
)


# ============================================================
# HARDWARE E-STOP RPC
#
# Called by the STM32 whenever GPIO 5 changes state.
# ============================================================

def set_emergency_stop(*args):
    if not args:
        active = False
    else:
        value = args[0]
        if isinstance(value, bool):
            active = value
        elif isinstance(value, (int, float)):
            active = value != 0
        else:
            active = str(value).strip().lower() in (
                "1", "true", "yes", "on", "active"
            )

    set_emergency_stop_state(active)

    response = {
        "ok": True,
        "emergency_stop": active,
        "cmd": 5 if active else 0,
        "action": (
            "EMERGENCY_STOP_ACTIVE"
            if active
            else "E_STOP_RELEASED_WAITING_FOR_AI"
        )
    }

    print(
        "[E-STOP]",
        "ACTIVE" if active else "RELEASED",
        flush=True
    )

    return json.dumps(
        response,
        separators=(",", ":")
    )


Bridge.provide(
    "set_emergency_stop",
    set_emergency_stop
)

print(
    "[Bridge] set_emergency_stop RPC registered."
)


# ============================================================
# OPTIONAL WEB SOCKET EVENT
#
# Allows the dashboard to request a fresh state.
# ============================================================

def on_get_initial_state(
    client,
    data
):

    try:

        state = get_status()

        web_ui.send_message(
            "neuralgrid_state",
            state,
            client
        )

    except Exception as e:

        print(
            "[WebUI] Initial state error:",
            str(e)
        )


try:

    web_ui.on_message(
        "get_initial_state",
        on_get_initial_state
    )

    print(
        "[WebUI] Socket initial-state handler registered."
    )

except Exception as e:

    print(
        "[WebUI] Socket handler unavailable:",
        str(e)
    )


# ============================================================
# TELEMETRY BROADCAST THREAD
#
# The browser normally polls /api/status, but this also
# provides a WebSocket-style update when supported.
# ============================================================

def dashboard_broadcast_loop():

    while True:

        try:

            state = get_status()

            try:

                web_ui.send_message(
                    "neuralgrid_state",
                    state
                )

            except Exception:
                pass

            time.sleep(
                0.25
            )

        except Exception:

            time.sleep(
                1.0
            )


# ============================================================
# START BROADCAST THREAD
# ============================================================

broadcast_thread = threading.Thread(

    target=dashboard_broadcast_loop,

    daemon=True

)

broadcast_thread.start()


print(
    "[WebUI] Live dashboard broadcast thread started."
)


# ============================================================
# STARTUP INFORMATION
# ============================================================

print()
print("==================================================")
print(" NEURALGRID EDGE AI SYSTEM")
print("==================================================")

print(
    "[SYSTEM] Version:",
    APP_VERSION
)

print(
    "[SYSTEM] Platform: Arduino UNO Q"
)

print(
    "[SYSTEM] MCU: STM32"
)

print(
    "[SYSTEM] MPU: Qualcomm Linux"
)

print()
print(
    "[AI] Primary:",
    "14-Feature Neural Network"
)

print(
    "[AI] Secondary:",
    "Edge Impulse Time-Series"
)

print(
    "[AI] Fusion:",
    "Deterministic Safety Engine"
)

print()
print(
    "[RPC] predict_json: READY"
)

print(
    "[RPC] set_emergency_stop: READY"
)

print(
    "[API] GET /api/status: READY"
)

print(
    "[API] GET /api/history: READY"
)

print(
    "[API] GET /api/dashboard/history: READY"
)

print(
    "[API] GET /api/models: READY"
)

print(
    "[API] GET /api/emergency-stop: READY"
)

print(
    "[API] GET /api/health: READY"
)

print()
print(
    "[WEB] Dashboard server starting..."
)

print("==================================================")


# ============================================================
# MAIN APPLICATION
# ============================================================

if __name__ == "__main__":

    try:

        # ----------------------------------------------------
        # App.run() is CRITICAL.
        #
        # It starts the Arduino UNO Q application runtime
        # and WebUI server.
        # ----------------------------------------------------

        App.run()


    except KeyboardInterrupt:

        print()
        print(
            "[NeuralGrid] Shutdown requested."
        )


    except Exception as e:

        print()
        print(
            "[FATAL]",
            str(e)
        )


    finally:

        # ----------------------------------------------------
        # Stop Edge Impulse runner cleanly.
        # ----------------------------------------------------

        try:

            ei_runner.stop()

        except Exception:

            pass


        print(
            "[NeuralGrid] AI services stopped."
        )

        print(
            "[NeuralGrid] System shutdown complete."
        )
        