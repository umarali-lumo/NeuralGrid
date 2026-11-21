/*
============================================================================
 NEURALGRID — SMART PREDICTIVE LOAD INTELLIGENCE SYSTEM 
 Arduino UNO Q — Real-Time Controller Firmware

 NeuralGrid measures electrical conditions, sends them to the local AI
 engine for analysis, and converts the resulting prediction into a
 physical load-control decision.

 The UNO Q real-time controller handles telemetry, safety checks, relay
 control, the local HMI, and industrial RS485 communication.

 PINOUT
 ──────────────────────────────────────────────────────────────────────────
 D2  → MAX485 DI       | RS485 TX
 D4  → MAX485 DE       | RS485 Driver Enable
 D5  → E-STOP          | NC Emergency Stop → GND
 D6  → Relay R4        | Auxiliary Load
 D7  → Relay R3        | Low-Priority Load
 D8  → Relay R2        | High-Priority Load
 D9  → Relay R1        | Critical Load
 D10 → PZEM #2 RX      | Critical Load Observation
 D11 → PZEM #2 TX      | Critical Load Observation
 D0/D1 → PZEM #1       | Primary Electrical Measurement (MAIN)

 RS485 operates as a TX-only industrial communication interface at
 1200 baud, 8N1. The current prototype transmits NeuralGrid's AI
 decision and system status to an external industrial controller.

 The firmware forms the real-time control layer of the complete
 Measure → Analyze → Predict → Decide → Act NeuralGrid architecture.

 Credit to Aurdino and Hackster.io 

 Made by Syeda Noor Fatima 
============================================================================
*/

#include <Arduino_RouterBridge.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h>

#define USB_BAUD 115200
#define DISPLAY_I2C_ADDR 0x42

// ============================================================================
// RS485 SETTINGS — pins, speed, and timing
// ============================================================================

#define RS485_TX_PIN   2
#define RS485_DE_PIN   4
#define RS485_BAUD     1200

const unsigned long RS485_BIT_TIME_US =
    1000000UL / RS485_BAUD;


// ============================================================================
// PZEM-004T v3.0 SETTINGS
// ============================================================================

#define PZEM_BAUD 9600
#define PZEM_ADDRESS 0xF8

// PZEM #1 uses the UNO Q hardware serial port: D0 receives data and D1 sends data.
#define PZEM1_SERIAL Serial1

// PZEM #2 uses GPIO 10 as RX and GPIO 11 as TX through software timing.
#define PZEM2_RX_PIN 10
#define PZEM2_TX_PIN 11

const unsigned long PZEM2_BIT_TIME_US =
    1000000UL / PZEM_BAUD;

struct PZEMData {
  float voltage = NAN;
  float current = NAN;
  float power = NAN;
  float energy = NAN;
  float frequency = NAN;
  float pf = NAN;
  bool valid = false;
};

PZEMData pzem1Data;
PZEMData pzem2Data;

unsigned long lastPZEMRead = 0;
const unsigned long PZEM_READ_INTERVAL = 1000UL;


// ============================================================================
// PZEM SERIAL DIAGNOSTIC PRINT INTERVAL
// ============================================================================
//
// Note:
//
// The sensors are still read every second. Only the Serial Monitor output is slowed down so debugging is easier.
// PZEM #1 is the primary sensor. Its values are sent to the AI.
// PZEM #2 currently watches the critical load only. It is kept separate so deeper safety functions can be added later.


unsigned long lastPZEMStatusPrint = 0;
const unsigned long PZEM_STATUS_PRINT_INTERVAL = 30000UL;

unsigned long lastPZEMErrorPrint = 0;
const unsigned long PZEM_ERROR_PRINT_INTERVAL = 30000UL;



// RELAY PINS

const int RELAY_CRITICAL      = 9;
const int RELAY_HIGH_PRIORITY = 8;
const int RELAY_LOW_PRIORITY  = 7;
const int RELAY_AUXILIARY     = 6;

const int RELAY_ON  = LOW;
const int RELAY_OFF = HIGH;

// The emergency stop is connected to a digital GPIO and uses the internal pull-up resistor.

#define ESTOP_PIN 5
#define ESTOP_ACTIVE HIGH

bool emergencyStopActive = false;
bool lastEmergencyStopState = false;

String serialBuffer = "";

float g_volts = 230.0f;
float g_amps  = 0.0f;
float g_pf    = 0.95f;
float g_freq  = 50.0f;

float previousVoltage = 230.0f;
float previousCurrent = 0.0f;

unsigned long previousFeatureTime = 0;
unsigned long previousThermalTime = 0;

float g_dI_dt        = 0.0f;
float g_dV_dt        = 0.0f;
float g_variance     = 0.0f;
float g_thermalIndex = 0.0f;

#define HISTORY_SIZE 10
float currentHistory[HISTORY_SIZE];
int historyCount = 0;
int historyIndex = 0;

float thermalAccumulation = 0.0f;
const float THERMAL_REFERENCE = 6000.0f;
const float THERMAL_TIME_CONSTANT = 120.0f;


// ============================================================================
// FUNCTION DECLARATIONS — telling the compiler which functions exist
// ============================================================================

void driveRelays(int r1, int r2, int r3, int r4);
void applyPolicyState(int cmdPolicy);
String getRelayStateStr(int cmdPolicy);
void parseInputTelemetry(const String &input);
bool validateTelemetry();
void processTelemetryPacket(const String &packet);
void updateDynamicFeatures();
void updateCurrentVariance();
void updateThermalAccumulation();
void resetDynamicFeatures();
void constructAndSendDisplayPayload(JsonDocument &aiDoc);
void printUnoQStatus(const JsonDocument &aiDoc);

// Emergency-stop functions
bool readEmergencyStop();
void updateEmergencyStopState();
void syncEmergencyStopToLinux();
void sendEmergencyStopStatus();


// ============================================================================
// PZEM FUNCTION DECLARATIONS
// ============================================================================

uint16_t pzemCRC16(const uint8_t *data, uint16_t len);
bool pzemReadFrame(Stream &port, uint8_t *buffer, size_t expectedLength);
bool readPZEM(Stream &port, PZEMData &data);

// PZEM #2 software UART functions
void pzem2SetTransmitMode();
void pzem2SetReceiveMode();
void pzem2WriteByte(uint8_t data);
bool pzem2ReadByte(uint8_t &data, unsigned long timeoutMs);
bool pzem2ReadFrame(uint8_t *buffer, size_t expectedLength);
bool readPZEM2CriticalLoad(PZEMData &data);

void updatePZEMSensors();
void printPZEMStatus();


// ============================================================================
// RS485 FUNCTION DECLARATIONS
// ============================================================================

void rs485SetTransmitMode();
void rs485SetIdleMode();
void rs485WriteByte(uint8_t data);
void rs485WriteString(const String &text);
void rs485SendNeuralGridPacket(const JsonDocument &aiDoc);


// ============================================================================
// PZEM-004T v3.0 COMMUNICATION
// ============================================================================

uint16_t pzemCRC16(const uint8_t *data, uint16_t len) {

  uint16_t crc = 0xFFFF;

  for (uint16_t pos = 0; pos < len; pos++) {

    crc ^= (uint16_t)data[pos];

    for (uint8_t i = 0; i < 8; i++) {

      if (crc & 0x0001) {

        crc >>= 1;
        crc ^= 0xA001;

      } else {

        crc >>= 1;
      }
    }
  }

  return crc;
}


bool pzemReadFrame(
    Stream &port,
    uint8_t *buffer,
    size_t expectedLength
) {

  const unsigned long timeout = 700UL;

  unsigned long start = millis();

  size_t index = 0;

  while (
      (millis() - start) < timeout &&
      index < expectedLength
  ) {

    while (
        port.available() &&
        index < expectedLength
    ) {

      int value = port.read();

      if (value >= 0) {
        buffer[index++] = (uint8_t)value;
      }
    }

    if (index >= expectedLength) {
      break;
    }

    delayMicroseconds(100);
  }

  return index == expectedLength;
}


bool readPZEM(
    Stream &port,
    PZEMData &data
) {

  uint8_t request[8] = {

    PZEM_ADDRESS,
    0x04,
    0x00,
    0x00,
    0x00,
    0x0A,
    0x00,
    0x00
  };

  uint16_t requestCRC =
      pzemCRC16(request, 6);

  request[6] =
      requestCRC & 0xFF;

  request[7] =
      (requestCRC >> 8) & 0xFF;


  while (port.available()) {
    port.read();
  }


  port.write(
      request,
      sizeof(request)
  );


  uint8_t response[25];


  if (!pzemReadFrame(
          port,
          response,
          sizeof(response)
      )) {

    data.valid = false;

    return false;
  }


  if (
      response[0] != PZEM_ADDRESS ||
      response[1] != 0x04 ||
      response[2] != 20
  ) {

    data.valid = false;

    return false;
  }


  uint16_t receivedCRC =
      (uint16_t)response[23] |
      ((uint16_t)response[24] << 8);


  uint16_t calculatedCRC =
      pzemCRC16(
          response,
          23
      );


  if (
      receivedCRC != calculatedCRC
  ) {

    data.valid = false;

    return false;
  }


  uint16_t voltageRaw =
      ((uint16_t)response[3] << 8) |
      response[4];


  uint32_t currentRaw =
      ((uint32_t)response[5] << 8) |
      response[6];

  currentRaw |=
      ((uint32_t)response[7] << 24) |
      ((uint32_t)response[8] << 16);


  uint32_t powerRaw =
      ((uint32_t)response[9] << 8) |
      response[10];

  powerRaw |=
      ((uint32_t)response[11] << 24) |
      ((uint32_t)response[12] << 16);


  uint32_t energyRaw =
      ((uint32_t)response[13] << 8) |
      response[14];

  energyRaw |=
      ((uint32_t)response[15] << 24) |
      ((uint32_t)response[16] << 16);


  uint16_t frequencyRaw =
      ((uint16_t)response[17] << 8) |
      response[18];


  uint16_t pfRaw =
      ((uint16_t)response[19] << 8) |
      response[20];


  data.voltage =
      (float)voltageRaw / 10.0f;

  data.current =
      (float)currentRaw / 1000.0f;

  data.power =
      (float)powerRaw / 10.0f;

  data.energy =
      (float)energyRaw / 1000.0f;

  data.frequency =
      (float)frequencyRaw / 10.0f;

  data.pf =
      (float)pfRaw / 100.0f;

  data.valid = true;

  return true;
}


// ============================================================================
// PZEM #2 — SOFTWARE UART
// ============================================================================

void pzem2SetTransmitMode() {

  pinMode(
      PZEM2_TX_PIN,
      OUTPUT
  );

  digitalWrite(
      PZEM2_TX_PIN,
      HIGH
  );
}


void pzem2SetReceiveMode() {

  pinMode(
      PZEM2_TX_PIN,
      OUTPUT
  );

  digitalWrite(
      PZEM2_TX_PIN,
      HIGH
  );

  pinMode(
      PZEM2_RX_PIN,
      INPUT_PULLUP
  );
}


void pzem2WriteByte(
    uint8_t data
) {

// Start bit: tells the receiver that a new byte is beginning.

  digitalWrite(
      PZEM2_TX_PIN,
      LOW
  );

  delayMicroseconds(
      PZEM2_BIT_TIME_US
  );


// Data bits: send the 8 bits of the byte, least-significant bit first.

  for (
      uint8_t i = 0;
      i < 8;
      i++
  ) {

    if (
        data & (1 << i)
    ) {

      digitalWrite(
          PZEM2_TX_PIN,
          HIGH
      );

    } else {

      digitalWrite(
          PZEM2_TX_PIN,
          LOW
      );
    }

    delayMicroseconds(
        PZEM2_BIT_TIME_US
    );
  }


// Stop bit: returns the line HIGH to finish the byte.

  digitalWrite(
      PZEM2_TX_PIN,
      HIGH
  );

  delayMicroseconds(
      PZEM2_BIT_TIME_US
  );
}


bool pzem2ReadByte(
    uint8_t &data,
    unsigned long timeoutMs
) {

  unsigned long startWait =
      millis();


  while (
      digitalRead(PZEM2_RX_PIN) == HIGH
  ) {

    if (
        (millis() - startWait) >= timeoutMs
    ) {

      return false;
    }
  }


  delayMicroseconds(
      PZEM2_BIT_TIME_US +
      (PZEM2_BIT_TIME_US / 2)
  );


  data = 0;


  for (
      uint8_t i = 0;
      i < 8;
      i++
  ) {

    if (
        digitalRead(PZEM2_RX_PIN)
    ) {

      data |=
          (uint8_t)(1 << i);
    }

    delayMicroseconds(
        PZEM2_BIT_TIME_US
    );
  }


  delayMicroseconds(
      PZEM2_BIT_TIME_US
  );


  return true;
}


bool pzem2ReadFrame(
    uint8_t *buffer,
    size_t expectedLength
) {

  for (
      size_t i = 0;
      i < expectedLength;
      i++
  ) {

    if (
        !pzem2ReadByte(
            buffer[i],
            100UL
        )
    ) {

      return false;
    }
  }

  return true;
}


bool readPZEM2CriticalLoad(
    PZEMData &data
) {

  uint8_t request[8] = {

    PZEM_ADDRESS,
    0x04,
    0x00,
    0x00,
    0x00,
    0x0A,
    0x00,
    0x00
  };


  uint16_t requestCRC =
      pzemCRC16(
          request,
          6
      );


  request[6] =
      requestCRC & 0xFF;

  request[7] =
      (requestCRC >> 8) & 0xFF;


  pzem2SetTransmitMode();


  for (
      uint8_t i = 0;
      i < sizeof(request);
      i++
  ) {

    pzem2WriteByte(
        request[i]
    );
  }


  pzem2SetReceiveMode();


  delayMicroseconds(
      PZEM2_BIT_TIME_US * 2
  );


  uint8_t response[25];


  if (
      !pzem2ReadFrame(
          response,
          sizeof(response)
      )
  ) {

    data.valid = false;

    return false;
  }


  if (
      response[0] != PZEM_ADDRESS ||
      response[1] != 0x04 ||
      response[2] != 20
  ) {

    data.valid = false;

    return false;
  }


  uint16_t receivedCRC =
      (uint16_t)response[23] |
      ((uint16_t)response[24] << 8);


  uint16_t calculatedCRC =
      pzemCRC16(
          response,
          23
      );


  if (
      receivedCRC != calculatedCRC
  ) {

    data.valid = false;

    return false;
  }


  uint16_t voltageRaw =
      ((uint16_t)response[3] << 8) |
      response[4];


  uint32_t currentRaw =
      ((uint32_t)response[5] << 8) |
      response[6];

  currentRaw |=
      ((uint32_t)response[7] << 24) |
      ((uint32_t)response[8] << 16);


  uint32_t powerRaw =
      ((uint32_t)response[9] << 8) |
      response[10];

  powerRaw |=
      ((uint32_t)response[11] << 24) |
      ((uint32_t)response[12] << 16);


  uint32_t energyRaw =
      ((uint32_t)response[13] << 8) |
      response[14];

  energyRaw |=
      ((uint32_t)response[15] << 24) |
      ((uint32_t)response[16] << 16);


  uint16_t frequencyRaw =
      ((uint16_t)response[17] << 8) |
      response[18];


  uint16_t pfRaw =
      ((uint16_t)response[19] << 8) |
      response[20];


  data.voltage =
      (float)voltageRaw / 10.0f;

  data.current =
      (float)currentRaw / 1000.0f;

  data.power =
      (float)powerRaw / 10.0f;

  data.energy =
      (float)energyRaw / 1000.0f;

  data.frequency =
      (float)frequencyRaw / 10.0f;

  data.pf =
      (float)pfRaw / 100.0f;

  data.valid = true;

  return true;
}


// ============================================================================
// PZEM SENSOR UPDATE
// ============================================================================

void updatePZEMSensors() {

  unsigned long now =
      millis();


  if (
      (now - lastPZEMRead) <
      PZEM_READ_INTERVAL
  ) {

    return;
  }


  lastPZEMRead =
      now;


// Read PZEM #1 through the UNO Q hardware serial port.

  readPZEM(
      PZEM1_SERIAL,
      pzem1Data
  );


// Read PZEM #2 through the GPIO-based software UART.

  readPZEM2CriticalLoad(
      pzem2Data
  );
}


// ============================================================================
// PZEM STATUS PRINT
// ============================================================================
//
// Important:
//
// The main loop checks this function every second,
// but the function itself only prints once every 30 seconds.
//
// So the timing is:
//
// SENSOR READ     = every 1 second
// SERIAL DISPLAY  = every 30 seconds
//
// ============================================================================

void printPZEMStatus() {

  unsigned long now =
      millis();


  if (
      (now - lastPZEMStatusPrint) <
      PZEM_STATUS_PRINT_INTERVAL
  ) {

    return;
  }


  lastPZEMStatusPrint =
      now;


  Serial.println(
      F("[PZEM] ------------------------------------------------")
  );


// ==========================================================
// PZEM #1 — primary AI sensor
// ==========================================================

  Serial.print(
      F("[PZEM 1 / PRIMARY AI] ")
  );


  if (
      pzem1Data.valid
  ) {

    Serial.print(F("V="));
    Serial.print(
        pzem1Data.voltage,
        1
    );

    Serial.print(F("V  I="));
    Serial.print(
        pzem1Data.current,
        3
    );

    Serial.print(F("A  P="));
    Serial.print(
        pzem1Data.power,
        1
    );

    Serial.print(F("W  PF="));
    Serial.print(
        pzem1Data.pf,
        2
    );

    Serial.print(F("  F="));
    Serial.print(
        pzem1Data.frequency,
        1
    );

    Serial.print(F("Hz  E="));
    Serial.print(
        pzem1Data.energy,
        3
    );

    Serial.println(F("kWh"));

  } else {

    Serial.println(
        F("READ ERROR")
    );
  }


// ==========================================================
// PZEM #2 — critical-load observation sensor
// ==========================================================

  Serial.print(
      F("[PZEM 2 / CRITICAL LOAD] ")
  );


  if (
      pzem2Data.valid
  ) {

    Serial.print(F("V="));
    Serial.print(
        pzem2Data.voltage,
        1
    );

    Serial.print(F("V  I="));
    Serial.print(
        pzem2Data.current,
        3
    );

    Serial.print(F("A  P="));
    Serial.print(
        pzem2Data.power,
        1
    );

    Serial.print(F("W  PF="));
    Serial.print(
        pzem2Data.pf,
        2
    );

    Serial.print(F("  F="));
    Serial.print(
        pzem2Data.frequency,
        1
    );

    Serial.print(F("Hz  E="));
    Serial.print(
        pzem2Data.energy,
        3
    );

    Serial.println(F("kWh"));

  } else {

    Serial.println(
        F("READ ERROR")
    );
  }
}


// ============================================================================
// Apply the AI policy to the four relays, unless the emergency stop is active.
// ============================================================================

void driveRelays(
    int r1,
    int r2,
    int r3,
    int r4
) {

  digitalWrite(
      RELAY_CRITICAL,
      r1
  );

  digitalWrite(
      RELAY_HIGH_PRIORITY,
      r2
  );

  digitalWrite(
      RELAY_LOW_PRIORITY,
      r3
  );

  digitalWrite(
      RELAY_AUXILIARY,
      r4
  );
}


void applyPolicyState(
    int cmdPolicy
) {

  switch (
      cmdPolicy
  ) {

    case 0:
    case 1:

      driveRelays(
          RELAY_ON,
          RELAY_ON,
          RELAY_ON,
          RELAY_ON
      );

      break;


    case 2:

      driveRelays(
          RELAY_ON,
          RELAY_ON,
          RELAY_ON,
          RELAY_OFF
      );

      break;


    case 3:

      driveRelays(
          RELAY_ON,
          RELAY_ON,
          RELAY_OFF,
          RELAY_OFF
      );

      break;


    case 4:
    case 5:

      driveRelays(
          RELAY_OFF,
          RELAY_OFF,
          RELAY_OFF,
          RELAY_OFF
      );

      break;


    default:

      driveRelays(
          RELAY_OFF,
          RELAY_OFF,
          RELAY_OFF,
          RELAY_OFF
      );

      break;
  }
}


String getRelayStateStr(
    int cmdPolicy
) {

  switch (
      cmdPolicy
  ) {

    case 0:
    case 1:

      return "ALL_ON";


    case 2:

      return "TRIP_AUX_ONLY";


    case 3:

      return "TRIP_LOW_AUX";


    case 4:

      return "ALL_TRIPPED";


    case 5:

      return "EMERGENCY_STOP";


    default:

      return "ALL_TRIPPED";
  }
}


// ============================================================================
// EMERGENCY STOP — safety check happens before manual Serial processing
// ============================================================================

bool readEmergencyStop() {

  return digitalRead(
      ESTOP_PIN
  ) == ESTOP_ACTIVE;
}


void syncEmergencyStopToLinux() {

  RpcCall call =
      Bridge.call(
          "set_emergency_stop",
          emergencyStopActive
              ? "1"
              : "0"
      );


  String result;

  call.result(
      result
  );
}


void sendEmergencyStopStatus() {

  StaticJsonDocument<256>
      estopDoc;


  estopDoc["cmd"] =
      5;

  estopDoc["hlth"] =
      0;

  estopDoc["ai"] =
      "EMERGENCY_STOP";

  estopDoc["conf"] =
      1.0f;

  estopDoc["anom"] =
      true;

  estopDoc["raw"] =
      100.0f;

  estopDoc["act"] =
      "EMERGENCY_STOP_ACTIVE";

  estopDoc["rly"] =
      "EMERGENCY_STOP";

  estopDoc["estop"] =
      true;


  driveRelays(
      RELAY_OFF,
      RELAY_OFF,
      RELAY_OFF,
      RELAY_OFF
  );


  printUnoQStatus(
      estopDoc
  );


  constructAndSendDisplayPayload(
      estopDoc
  );


  rs485SendNeuralGridPacket(
      estopDoc
  );
}


void updateEmergencyStopState() {

  bool currentState =
      readEmergencyStop();


  if (
      currentState ==
      emergencyStopActive
  ) {

    if (
        emergencyStopActive
    ) {

      driveRelays(
          RELAY_OFF,
          RELAY_OFF,
          RELAY_OFF,
          RELAY_OFF
      );
    }

    return;
  }


  emergencyStopActive =
      currentState;


  resetDynamicFeatures();


  if (
      emergencyStopActive
  ) {

    driveRelays(
        RELAY_OFF,
        RELAY_OFF,
        RELAY_OFF,
        RELAY_OFF
    );


    Serial.println(
        F(
          "[E-STOP] EMERGENCY STOP ACTIVE — ALL RELAYS OFF"
        )
    );


    syncEmergencyStopToLinux();


    sendEmergencyStopStatus();

  } else {

    Serial.println(
        F(
          "[E-STOP] Released — waiting for the next AI decision."
        )
    );


    syncEmergencyStopToLinux();
  }


  lastEmergencyStopState =
      emergencyStopActive;
}


// ============================================================================
// TELEMETRY PARSER — converts text like 230,2.5,0.95,50 into usable numbers
// ============================================================================

void parseInputTelemetry(
    const String &input
) {

  int idx1 =
      input.indexOf(',');

  int idx2 =
      input.indexOf(
          ',',
          idx1 + 1
      );

  int idx3 =
      input.indexOf(
          ',',
          idx2 + 1
      );


  if (
      idx1 != -1 &&
      idx2 != -1 &&
      idx3 != -1
  ) {

    g_volts =
        input.substring(
            0,
            idx1
        ).toFloat();


    g_amps =
        input.substring(
            idx1 + 1,
            idx2
        ).toFloat();


    g_pf =
        input.substring(
            idx2 + 1,
            idx3
        ).toFloat();


    g_freq =
        input.substring(
            idx3 + 1
        ).toFloat();
  }
}


bool validateTelemetry() {

  if (
      !isfinite(g_volts) ||
      g_volts < 0.0f ||
      g_volts > 500.0f
  ) {

    return false;
  }


  if (
      !isfinite(g_amps) ||
      g_amps < 0.0f ||
      g_amps > 100.0f
  ) {

    return false;
  }


  if (
      !isfinite(g_pf) ||
      g_pf < 0.0f ||
      g_pf > 1.0f
  ) {

    return false;
  }


  if (
      !isfinite(g_freq) ||
      g_freq < 0.0f ||
      g_freq > 100.0f
  ) {

    return false;
  }


  return true;
}


// ============================================================================
// 3. DYNAMIC FEATURES — calculate rate of change, variance, and thermal index
// ============================================================================

void updateDynamicFeatures() {

  unsigned long now =
      millis();


  if (
      previousFeatureTime == 0
  ) {

    previousVoltage =
        g_volts;

    previousCurrent =
        g_amps;

    previousFeatureTime =
        now;

    previousThermalTime =
        now;


    g_dI_dt =
        0.0f;

    g_dV_dt =
        0.0f;


    updateCurrentVariance();

    return;
  }


  float dt =
      (float)(
          now -
          previousFeatureTime
      ) / 1000.0f;


  if (
      dt < 0.001f
  ) {

    dt = 0.001f;
  }


  if (
      dt > 10.0f
  ) {

    dt = 10.0f;
  }


  g_dI_dt =
      constrain(
          (g_amps -
           previousCurrent) /
          dt,

          -999.0f,
          999.0f
      );


  g_dV_dt =
      constrain(
          (g_volts -
           previousVoltage) /
          dt,

          -999.0f,
          999.0f
      );


  currentHistory[
      historyIndex
  ] = g_amps;


  historyIndex =
      (historyIndex + 1) %
      HISTORY_SIZE;


  if (
      historyCount <
      HISTORY_SIZE
  ) {

    historyCount++;
  }


  updateCurrentVariance();

  updateThermalAccumulation();


  previousVoltage =
      g_volts;

  previousCurrent =
      g_amps;

  previousFeatureTime =
      now;
}


void updateCurrentVariance() {

  if (
      historyCount <= 1
  ) {

    g_variance =
        0.0f;

    return;
  }


  float sum =
      0.0f;


  for (
      int i = 0;
      i < historyCount;
      i++
  ) {

    sum +=
        currentHistory[i];
  }


  float mean =
      sum /
      (float)historyCount;


  float varianceSum =
      0.0f;


  for (
      int i = 0;
      i < historyCount;
      i++
  ) {

    float diff =
        currentHistory[i] -
        mean;


    varianceSum +=
        diff * diff;
  }


  g_variance =
      varianceSum /
      (float)historyCount;


  if (
      !isfinite(g_variance)
  ) {

    g_variance =
        0.0f;
  }
}


void updateThermalAccumulation() {

  unsigned long now =
      millis();


  if (
      previousThermalTime == 0
  ) {

    previousThermalTime =
        now;

    return;
  }


  float dt =
      (float)(
          now -
          previousThermalTime
      ) / 1000.0f;


  previousThermalTime =
      now;


  if (
      dt < 0.0f
  ) {

    dt = 0.0f;
  }


  if (
      dt > 10.0f
  ) {

    dt = 10.0f;
  }


  thermalAccumulation *=
      exp(
          -dt /
          THERMAL_TIME_CONSTANT
      );


  thermalAccumulation +=
      (g_amps * g_amps) *
      dt;


  if (
      !isfinite(
          thermalAccumulation
      )
  ) {

    thermalAccumulation =
        0.0f;
  }


  g_thermalIndex =
      constrain(
          (
              thermalAccumulation /
              THERMAL_REFERENCE
          ) * 100.0f,

          0.0f,
          100.0f
      );
}


void resetDynamicFeatures() {

  g_dI_dt =
      0.0f;

  g_dV_dt =
      0.0f;

  g_variance =
      0.0f;

  g_thermalIndex =
      0.0f;


  thermalAccumulation =
      0.0f;


  historyCount =
      0;

  historyIndex =
      0;


  previousFeatureTime =
      0;

  previousThermalTime =
      0;


  previousVoltage =
      g_volts;

  previousCurrent =
      g_amps;


  for (
      int i = 0;
      i < HISTORY_SIZE;
      i++
  ) {

    currentHistory[i] =
        0.0f;
  }
}


// ============================================================================
// UNO Q DIAGNOSTICS — prints a readable summary to the Serial Monitor
// ============================================================================

void printUnoQStatus(
    const JsonDocument &aiDoc
) {

  int cmd =
      aiDoc["cmd"] | 0;


  String aiClass =
      aiDoc["ai"] |
      String("NORMAL");


  String action =
      aiDoc["act"] |
      String("SYSTEM_HEALTHY");


  int r1State =
      digitalRead(
          RELAY_CRITICAL
      );


  int r2State =
      digitalRead(
          RELAY_HIGH_PRIORITY
      );


  int r3State =
      digitalRead(
          RELAY_LOW_PRIORITY
      );


  int r4State =
      digitalRead(
          RELAY_AUXILIARY
      );


  Serial.println(
      F(
        "\n----------------- UNO Q SYSTEM DIAGNOSTICS -----------------"
      )
  );


  Serial.print(
      F("TELEMETRY  : ")
  );

  Serial.print(
      g_volts,
      1
  );

  Serial.print(
      F("V | ")
  );

  Serial.print(
      g_amps,
      2
  );

  Serial.print(
      F("A | PF: ")
  );

  Serial.print(
      g_pf,
      2
  );

  Serial.print(
      F(" | Freq: ")
  );

  Serial.print(
      g_freq,
      1
  );

  Serial.println(
      F("Hz")
  );


  Serial.print(
      F("AI DECISION: ")
  );

  Serial.print(
      aiClass
  );

  Serial.print(
      F(" | Policy Cmd: ")
  );

  Serial.print(
      cmd
  );

  Serial.print(
      F(" | Action: ")
  );

  Serial.println(
      action
  );


  Serial.print(
      F("RELAY MODE : ")
  );

  Serial.println(
      getRelayStateStr(
          cmd
      )
  );


  Serial.print(
      F("PIN STATES : R1(Crit):")
  );

  Serial.print(
      r1State == RELAY_ON
          ? "ON "
          : "OFF"
  );


  Serial.print(
      F(" | R2(High):")
  );

  Serial.print(
      r2State == RELAY_ON
          ? "ON "
          : "OFF"
  );


  Serial.print(
      F(" | R3(Low):")
  );

  Serial.print(
      r3State == RELAY_ON
          ? "ON "
          : "OFF"
  );


  Serial.print(
      F(" | R4(Aux):")
  );

  Serial.println(
      r4State == RELAY_ON
          ? "ON "
          : "OFF"
  );


  Serial.println(
      F(
        "-----------------------------------------------------------\n"
      )
  );
}


// ============================================================================
// I2C DISPLAY PAYLOAD — packages the latest system information as JSON for the display
// ============================================================================

void constructAndSendDisplayPayload(
    JsonDocument &aiDoc
) {

  StaticJsonDocument<1536>
      displayDoc;


  displayDoc["voltage"] =
      g_volts;


  displayDoc["current"] =
      g_amps;


  displayDoc["power"] =
      g_volts *
      g_amps *
      g_pf;


  displayDoc["power_factor"] =
      g_pf;


  displayDoc["frequency"] =
      g_freq;


  displayDoc["energy"] =
      1.25f;


  int cmd =
      aiDoc["cmd"] | 0;


  int health =
      aiDoc["hlth"] | 100;


  String aiClass =
      aiDoc["ai"] |
      String("NORMAL");


  float confidence =
      aiDoc["conf"] |
      0.0f;


  bool anomaly =
      aiDoc["anom"] |
      false;


  float raw =
      aiDoc["raw"] |
      0.0f;


  String action =
      aiDoc["act"] |
      String("SYSTEM_HEALTHY");


  displayDoc["health"] =
      health;


  displayDoc["ai_class"] =
      aiClass;


  displayDoc["ai_confidence"] =
      confidence;


  displayDoc["anomaly"] =
      anomaly;


  displayDoc["anomaly_score"] =
      raw;


  displayDoc["final_class"] =
      aiClass;


  displayDoc["policy_command"] =
      cmd;


  displayDoc["action"] =
      action;


  displayDoc["emergency_stop"] =
      aiDoc["estop"] |
      emergencyStopActive;


  displayDoc["relay_state"] =
      getRelayStateStr(
          cmd
      );


  displayDoc["dI_dt"] =
      g_dI_dt;


  displayDoc["dV_dt"] =
      g_dV_dt;


  displayDoc["variance"] =
      g_variance;


  displayDoc["thermal_index"] =
      g_thermalIndex;


  String displayJson;


  serializeJson(
      displayDoc,
      displayJson
  );


  displayJson +=
      "\n";


  Wire.beginTransmission(
      DISPLAY_I2C_ADDR
  );


  for (
      size_t i = 0;
      i < displayJson.length();
      i++
  ) {

    Wire.write(
        displayJson[i]
    );


    if (
        (i + 1) % 30 == 0
    ) {

      Wire.endTransmission(false);


      Wire.beginTransmission(
          DISPLAY_I2C_ADDR
      );
    }
  }


  Wire.endTransmission();


  Serial.print(
      F("[DISPLAY I2C TX] ")
  );


  Serial.println(
      displayJson
  );
}


// ============================================================================
// RS485 — SOFTWARE UART
// ============================================================================

void rs485SetTransmitMode() {

  digitalWrite(
      RS485_DE_PIN,
      HIGH
  );
}


void rs485SetIdleMode() {

  digitalWrite(
      RS485_DE_PIN,
      LOW
  );
}


// ============================================================================
// RS485 — send one byte using software timing
// ============================================================================

void rs485WriteByte(
    uint8_t data
) {

  digitalWrite(
      RS485_TX_PIN,
      LOW
  );


  delayMicroseconds(
      RS485_BIT_TIME_US
  );


  for (
      uint8_t i = 0;
      i < 8;
      i++
  ) {

    if (
        data & (1 << i)
    ) {

      digitalWrite(
          RS485_TX_PIN,
          HIGH
      );

    } else {

      digitalWrite(
          RS485_TX_PIN,
          LOW
      );
    }


    delayMicroseconds(
        RS485_BIT_TIME_US
    );
  }


  digitalWrite(
      RS485_TX_PIN,
      HIGH
  );


  delayMicroseconds(
      RS485_BIT_TIME_US
  );
}


// ============================================================================
// RS485 — send a complete text string one byte at a time
// ============================================================================

void rs485WriteString(
    const String &text
) {

  for (
      size_t i = 0;
      i < text.length();
      i++
  ) {

    rs485WriteByte(
        (uint8_t)text[i]
    );
  }
}


// ============================================================================
// RS485 — build and send the NeuralGrid status packet
// ============================================================================

void rs485SendNeuralGridPacket(
    const JsonDocument &aiDoc
) {

  StaticJsonDocument<768>
      rs485Doc;


  rs485Doc["system"] =
      "NEURALGRID";


  rs485Doc["type"] =
      "AI_STATUS";


  rs485Doc["voltage"] =
      g_volts;


  rs485Doc["current"] =
      g_amps;


  rs485Doc["power"] =
      g_volts *
      g_amps *
      g_pf;


  rs485Doc["pf"] =
      g_pf;


  rs485Doc["freq"] =
      g_freq;


  rs485Doc["dI_dt"] =
      g_dI_dt;


  rs485Doc["dV_dt"] =
      g_dV_dt;


  rs485Doc["variance"] =
      g_variance;


  rs485Doc["thermal"] =
      g_thermalIndex;


  rs485Doc["cmd"] =
      aiDoc["cmd"] | 0;


  rs485Doc["ai"] =
      aiDoc["ai"] |
      String("NORMAL");


  rs485Doc["conf"] =
      aiDoc["conf"] |
      0.0f;


  rs485Doc["anom"] =
      aiDoc["anom"] |
      false;


  rs485Doc["raw"] =
      aiDoc["raw"] |
      0.0f;


  rs485Doc["act"] =
      aiDoc["act"] |
      String("SYSTEM_HEALTHY");


  rs485Doc["health"] =
      aiDoc["hlth"] |
      100;


  rs485Doc["emergency_stop"] =
      aiDoc["estop"] |
      emergencyStopActive;


  rs485Doc["relay"] =
      getRelayStateStr(
          aiDoc["cmd"] | 0
      );


  String packet;


  serializeJson(
      rs485Doc,
      packet
  );


  packet +=
      "\r\n";


  rs485SetTransmitMode();


  delayMicroseconds(
      RS485_BIT_TIME_US
  );


  rs485WriteString(
      packet
  );


  delayMicroseconds(
      RS485_BIT_TIME_US
  );


  rs485SetIdleMode();


  Serial.print(
      F("[RS485 TX] ")
  );


  Serial.println(
      packet
  );
}


// ============================================================================
// SETUP — runs once when the UNO Q starts
// ============================================================================

void setup() {

  Serial.begin(
      USB_BAUD
  );


// ==========================================================
// I2C SETUP
// ==========================================================

  pinMode(
      SDA,
      INPUT_PULLUP
  );


  pinMode(
      SCL,
      INPUT_PULLUP
  );


  Wire.begin();


  Wire.setClock(
      400000
  );


// ==========================================================
// RELAY SETUP
// ==========================================================

  pinMode(
      RELAY_CRITICAL,
      OUTPUT
  );


  pinMode(
      RELAY_HIGH_PRIORITY,
      OUTPUT
  );


  pinMode(
      RELAY_LOW_PRIORITY,
      OUTPUT
  );


  pinMode(
      RELAY_AUXILIARY,
      OUTPUT
  );


// ==========================================================
// EMERGENCY STOP — safety check happens before manual Serial processing
// ==========================================================

  pinMode(
      ESTOP_PIN,
      INPUT_PULLUP
  );


  driveRelays(
      RELAY_OFF,
      RELAY_OFF,
      RELAY_OFF,
      RELAY_OFF
  );


  emergencyStopActive =
      readEmergencyStop();


  lastEmergencyStopState =
      emergencyStopActive;


// ==========================================================
// 3. DYNAMIC FEATURES — calculate rate of change, variance, and thermal index
// ==========================================================

  resetDynamicFeatures();


// ==========================================================
// RS485 SOFTWARE UART SETUP
// ==========================================================

  pinMode(
      RS485_TX_PIN,
      OUTPUT
  );


  pinMode(
      RS485_DE_PIN,
      OUTPUT
  );


  digitalWrite(
      RS485_TX_PIN,
      HIGH
  );


  rs485SetIdleMode();


// ==========================================================
// PZEM SERIAL PORT SETUP
// ==========================================================

  PZEM1_SERIAL.begin(
      PZEM_BAUD
  );


  pzem2SetReceiveMode();


  lastPZEMRead =
      millis() -
      PZEM_READ_INTERVAL;


  Serial.println(
      F(
        "[PZEM 1] Hardware UART: D0=RX, D1=TX, 9600 8N1"
      )
  );


  Serial.println(
      F(
        "[PZEM 2] Critical Load: GPIO10=RX, GPIO11=TX, direct GPIO UART, 9600 8N1"
      )
  );


  Serial.println(
      F(
        "[PZEM] Diagnostic messages limited to once every 30 seconds."
      )
  );


// ==========================================================
// ROUTER BRIDGE — connects the MCU side to the Linux/AI side of the UNO Q
// ==========================================================

  Bridge.begin();


  syncEmergencyStopToLinux();


  if (
      emergencyStopActive
  ) {

    Serial.println(
        F(
          "[E-STOP] Startup blocked — emergency stop is active."
        )
    );


    sendEmergencyStopStatus();

  } else {

    driveRelays(
        RELAY_ON,
        RELAY_ON,
        RELAY_ON,
        RELAY_ON
    );
  }


// ==========================================================
// SERIAL MONITOR STARTUP
// ==========================================================

  while (
      !Serial &&
      millis() < 3000
  ) {

    delay(10);
  }


  Serial.println(
      F(
        "\nNEURALGRID — UNO Q SYSTEM CONTROLLER (I2C HARDWARE READY)"
      )
  );


  Serial.println(
      F(
        "[RS485] Direct GPIO software UART initialized."
      )
  );


  Serial.print(
      F("[RS485] TX GPIO: ")
  );


  Serial.println(
      RS485_TX_PIN
  );


  Serial.print(
      F("[RS485] DE GPIO: ")
  );


  Serial.println(
      RS485_DE_PIN
  );


  Serial.println(
      F(
        "[RS485] /RE: EXTERNAL 5V (RX DISABLED)"
      )
  );


  Serial.print(
      F("[RS485] Baud: ")
  );


  Serial.println(
      RS485_BAUD
  );


  Serial.println(
      F("[RS485] Format: 8N1")
  );


  Serial.println(
      F(
        "[RS485] Mode: SOFTWARE UART TRANSMITTER"
      )
  );
}


// ============================================================================
// TELEMETRY → AI PROCESSING — this is the main path from sensor data to the AI
// ============================================================================

void processTelemetryPacket(
    const String &packet
) {

  if (
      emergencyStopActive
  ) {

    return;
  }


  String telemetry =
      packet;


  telemetry.trim();


  if (
      telemetry.length() == 0
  ) {

    return;
  }


// ==========================================================
// 1. PARSE — split the incoming telemetry into voltage, current, PF, and frequency
// ==========================================================

  parseInputTelemetry(
      telemetry
  );


// ==========================================================
// 2. VALIDATE — reject values that are clearly outside the allowed range
// ==========================================================

  if (
      !validateTelemetry()
  ) {

    Serial.println(
        F(
          "[TELEMETRY] Invalid packet ignored."
        )
    );

    return;
  }


// ==========================================================
// 3. DYNAMIC FEATURES — calculate rate of change, variance, and thermal index
// ==========================================================

  updateDynamicFeatures();


// ==========================================================
// 4. AI — send the telemetry to the Linux-side AI through the UNO Q bridge
// ==========================================================

  RpcCall call =
      Bridge.call(
          "predict_json",
          telemetry
      );


  String resultJson;


// ==========================================================
// 5. AI RESPONSE — read the JSON result returned by the AI
// ==========================================================

  if (
      call.result(
          resultJson
      )
  ) {

    StaticJsonDocument<1024>
        doc;


    DeserializationError error =
        deserializeJson(
            doc,
            resultJson
        );


    if (
        !error
    ) {

// Apply the AI policy to the four relays, unless the emergency stop is active.

      if (
          emergencyStopActive
      ) {

        driveRelays(
            RELAY_OFF,
            RELAY_OFF,
            RELAY_OFF,
            RELAY_OFF
        );

      } else {

        applyPolicyState(
            doc["cmd"] | 0
        );
      }


// Print a human-readable summary so the system can be monitored during testing.

      printUnoQStatus(
          doc
      );


// Send the AI result and electrical data to the I2C display.

      constructAndSendDisplayPayload(
          doc
      );


// Send the same important status information to the RS485 output.

      rs485SendNeuralGridPacket(
          doc
      );

    } else {

      Serial.println(
          F(
            "[AI] Invalid JSON response."
          )
      );


      applyPolicyState(
          4
      );
    }

  } else {

// ========================================================
// AI FALLBACK — if the AI cannot respond, the safe action is to trip all relays.
// ========================================================

    StaticJsonDocument<256>
        fallbackAI;


    fallbackAI["cmd"] =
        4;


    fallbackAI["hlth"] =
        0;


    fallbackAI["ai"] =
        "AI_OFFLINE";


    fallbackAI["conf"] =
        0.0f;


    fallbackAI["anom"] =
        true;


    fallbackAI["raw"] =
        0.0f;


    fallbackAI["act"] =
        "EMERGENCY_SHUTDOWN";


    fallbackAI["estop"] =
        false;


    applyPolicyState(
        4
    );


    printUnoQStatus(
        fallbackAI
    );


    constructAndSendDisplayPayload(
        fallbackAI
    );


    rs485SendNeuralGridPacket(
        fallbackAI
    );
  }
}


// ============================================================================
// MAIN LOOP — runs continuously while the UNO Q is powered
// ============================================================================

void loop() {

  Bridge.update();


// ============================================================
// PZEM SENSOR UPDATE
// ============================================================

  unsigned long pzemTimeBefore =
      lastPZEMRead;


  updatePZEMSensors();


  if (
      lastPZEMRead !=
      pzemTimeBefore
  ) {

    // ----------------------------------------------------------
// PZEM STATUS — printed at most once every 30 seconds
    //
// The function below handles the 30-second print limit.
// It does not change the 1-second sensor reading rate.
    // ----------------------------------------------------------

    printPZEMStatus();


// ==========================================================
// PRIMARY PZEM #1 → AI TELEMETRY
// ==========================================================

    if (
        pzem1Data.valid &&
        !emergencyStopActive
    ) {

      String pzemTelemetry;


      pzemTelemetry.reserve(
          64
      );


      pzemTelemetry +=
          String(
              pzem1Data.voltage,
              2
          );


      pzemTelemetry +=
          ',';


      pzemTelemetry +=
          String(
              pzem1Data.current,
              3
          );


      pzemTelemetry +=
          ',';


      pzemTelemetry +=
          String(
              pzem1Data.pf,
              3
          );


      pzemTelemetry +=
          ',';


      pzemTelemetry +=
          String(
              pzem1Data.frequency,
              2
          );


      Serial.print(
          F("[AI INPUT / PZEM 1] ")
      );


      Serial.println(
          pzemTelemetry
      );


      processTelemetryPacket(
          pzemTelemetry
      );


    } else if (
        !pzem1Data.valid
    ) {

// --------------------------------------------------------
// PZEM #1 ERROR MESSAGE
//--------------------------------------------------------


      unsigned long now =
          millis();


      if (
          (now -
           lastPZEMErrorPrint) >=
          PZEM_ERROR_PRINT_INTERVAL
      ) {

        lastPZEMErrorPrint =
            now;


        Serial.println(
            F(
              "[AI INPUT] PZEM #1 unavailable — AI update skipped."
            )
        );
      }
    }
  }


// ============================================================
// EMERGENCY STOP — safety check happens before manual Serial processing
// ============================================================

  updateEmergencyStopState();


  if (
      emergencyStopActive
  ) {

    while (
        Serial.available()
    ) {

      Serial.read();
    }


    return;
  }


// ============================================================
// USB SERIAL TELEMETRY INPUT — allows manual test data from the Serial Monitor
// ============================================================

  while (
      Serial.available()
  ) {

    char c =
        Serial.read();


    if (
        c == '\n'
    ) {

      serialBuffer.trim();


      if (
          serialBuffer.length() > 0
      ) {

        processTelemetryPacket(
            serialBuffer
        );


        serialBuffer =
            "";
      }

    } else if (
        c != '\r'
    ) {

      if (
          serialBuffer.length() <
          512
      ) {

        serialBuffer +=
            c;

      } else {

        serialBuffer =
            "";

        Serial.println(
            F(
              "[SERIAL] Input buffer overflow — cleared."
            )
        );
      }
    }
  }
}