/*
 * stm32_modbus_bridge.ino — TEMPLATE, not yet tested on hardware.
 *
 * STM32F103C8T6 (Blue Pill, STM32duino) as Modbus RTU master for
 * 4x RMCS-3001 (Mode 1 digital closed-loop, slave IDs 1-4) on a mecanum base.
 *
 * Wiring (TODO: confirm free UART):
 *   STM32 TX2 (PA2?) -> MAX485 DI, RX2 -> RO, PA8 -> DE+RE
 *   MAX485 A/B -> daisy-chained A/B on all 4 drives (120R terminator on ends)
 *   USB Serial -> RPi/laptop (115200): "V vx vy wz" in, "O h0 h1 h2 h3" out
 *
 * RMCS-3001 registers used (RTU, default 9600 8N1 — confirm on your units):
 *   0x02 (40003) control: 0x0101 enable CW, 0x0109 enable CCW, 0x0100 stop
 *   0x06 (40007) frequency 0-400 Hz target
 *   0x08 (40009) speed feedback Hz (read)
 * Wheel RPM = 60 * Hz / POLE_PAIRS (TODO: set from NEMA32 datasheet).
 *
 * Mecanum IK (X-config, matches rmcs3001_mecanum.ino tables):
 *   w_FL = vy + vx + L*wz, w_FR = vy - vx - L*wz,
 *   w_RL = vy - vx + L*wz, w_RR = vy + vx - L*wz,  L = LX + LY.
 * Sign<0 -> CCW word, magnitude -> Hz (clamped to 400).
 *
 * Bring-up order: set slave IDs 1-4 + Mode 1 via vendor tool first,
 * then flash this, wheels on blocks, send "V 0.1 0 0" and check "O ..." reply.
 */

#include <Arduino.h>

// ---------------- TODO: confirm pins ----------------
#define RS485_SERIAL Serial2   // check STM32duino mapping for your wiring
#define RS485_BAUD 9600
#define DE_PIN PA8             // DE+RE tied together
#define USB_BAUD 115200

// ---------------- geometry (must match URDF + ROS bridge) ----------------
const float WHEEL_R = 0.048;
const float LX = 0.220, LY = 0.1196;
const int POLE_PAIRS = 4;      // TODO: NEMA32 datasheet
const float MAX_HZ = 400.0;
// 0.5 m/s * pp / (2*pi*r) ≈ tiny — Hz target for full speed:
const float VEL_TO_HZ = POLE_PAIRS / (2.0 * PI * WHEEL_R);  // hz per (m/s)

#define N_SLAVES 4
const uint8_t SLAVE[N_SLAVES] = {1, 2, 3, 4};

// ---------------- Modbus RTU CRC16 ----------------
static uint16_t crc16(const uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

static void rs485Write(const uint8_t *b, uint8_t n) {
  digitalWrite(DE_PIN, HIGH);
  RS485_SERIAL.write(b, n);
  RS485_SERIAL.flush();
  digitalWrite(DE_PIN, LOW);
}

// Write single holding register (fn 06). Returns true on valid echo.
static bool modbusWriteReg(uint8_t slave, uint16_t addr, uint16_t val) {
  uint8_t q[8] = {slave, 0x06, (uint8_t)(addr >> 8), (uint8_t)addr,
                  (uint8_t)(val >> 8), (uint8_t)val, 0, 0};
  uint16_t c = crc16(q, 6);
  q[6] = c & 0xFF; q[7] = c >> 8;
  rs485Write(q, 8);
  delay(15);
  uint8_t r[8]; uint8_t n = 0;
  while (RS485_SERIAL.available() && n < 8) r[n++] = RS485_SERIAL.read();
  return n == 8 && r[0] == slave && r[1] == 0x06;
}

// Read 1 holding register (fn 03). Returns true, fills *val.
static bool modbusReadReg(uint8_t slave, uint16_t addr, uint16_t *val) {
  uint8_t q[8] = {slave, 0x03, (uint8_t)(addr >> 8), (uint8_t)addr, 0, 1, 0, 0};
  uint16_t c = crc16(q, 6);
  q[6] = c & 0xFF; q[7] = c >> 8;
  rs485Write(q, 8);
  delay(15);
  uint8_t r[7]; uint8_t n = 0;
  while (RS485_SERIAL.available() && n < 7) r[n++] = RS485_SERIAL.read();
  if (n < 7 || r[0] != slave || r[1] != 0x03) return false;
  *val = ((uint16_t)r[3] << 8) | r[4];
  return true;
}

// ---------------- drive helpers ----------------
static void driveWheel(uint8_t i, float wheel_ms) {
  // sign -> direction word, magnitude -> Hz
  uint16_t ctrl = 0x0100;  // stop
  uint16_t hz = 0;
  if (fabs(wheel_ms) > 0.01) {
    ctrl = (wheel_ms >= 0) ? 0x0101 : 0x0109;  // CW / CCW
    hz = (uint16_t)constrain(fabs(wheel_ms) * VEL_TO_HZ, 0, MAX_HZ);
  }
  modbusWriteReg(SLAVE[i], 0x06, hz);
  modbusWriteReg(SLAVE[i], 0x02, ctrl);
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(USB_BAUD);
  RS485_SERIAL.begin(RS485_BAUD);
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);
  Serial.println(F("stm32_modbus_bridge TEMPLATE — wheels on blocks!"));
  Serial.println(F("send: V <vx> <vy> <wz>   e.g. V 0.1 0 0"));
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      float vx, vy, wz;
      if (sscanf(line.c_str(), "V %f %f %f", &vx, &vy, &wz) == 3) {
        float L = LX + LY;
        float w[4] = {vy + vx + L * wz, vy - vx - L * wz,
                      vy - vx + L * wz, vy + vx - L * wz};
        for (uint8_t i = 0; i < N_SLAVES; i++) driveWheel(i, w[i]);
      }
      line = "";
    } else if (c != '\r') line += c;
  }

  // 10 Hz feedback reply: O h0 h1 h2 h3 (Hz, FL FR RL RR)
  static uint32_t last = 0;
  if (millis() - last > 100) {
    last = millis();
    Serial.print(F("O"));
    for (uint8_t i = 0; i < N_SLAVES; i++) {
      uint16_t hz = 0;
      modbusReadReg(SLAVE[i], 0x08, &hz);
      Serial.print(F(" "));
      Serial.print(hz);
    }
    Serial.println();
  }
}
