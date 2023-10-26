// =============================================================================
// This is the tenssy firmware for the MEMS controller board.
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#include "board_pinout.h"
#include "fcmp_serial.h"
#include "mems_drv.h"
#include <Arduino.h>
#include <SPI.h>
#include <cobs.h>
void setup() {
  // Serial
  Serial.begin(SERIAL_BAUD);
  // MEMS Enable
  pinMode(PIN_MEMS_EN, OUTPUT);
  digitalWrite(PIN_MEMS_EN, LOW);
  // trigger -> {camera} -> strobe (start-end of capture)
  // Camera trigger out / strobe in
  pinMode(CAMERA_STROBE, INPUT);
  // SPI
  pinMode(PIN_SPI_CS, OUTPUT);
  digitalWrite(PIN_SPI_CS, HIGH);
  SPI.begin();
  fcmp_config_apply();
}

struct {
  // Will be set to true when COBS decoding error occurs.
  // Will discard all incoming bytes until next zero byte.
  bool flag_flush;
  // Number of available bytes
  unsigned length;
  uint8_t buffer[COBS_MAX_ENCODED];
} serial_rx = {.flag_flush = false, .length = 0};

void loop() {
  // a3d::loop();
  // return;
  // Receive serial data
  int serial_bytes = Serial.available();
  if (serial_bytes > 0) {
    unsigned space = sizeof(serial_rx.buffer) - serial_rx.length;
    if ((unsigned)serial_bytes > space) {
      // Serial buffer overflow
      serial_bytes = space;
    }
    // Read serial data
    Serial.readBytes((char *)serial_rx.buffer + serial_rx.length,
                     (size_t)serial_bytes);
    serial_rx.length += serial_bytes;
  } else if (serial_bytes < 0) {
    fcmp_printf("USB Serial Error %d", serial_bytes);
    // Serial error
    Serial.flush();
    Serial.end();
    Serial.begin(SERIAL_BAUD);
  }
  // Check for flush flag (caused by previous COBS decoding error)
  if (serial_rx.flag_flush) {
    unsigned i;
    // Flush serial buffer until next zero byte
    for (i = 0; i < serial_rx.length; i++) {
      if (serial_rx.buffer[i] == 0) {
        // Shift remaining data
        break;
      }
    }
    i++;
    if (i >= serial_rx.length) {
      // No zero byte found, flush all
      serial_rx.length = 0;
    } else {
      // Zero byte found, flush all bytes before it
      move_ahead(serial_rx.buffer, &serial_rx.length, i);
      serial_rx.length -= i;
      memmove(serial_rx.buffer, serial_rx.buffer + i, serial_rx.length);
      serial_rx.flag_flush = false;
    }
  }
  // Process serial data
  if (serial_rx.length > 0) {
    // Decode COBS
    int cobs_ret = cobs_decode(&cobs_rx, serial_rx.buffer, serial_rx.length);
    if (cobs_ret > 0) {
      move_ahead(serial_rx.buffer, &serial_rx.length, cobs_ret);
      // Process FCMP frame
      uint8_t checksum = fcmp_checksum(cobs_rx.data, cobs_rx.length);
      if (checksum != 0) {
        // Checksum error
        fcmp_printf("FCMP Checksum Error <0x%02X>", checksum);
      } else {
        fcmp_handle_frame(cobs_rx.length);
      }
      // Reset COBS decoder
      cobs_reset(&cobs_rx);
    } else if (cobs_ret < 0) {
      // COBS error
      cobs_reset(&cobs_rx);
      serial_rx.flag_flush = true;
      fcmp_printf("COBS Decode Error %d", cobs_ret);
    }
    // Else: cobs_ret == 0
    // Incomplete COBS frame
    // Do nothing, wait for more data
  }
  // Check if new position data is available for SPI transmission
  if (self.position_pending &&
      TEST_BIT(self.config_list, FCMP_CFG_BIT_MEMS_EN) &&
      !TEST_BIT(self.config_list, FCMP_CFG_BIT_STROBE_SYNC)) {
    // Reset flag
    self.position_pending = false;
    // Send through SPI, including tailing delimiter
    mems_set((fcmp_field_pos *)&self.position);
    // Acknowledge position change
    fcmp_compose_frame(FCMP_TX_BUFFER, FCMP_METHOD_ACK | FCMP_FIELD_ANY,
                       self.position);
  }
  // Check if new LP filter data is available for SPI transmission
  if (self.lpf_pending && TEST_BIT(self.config_list, FCMP_CFG_BIT_LPF)) {
    // Reset flag
    self.lpf_pending = false;
    // Update lp filter clock
    tone(PIN_LPF, self.lpf * 60);
  }
}
