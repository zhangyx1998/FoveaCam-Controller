#include "fcmp_serial.h"
#include "board_pinout.h"
#include "cobs.h"
#include "fcmp.h"
#include "mems_drv.h"
#include <Arduino.h>
#include <stdint.h>

cobs_buffer_t cobs_rx, cobs_tx;

uint8_t fcmp_buffer[2][COBS_MAX_CONTENT];

struct fcmp_state_s self = {
    .config_list = (0 << FCMP_CFG_BIT_MEMS_EN) |
                   (0 << FCMP_CFG_BIT_STROBE_SYNC) | (0 << FCMP_CFG_BIT_LPF) |
                   (0 << FCMP_CFG_BIT_LOG),
    .config_mask = (1 << FCMP_CFG_BIT_MEMS_EN) |
                   (1 << FCMP_CFG_BIT_STROBE_SYNC) | (1 << FCMP_CFG_BIT_LPF) |
                   (1 << FCMP_CFG_BIT_LOG),
    .position_pending = false,
    .position = {0, 0, 0, 0},
    .lpf = 100,
    .log = "",
};

void fcmp_config_apply() {
  if (!self.config_mask)
    return;
  if (TEST_BIT(self.config_mask, FCMP_CFG_BIT_MEMS_EN)) {
    if (TEST_BIT(self.config_list, FCMP_CFG_BIT_MEMS_EN))
      mems_enable();
    else
      mems_disable();
  }
  if (TEST_BIT(self.config_mask, FCMP_CFG_BIT_LPF)) {
    if (TEST_BIT(self.config_list, FCMP_CFG_BIT_LPF))
      tone(PIN_LPF, self.lpf * 60);
    else
      noTone(PIN_LPF);
  }
  if (TEST_BIT(self.config_mask, FCMP_CFG_BIT_LOG)) {
    static const char log[] = "FCMP log enabled";
    fcmp_log(log, sizeof(log));
  }
  self.config_mask = 0;
}

void fcmp_log(const char *log, uint8_t len) {
  if (!TEST_BIT(self.config_list, FCMP_CFG_BIT_LOG)) {
    return;
  }
  // Compose log frame
  const uint8_t fcmp_len = sizeof(fcmp_frame_t) + len;
  fcmp_compose_frame(FCMP_TX_BUFFER, FCMP_METHOD_LOG | FCMP_FIELD_ANY, log);
  FCMP_TX_BUFFER->checksum = 0;
  FCMP_TX_BUFFER->header = FCMP_METHOD_LOG | FCMP_FIELD_ANY;
  memcpy(FCMP_TX_BUFFER->field, log, len);
  FCMP_TX_BUFFER->checksum = fcmp_checksum((uint8_t *)FCMP_TX_BUFFER, fcmp_len);
  // Reset COBS buffer
  cobs_reset(&cobs_tx);
  // COBS encode
  uint8_t cobs_len = cobs_encode(&cobs_tx, (uint8_t *)FCMP_TX_BUFFER, fcmp_len);
  // Send through serial, including tailing delimiter
  Serial.write(cobs_tx.data, (size_t)cobs_len + 1);
}

static const char *default_reject_reason = "Unknown reason";

void fcmp_reject(uint8_t method, uint8_t field,
                 const char *reason = default_reject_reason) {
  uint8_t frame_size = __fcmp_compose_frame__(
      FCMP_TX_BUFFER, FCMP_METHOD_REJ | field, (void *)reason, strlen(reason));
  cobs_reset(&cobs_tx);
  int ret = cobs_encode(&cobs_tx, (uint8_t *)FCMP_TX_BUFFER, frame_size);
  if (ret > 0)
    Serial.write(cobs_tx.data, ret + 1);
}

void fcmp_handle_frame(const uint8_t frame_size) {
  const uint8_t method = FCMP_RX_BUFFER->header & FCMP_METHOD,
                field = FCMP_RX_BUFFER->header & FCMP_FIELD,
                field_size = frame_size - sizeof(fcmp_frame_t);
  // Switch depending on request method
  switch (method) {
  case FCMP_METHOD_NOP:
    // Do nothing, ignore field type
    fcmp_print("NOP");
    return;
  case FCMP_METHOD_ACK:
  case FCMP_METHOD_REJ:
    // Discard frame
    return;
  case FCMP_METHOD_LOG:
    // Echo Back
    fcmp_print("LOG (echo back)");
    fcmp_log((char *)FCMP_RX_BUFFER->field, field_size);
    return;
  default:
    // Enter next switch statement
    break;
  }
  // Switch depending on field type (request name)
  switch (field) {
  // Set/Get Configuration
  case FCMP_FIELD_CFG:
    switch (method) {
    case FCMP_METHOD_SET: {
      fcmp_print("SET CFG");
      if (field_size != sizeof(fcmp_field_cfg))
        return fcmp_reject(method, field, "Invalid field size");
      fcmp_field_cfg config = *(fcmp_field_cfg *)FCMP_RX_BUFFER->field;
      self.config_mask = self.config_list ^ config;
      self.config_list = config;
      fcmp_config_apply();
    }
    case FCMP_METHOD_GET:
      fcmp_print("GET CFG");
      fcmp_cobs_tx(&cobs_tx, FCMP_TX_BUFFER, FCMP_METHOD_ACK | FCMP_FIELD_CFG,
                   self.config_list) return;
    default:
      return fcmp_reject(method, field, "Invalid method");
    }
    break;
  // Set/Get Position
  case FCMP_FIELD_POS:
    switch (method) {
    case FCMP_METHOD_SET:
      fcmp_print("SET POSITION");
      if (field_size != sizeof(fcmp_field_pos))
        return fcmp_reject(method, field, "Invalid field size");
      if (self.position_pending) {
        // Position change already pending
        return fcmp_reject(method, field, "Position change already pending");
      } else {
        self.position_pending = true;
        memcpy(&self.position, FCMP_RX_BUFFER->field, field_size);
        // ACK will be sent after position change is complete
        return;
      }
    case FCMP_METHOD_GET:
      fcmp_print("GET POSITION");
      fcmp_cobs_tx(&cobs_tx, FCMP_TX_BUFFER, FCMP_METHOD_ACK | FCMP_FIELD_POS,
                   self.position);
      return;
    default:
      return fcmp_reject(method, field, "Invalid method");
    }
    // Set/Get LP Filter
  case FCMP_FIELD_LPF:
    switch (method) {
    case FCMP_METHOD_SET:
      fcmp_print("SET LP FILTER");
      if (field_size != sizeof(fcmp_field_lpf))
        return fcmp_reject(method, field, "Invalid field size");
      self.lpf_pending = true;
      memcpy(&self.lpf, FCMP_RX_BUFFER->field, field_size);
    case FCMP_METHOD_GET:
      fcmp_print("GET LP FILTER");
      fcmp_cobs_tx(&cobs_tx, FCMP_TX_BUFFER, FCMP_METHOD_ACK | FCMP_FIELD_LPF,
                   self.lpf);
      return;
    default:
      return fcmp_reject(method, field, "Invalid method");
    }
  default:
    // Unknown field type
    return fcmp_reject(method, field, "Unknown field id");
  }
}
