// =============================================================================
// Header file for fcmp serial operations
// =============================================================================
// License: MIT
// Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
// =============================================================================
#ifndef FCMP_SERIAL_H
#define FCMP_SERIAL_H

#include "fcmp.h"
#include <cobs.h>
#include <string.h>

extern cobs_buffer_t cobs_rx, cobs_tx;

extern uint8_t fcmp_buffer[2][COBS_MAX_CONTENT];

#define FCMP_RX_BUFFER ((fcmp_frame_t *)cobs_rx.data)
// #define FCMP_RX_BUFFER ((fcmp_frame_t *)fcmp_buffer[0])
#define FCMP_TX_BUFFER ((fcmp_frame_t *)fcmp_buffer[1])

struct fcmp_state_s {
  fcmp_field_cfg config_list, config_mask;
  bool position_pending;
  fcmp_field_pos position;
  bool lpf_pending;
  fcmp_field_lpf lpf;
  char log[COBS_MAX_CONTENT + 1];
};

extern struct fcmp_state_s self;

void fcmp_config_apply();

void fcmp_config_update(uint16_t);

void fcmp_log(const char *log, uint8_t len);

void fcmp_reject(uint8_t method, uint8_t field);

#define fcmp_print(STR)                                                        \
  { fcmp_log(STR, strlen(STR)); }

#define fcmp_printf(...)                                                       \
  {                                                                            \
    if (TEST_BIT(self.config_list, FCMP_CFG_BIT_LOG)) {                        \
      sprintf(self.log, __VA_ARGS__);                                          \
      fcmp_print(self.log);                                                    \
    }                                                                          \
  }

void fcmp_handle_frame(const uint8_t frame_size);

#endif
