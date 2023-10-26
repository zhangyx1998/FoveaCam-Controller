#include "mems_drv.h"
#include "board_pinout.h"
#include "fcmp_serial.h"
#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

// Driver board accepts 3 Byte SPI word:
// Header (1 Byte) | Value (2 Bytes, Big Endian)
// Last (4th) byte of spi_word is unused
typedef union {
  uint32_t word;
  uint8_t byte[3];
  struct {
    uint8_t const header;
    uint8_t value[2];
  };
} spi_word;

#define SPI_HEAD(CMD, CH) (((CMD) << 3) | (CH))

#define SPI_WORD(CMD, CH, VAL)                                                 \
  {                                                                            \
    .header = (uint8_t)SPI_HEAD(CMD, CH), .value = {                           \
      (uint8_t)(((VAL) >> 8) & 0xFF),                                          \
      (uint8_t)((VAL)&0xFF)                                                    \
    }                                                                          \
  }

enum CMD_TYPE {
  // Write to Input Register N
  CMD_WRITE_IREG = 0b000,
  // Update DAC Register N
  CMD_UPDATE_DAC = 0b001,
  // Write to Input Register N, Update All (software LDAC)
  CMD_WR_LD_IREG = 0b010,
  // Write and Update DAC Register N
  CMD_WRITE_UPDATE_DAC = 0b011,
  // DAC Power Control (specified by 0~4 bits in the value field)
  CMD_DAC_POWER = 0b100,
  // Reset
  CMD_RESET = 0b101,
  // LDAC Register Setup
  CMD_LDAC_SETUP = 0b110,
  // Internal Reference Setup (on/off)
  CMD_INT_REF_SETUP = 0b111,
};

enum CMD_CHANNEL {
  DAC_NONE = 0b000, // Used when DAC field is ignored
  DAC_CH_A = 0b000,
  DAC_CH_B = 0b001,
  DAC_CH_C = 0b010,
  DAC_CH_D = 0b011,
  DAC_ALL = 0b111,
};

void spi_send(const spi_word *word, const size_t count = 1) {
  for (size_t i = 0; i < count; i++) {
    digitalWrite(PIN_SPI_CS, LOW);
    // SPI.transfer((void *)&word[i].byte, NULL, 3);
    for (uint8_t j = 0; j < 3; j++) {
      SPI.transfer(word[i].byte[j]);
    }
    digitalWrite(PIN_SPI_CS, HIGH);
    fcmp_printf("SPI -> %02X %02X %02X", word[i].byte[0], word[i].byte[1],
                word[i].byte[2]);
  }
}

static const spi_word start_sequence[] = {
    // 0x280001 FULL RESET
    SPI_WORD(CMD_RESET, DAC_NONE, 1),
    // 0x380001 ENABLE INTERNAL REFERENCE
    SPI_WORD(CMD_INT_REF_SETUP, DAC_NONE, 1),
    // 0x20000F ENABLE ALL DAC Channels
    SPI_WORD(CMD_DAC_POWER, DAC_NONE, 0b1111),
    // 0x300000 ENABLE SOFTWARE LDAC
    SPI_WORD(CMD_LDAC_SETUP, DAC_NONE, 0),
    // Set V_BIAS to Neutral
    SPI_WORD(CMD_WRITE_UPDATE_DAC, DAC_ALL, 0x6666),
};

static const spi_word end_sequence[] = {
    // Set V_BIAS to Neutral
    SPI_WORD(CMD_WRITE_UPDATE_DAC, DAC_ALL, 0x6666),
};

#define SPI_SEND_STATIC(SEQ) spi_send(SEQ, sizeof(SEQ) / sizeof(spi_word))

void mems_enable() {
  fcmp_print("MEMS Enable");
  digitalWrite(PIN_MEMS_EN, HIGH);
  delay(1); // Wait for board to wake up
  SPI_SEND_STATIC(start_sequence);
}

void mems_disable() {
  fcmp_print("MEMS Disable");
  if (digitalRead(PIN_MEMS_EN) == HIGH) {
    // MEMS_EN == HIGH, driver board is alive
    SPI_SEND_STATIC(end_sequence);
    delay(1); // Wait for V_BIAS to settle
    digitalWrite(PIN_MEMS_EN, LOW);
  } else {
    // MEMS_EN == LOW, no action required
  }
}

void mems_bias(uint16_t bias) {
  fcmp_printf("MEMS Bias: %d", bias);
  // Set Four Channel Values (only if changed)
  // spi_send(0x1F0000 | (bias & 0xFFFF));
}

static spi_word channel_sequence[4] = {
    {.header = SPI_HEAD(CMD_WRITE_IREG, DAC_CH_A)},
    {.header = SPI_HEAD(CMD_WRITE_IREG, DAC_CH_B)},
    {.header = SPI_HEAD(CMD_WRITE_IREG, DAC_CH_C)},
    {.header = SPI_HEAD(CMD_WR_LD_IREG, DAC_CH_D)},
};

void mems_set(fcmp_field_pos *pos) {
  fcmp_printf("mems set: %5d %5d %5d %5d", pos->ch[0], pos->ch[1], pos->ch[2],
              pos->ch[3]);
  // Set Four Channel Values (only if changed)
  for (uint8_t i = 0; i < 4; i++) {
    // Little Endian to Big Endian
    channel_sequence[i].value[0] = (uint8_t)((pos->ch[i] >> 8) & 0xFF);
    channel_sequence[i].value[1] = (uint8_t)(pos->ch[i] & 0xFF);
  }
  // Send to driver board
  SPI_SEND_STATIC(channel_sequence);
}
