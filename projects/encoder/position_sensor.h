/****************************************************************************
 * apps/examples/encoder/position_sensor.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_ENCODER_POSITION_SENSOR_H
#define __APPS_EXAMPLES_ENCODER_POSITION_SENSOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define N_POS_SAMPLES 20
#define N_LUT         128

typedef struct
{
  union
    {
      uint8_t spi_tx_buff[2];
      uint16_t spi_tx_word;
    };
  union
    {
      uint8_t spi_rx_buff[2];
      uint16_t spi_rx_word;
    };
  float angle_singleturn;
  float old_angle;
  float angle_multiturn[N_POS_SAMPLES];
  float elec_angle;
  float velocity;
  float elec_velocity;
  float ppairs;
  float vel2;
  float output_angle_multiturn;
  int raw;
  int count;
  int old_count;
  int turns;
  int count_buff[N_POS_SAMPLES];
  int m_zero;
  int e_zero;
  int offset_lut[N_LUT];
  uint8_t first_sample;
} EncoderStruct;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int ps_open(FAR const char *devpath);
void ps_close(void);
void ps_warmup(FAR EncoderStruct *encoder, int n);
void ps_sample(FAR EncoderStruct *encoder, float dt);
void ps_print(FAR EncoderStruct *encoder, int dt_ms);

#endif /* __APPS_EXAMPLES_ENCODER_POSITION_SENSOR_H */
