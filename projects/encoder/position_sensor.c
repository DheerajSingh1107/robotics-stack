/****************************************************************************
 * apps/examples/encoder/position_sensor.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#ifdef CONFIG_SPI_EXCHANGE
#  include <nuttx/spi/spi.h>
#  include <nuttx/spi/spi_transfer.h>
#endif

#include "position_sensor.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_ENCODER_READ_WORD
#  define CONFIG_EXAMPLES_ENCODER_READ_WORD 0x00
#endif

#ifndef CONFIG_EXAMPLES_ENCODER_CPR
#  define CONFIG_EXAMPLES_ENCODER_CPR 16384
#endif

#ifndef CONFIG_EXAMPLES_ENCODER_SPI_MODE
#  define CONFIG_EXAMPLES_ENCODER_SPI_MODE 1
#endif

#ifndef CONFIG_EXAMPLES_ENCODER_SPI_NBITS
#  define CONFIG_EXAMPLES_ENCODER_SPI_NBITS 16
#endif

#ifndef CONFIG_EXAMPLES_ENCODER_SPI_FREQ
#  define CONFIG_EXAMPLES_ENCODER_SPI_FREQ 1000000
#endif

#define PI_F           3.14159265358979323846f
#define TWO_PI_F       (2.0f * PI_F)
#define PI_OVER_2_F    (0.5f * PI_F)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_spi_fd = -1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ps_spi_exchange16(uint16_t tx, FAR uint16_t *rx)
{
#ifdef CONFIG_SPI_EXCHANGE
  struct spi_trans_s trans;
  struct spi_sequence_s seq;
  uint16_t loc_rx = 0;
  int ret;

  if (g_spi_fd < 0)
    {
      return -ENODEV;
    }

  memset(&trans, 0, sizeof(trans));
  memset(&seq, 0, sizeof(seq));

  trans.nwords   = 1;
  trans.txbuffer = &tx;
  trans.rxbuffer = &loc_rx;
  trans.deselect = true;

  seq.dev       = SPIDEV_USER(0);
  seq.mode      = CONFIG_EXAMPLES_ENCODER_SPI_MODE;
  seq.nbits     = CONFIG_EXAMPLES_ENCODER_SPI_NBITS;
  seq.ntrans    = 1;
  seq.frequency = CONFIG_EXAMPLES_ENCODER_SPI_FREQ;
  seq.trans     = &trans;

  ret = ioctl(g_spi_fd, SPIIOC_TRANSFER, (unsigned long)((uintptr_t)&seq));
  if (ret < 0)
    {
      return -errno;
    }

  if (rx != NULL)
    {
      *rx = loc_rx;
    }

  return 0;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ps_open(FAR const char *devpath)
{
#ifndef CONFIG_SPI_EXCHANGE
  (void)devpath;
  return -ENOSYS;
#else
  if (g_spi_fd >= 0)
    {
      return 0;
    }

   g_spi_fd = open(devpath, O_RDONLY);
  if (g_spi_fd < 0)
    {
      return -errno;
    }

  return 0;
#endif
}

void ps_close(void)
{
  if (g_spi_fd >= 0)
    {
      close(g_spi_fd);
      g_spi_fd = -1;
    }
}

void ps_warmup(FAR EncoderStruct *encoder, int n)
{
  int i;

  if (encoder == NULL || n <= 0)
    {
      return;
    }

  for (i = 0; i < n; i++)
    {
      (void)ps_spi_exchange16(0x0000, &encoder->spi_rx_word);
    }
}

void ps_sample(FAR EncoderStruct *encoder, float dt)
{
  int i;
  int off_1;
  int off_2;
  int off_interp;
  int int_angle;
  int rollover = 0;
  int ret;
  float angle_diff;

  if (encoder == NULL || dt <= 0.0f)
    {
      return;
    }

  encoder->old_angle = encoder->angle_singleturn;
  for (i = N_POS_SAMPLES - 1; i > 0; i--)
    {
      encoder->angle_multiturn[i] = encoder->angle_multiturn[i - 1];
    }

  encoder->spi_tx_word = CONFIG_EXAMPLES_ENCODER_READ_WORD;
  ret = ps_spi_exchange16(encoder->spi_tx_word, &encoder->spi_rx_word);
  if (ret < 0)
    {
      encoder->raw = -1;
      return;
    }

  encoder->raw = encoder->spi_rx_word >> 2;

  off_1 = encoder->offset_lut[(encoder->raw) >> 7];
  off_2 = encoder->offset_lut[((encoder->raw >> 7) + 1) % 128];
  off_interp = off_1 + ((off_2 - off_1) *
                        (encoder->raw - ((encoder->raw >> 7) << 7)) >> 7);
  encoder->count = encoder->raw + off_interp;

  encoder->angle_singleturn =
    ((float)(encoder->count - encoder->m_zero)) /
    ((float)CONFIG_EXAMPLES_ENCODER_CPR);
  int_angle = encoder->angle_singleturn;
  encoder->angle_singleturn =
    TWO_PI_F * (encoder->angle_singleturn - (float)int_angle);
  encoder->angle_singleturn =
    encoder->angle_singleturn < 0 ? encoder->angle_singleturn + TWO_PI_F :
                                    encoder->angle_singleturn;

  encoder->elec_angle =
    (encoder->ppairs * (float)(encoder->count - encoder->e_zero)) /
    ((float)CONFIG_EXAMPLES_ENCODER_CPR);
  int_angle = (int)encoder->elec_angle;
  encoder->elec_angle = TWO_PI_F * (encoder->elec_angle - (float)int_angle);
  encoder->elec_angle =
    encoder->elec_angle < 0 ? encoder->elec_angle + TWO_PI_F :
                              encoder->elec_angle;

  angle_diff = encoder->angle_singleturn - encoder->old_angle;
  if (angle_diff > PI_F)
    {
      rollover = -1;
    }
  else if (angle_diff < -PI_F)
    {
      rollover = 1;
    }

  encoder->turns += rollover;
  if (!encoder->first_sample)
    {
      encoder->turns = 0;
      if (encoder->angle_singleturn > PI_OVER_2_F)
        {
          encoder->turns = -1;
        }
      else if (encoder->angle_singleturn < -PI_OVER_2_F)
        {
          encoder->turns = 1;
        }

      encoder->first_sample = 1;
    }

  encoder->angle_multiturn[0] = encoder->angle_singleturn +
                                TWO_PI_F * (float)encoder->turns;
  encoder->velocity =
    (encoder->angle_multiturn[0] - encoder->angle_multiturn[N_POS_SAMPLES - 1]) /
    (dt * (float)(N_POS_SAMPLES - 1));
  encoder->elec_velocity = encoder->ppairs * encoder->velocity;
}

void ps_print(FAR EncoderStruct *encoder, int dt_ms)
{
  if (encoder == NULL)
    {
      return;
    }

  printf("   Raw: %d", encoder->raw);
  printf("   Linearized Count: %d", encoder->count);
  printf("   Single Turn: %f"     ,encoder->angle_singleturn);
  printf("   Multiturn: %f" ,encoder->angle_multiturn[0]);
  printf("   Electrical: %f", encoder->elec_angle);
  printf("   Turns: %d", encoder->turns);
  printf("   Velocity: %f\r\n", encoder->velocity);

  if (dt_ms > 0)
    {
      usleep((useconds_t)dt_ms * 1000);
    }
}
