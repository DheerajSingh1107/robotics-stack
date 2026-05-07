/****************************************************************************
 * apps/examples/encoder/encoder_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "position_sensor.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_ENCODER_DEFAULT_FREQ
#  define CONFIG_EXAMPLES_ENCODER_DEFAULT_FREQ 20000
#endif

#ifndef CONFIG_EXAMPLES_ENCODER_DEVPATH
#  define CONFIG_EXAMPLES_ENCODER_DEVPATH "/dev/spi3"
#endif

#ifndef CONFIG_EXAMPLES_ENCODER_WARMUP_SAMPLES
#  define CONFIG_EXAMPLES_ENCODER_WARMUP_SAMPLES 20
#endif

#ifndef CONFIG_EXAMPLES_ENCODER_PPAIRS
#  define CONFIG_EXAMPLES_ENCODER_PPAIRS 7
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile sig_atomic_t g_encoder_run = 1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void encoder_signal_handler(int signo)
{
  (void)signo;
  g_encoder_run = 0;
}

static void encoder_usage(void)
{
  printf("Usage:\n");
  printf("  encoder start [freq_hz]\n");
  printf("  encoder once [freq_hz]\n");
  printf("  encoder status\n");
  printf("\n");
  printf("Defaults: freq=%dHz dev=%s\n",
         CONFIG_EXAMPLES_ENCODER_DEFAULT_FREQ,
         CONFIG_EXAMPLES_ENCODER_DEVPATH);
}

static int encoder_run(unsigned int freq_hz, bool once)
{
  EncoderStruct encoder;
  unsigned int effective_freq_hz;
  unsigned int period_us;
  unsigned int print_div;
  unsigned int sample_count = 0;
  float dt;
  int ret;

  if (freq_hz == 0)
    {
      freq_hz = CONFIG_EXAMPLES_ENCODER_DEFAULT_FREQ;
    }

  memset(&encoder, 0, sizeof(encoder));
  encoder.ppairs = (float)CONFIG_EXAMPLES_ENCODER_PPAIRS;

  ret = ps_open(CONFIG_EXAMPLES_ENCODER_DEVPATH);
  if (ret < 0)
    {
      printf("encoder: ps_open(%s) failed: %d\n",
             CONFIG_EXAMPLES_ENCODER_DEVPATH, -ret);
      return EXIT_FAILURE;
    }

  ps_warmup(&encoder, CONFIG_EXAMPLES_ENCODER_WARMUP_SAMPLES);

  period_us = 1000000u / freq_hz;
  if (period_us == 0)
    {
      period_us = 1;
    }

#ifndef CONFIG_SCHED_TICKLESS
  if (period_us < CONFIG_USEC_PER_TICK)
    {
      printf("encoder: requested %uHz exceeds scheduler resolution (%uus tick), "
             "clamping period to %uus\n",
             freq_hz, CONFIG_USEC_PER_TICK, CONFIG_USEC_PER_TICK);
      period_us = CONFIG_USEC_PER_TICK;
    }
#endif

  effective_freq_hz = 1000000u / period_us;
  if (effective_freq_hz == 0)
    {
      effective_freq_hz = 1;
    }

  dt = (float)period_us / 1000000.0f;

  print_div = effective_freq_hz / 50u;
  if (print_div == 0)
    {
      print_div = 1;
    }

  if (once)
    {
      ps_sample(&encoder, dt);
      ps_print(&encoder, 0);
      ps_close();
      return EXIT_SUCCESS;
    }

  signal(SIGINT, encoder_signal_handler);
  g_encoder_run = 1;

  printf("encoder: sampling started requested=%uHz effective=%uHz (~%uus)\n",
         freq_hz, effective_freq_hz, period_us);
  printf("encoder: stop with Ctrl+C\n");

  while (g_encoder_run)
    {
      ps_sample(&encoder, dt);
      sample_count++;

      if ((sample_count % print_div) == 0)
        {
          ps_print(&encoder, 0);
        }

      usleep(period_us);
    }

  ps_close();
  printf("encoder: sampling stopped\n");
  return EXIT_SUCCESS;
}

static int encoder_status(void)
{
  printf("encoder: standalone mode\n");
  printf("encoder: dev=%s default_freq=%dHz\n",
         CONFIG_EXAMPLES_ENCODER_DEVPATH,
         CONFIG_EXAMPLES_ENCODER_DEFAULT_FREQ);
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[])
{
  unsigned int freq_hz = CONFIG_EXAMPLES_ENCODER_DEFAULT_FREQ;

  if (argc < 2)
    {
      encoder_usage();
      return EXIT_FAILURE;
    }

  if (strcmp(argv[1], "start") == 0)
    {
      if (argc > 2)
        {
          freq_hz = (unsigned int)strtoul(argv[2], NULL, 10);
        }

      return encoder_run(freq_hz, false);
    }

  if (strcmp(argv[1], "once") == 0)
    {
      if (argc > 2)
        {
          freq_hz = (unsigned int)strtoul(argv[2], NULL, 10);
        }

      return encoder_run(freq_hz, true);
    }

  if (strcmp(argv[1], "status") == 0)
    {
      return encoder_status();
    }

  encoder_usage();
  return EXIT_FAILURE;
}
