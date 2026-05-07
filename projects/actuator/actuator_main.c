/****************************************************************************
 * apps/examples/actuator/actuator_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Drives an Actuonix L12-P linear actuator (via Actuonix Ext-R board) in a
 * continuous extend/retract loop and prints position and current feedback.
 *
 * Wiring:
 *   Ext-R X1 Pin 3 (S, white)  → PA6  (TIM3 CH1, /dev/pwm1)
 *   Ext-R X1 Pin 1 (GND)       → GND
 *   Ext-R X1 Pin 2 (+)         → external 6-12 V supply
 *   Ext-R X3 F1 (position)     → PA0  (ADC1 CH0, /dev/adc0)
 *   Ext-R X3 F2 (current)      → PC0  (ADC1 CH10, /dev/adc0)
 *
 * Usage:
 *   actuator [step_ms [steps]]
 *     step_ms  – delay between PWM steps in milliseconds (default 40)
 *     steps    – number of steps per full sweep              (default 50)
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include <sys/ioctl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <fixedmath.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EXAMPLES_ACTUATOR_PWM_DEVPATH
#  define CONFIG_EXAMPLES_ACTUATOR_PWM_DEVPATH  "/dev/pwm1"
#endif

#ifndef CONFIG_EXAMPLES_ACTUATOR_ADC_DEVPATH
#  define CONFIG_EXAMPLES_ACTUATOR_ADC_DEVPATH  "/dev/adc0"
#endif

/* Ext-R RC servo signal: 50 Hz, 1000–2000 µs pulse width */

#define SERVO_FREQ_HZ   50u
#define PULSE_MIN_US    1000u     /* full retract */
#define PULSE_MAX_US    2000u     /* full extend  */
#define PERIOD_US       (1000000u / SERVO_FREQ_HZ)

#ifndef CONFIG_EXAMPLES_ACTUATOR_SWEEP_STEPS
#  define CONFIG_EXAMPLES_ACTUATOR_SWEEP_STEPS  50
#endif

#ifndef CONFIG_EXAMPLES_ACTUATOR_STEP_MS
#  define CONFIG_EXAMPLES_ACTUATOR_STEP_MS      40
#endif

/* ADC1 channel assignments (/dev/adc0, stm32_adc.c) */

#define ADC_CH_POSITION   0     /* F1 → PA0:  0 V = extended, 3.3 V = retracted */
#define ADC_CH_CURRENT    10    /* F2 → PC0:  0 V = 0 A,     3.3 V = 1 A        */
#define ADC_NCHANNELS     3     /* total channels configured on /dev/adc0         */
#define ADC_FULL_SCALE    4095  /* 12-bit ADC                                     */

/* Convert microsecond pulse width to ub16_t duty (1.0 == 0x10000 == 65536). */

#define PULSE_TO_DUTY(us) \
  ((ub16_t)(((uint32_t)(us) * 65536UL) / PERIOD_US))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile sig_atomic_t g_running = 1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void actuator_sighandler(int signo)
{
  (void)signo;
  g_running = 0;
}

static int pwm_set_pulse(int fd, uint32_t pulse_us)
{
  struct pwm_info_s info;

  memset(&info, 0, sizeof(info));
  info.frequency = SERVO_FREQ_HZ;

#ifdef CONFIG_PWM_MULTICHAN
  info.channels[0].channel = 1;
  info.channels[0].duty    = PULSE_TO_DUTY(pulse_us);
#else
  info.duty = PULSE_TO_DUTY(pulse_us);
#endif

  return ioctl(fd, PWMIOC_SETCHARACTERISTICS,
               (unsigned long)((uintptr_t)&info));
}

/* Reads ADC1 feedback; returns true if at least position was obtained. */

static bool adc_read_feedback(int fd, float *pos_pct, float *curr_ma)
{
  struct adc_msg_s msgs[ADC_NCHANNELS];
  ssize_t  nbytes;
  int      nsamples;
  int      i;
  bool     got_pos  = false;
  bool     got_curr = false;

  ioctl(fd, ANIOC_TRIGGER, 0);

  nbytes = read(fd, msgs, sizeof(msgs));
  if (nbytes <= 0)
    {
      return false;
    }

  nsamples = (int)(nbytes / sizeof(struct adc_msg_s));

  for (i = 0; i < nsamples; i++)
    {
      if (msgs[i].am_channel == ADC_CH_POSITION)
        {
          /* Invert: 3.3 V (raw 4095) = fully retracted = 0 % extension */

          *pos_pct = (1.0f - (float)msgs[i].am_data / ADC_FULL_SCALE)
                     * 100.0f;
          got_pos  = true;
        }
      else if (msgs[i].am_channel == ADC_CH_CURRENT)
        {
          /* 3.3 V = 1 A → scale raw to mA */

          *curr_ma  = ((float)msgs[i].am_data / ADC_FULL_SCALE) * 1000.0f;
          got_curr  = true;
        }
    }

  (void)got_curr;
  return got_pos;
}

static void print_header(void)
{
  printf("\n%-8s  %9s  %10s  %12s\n",
         "Dir", "Pulse(us)", "Pos(%)", "Current(mA)");
  printf("%-8s  %9s  %10s  %12s\n",
         "-------", "---------", "----------", "------------");
}

static int actuator_run(unsigned int step_ms, unsigned int steps)
{
  int      pwm_fd;
  int      adc_fd;
  float    pos_pct  = 0.0f;
  float    curr_ma  = 0.0f;
  uint32_t step_us;
  uint32_t pulse_us;
  unsigned int i;
  unsigned int line_count = 0;

  pwm_fd = open(CONFIG_EXAMPLES_ACTUATOR_PWM_DEVPATH, O_RDONLY);
  if (pwm_fd < 0)
    {
      fprintf(stderr, "actuator: open %s failed: %d\n",
              CONFIG_EXAMPLES_ACTUATOR_PWM_DEVPATH, errno);
      return EXIT_FAILURE;
    }

  adc_fd = open(CONFIG_EXAMPLES_ACTUATOR_ADC_DEVPATH, O_RDONLY);
  if (adc_fd < 0)
    {
      fprintf(stderr, "actuator: open %s failed: %d\n",
              CONFIG_EXAMPLES_ACTUATOR_ADC_DEVPATH, errno);
      close(pwm_fd);
      return EXIT_FAILURE;
    }

  step_us = (PULSE_MAX_US - PULSE_MIN_US) / steps;

  /* Park at full retract before starting */

  pwm_set_pulse(pwm_fd, PULSE_MIN_US);
  ioctl(pwm_fd, PWMIOC_START, 0);
  usleep(500000);

  signal(SIGINT, actuator_sighandler);
  g_running = 1;

  printf("actuator: pwm=%s  adc=%s  step=%u ms  steps=%u\n",
         CONFIG_EXAMPLES_ACTUATOR_PWM_DEVPATH,
         CONFIG_EXAMPLES_ACTUATOR_ADC_DEVPATH,
         step_ms, steps);
  printf("actuator: Ctrl+C to stop\n");

  while (g_running)
    {
      /* Extend: PULSE_MIN_US → PULSE_MAX_US */

      for (i = 0; i <= steps && g_running; i++)
        {
          if ((line_count % 20) == 0)
            {
              print_header();
            }

          pulse_us = PULSE_MIN_US + i * ste p_us;
          pwm_set_pulse(pwm_fd, pulse_us);
          adc_read_feedback(adc_fd, &pos_pct, &curr_ma);

          printf("%-8s  %9" PRIu32 "  %10.1f  %12.1f\n",
                 "EXTEND", pulse_us, pos_pct, curr_ma);
          line_count++;
          usleep((unsigned int)step_ms * 1000u);
        }

      usleep(1000000u);  /* hold 1 s at full extend */

      /* Retract: PULSE_MAX_US → PULSE_MIN_US */

      for (i = 0; i <= steps && g_running; i++)
        {
          if ((line_count % 20) == 0)
            {
              print_header();
            }

          pulse_us = PULSE_MAX_US - i * step_us;
          pwm_set_pulse(pwm_fd, pulse_us);
          adc_read_feedback(adc_fd, &pos_pct, &curr_ma);

          printf("%-8s  %9" PRIu32 "  %10.1f  %12.1f\n",
                 "RETRACT", pulse_us, pos_pct, curr_ma);
          line_count++;
          usleep((unsigned int)step_ms * 1000u);
        }

      usleep(1000000u);  /* hold 1 s at full retract */
    }

  ioctl(pwm_fd, PWMIOC_STOP, 0);
  printf("\nactuator: stopped\n");

  close(adc_fd);
  close(pwm_fd);
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[])
{
  unsigned int step_ms = CONFIG_EXAMPLES_ACTUATOR_STEP_MS;
  unsigned int steps   = CONFIG_EXAMPLES_ACTUATOR_SWEEP_STEPS;

  if (argc > 1)
    {
      step_ms = (unsigned int)strtoul(argv[1], NULL, 10);
    }

  if (argc > 2)
    {
      steps = (unsigned int)strtoul(argv[2], NULL, 10);
      if (steps == 0)
        {
          steps = 1;
        }
    }

  return actuator_run(step_ms, steps);
}
