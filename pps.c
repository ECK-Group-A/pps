#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <string.h>
#include <sys/timex.h>

#include <pigpio.h>

/*

pps.c
2020-09-18
Public Domain

gcc -o pps pps.c -lpigpio

sudo ./pps

*/

#define PPS_GPIO    4   /* gpio for output pulse */
#define PPS_PULSE  5000   /* pulse length in microseconds */
#define TRIGGER_PULSE  5000   /* pulse length in microseconds */
#define INTERVAL 1000000 /* pulse every second */
#define SLACK  200 /* slack period to correct time */
#define NUM_CAMERAS 2 /* number of cameras used */

static const int cam_gpio[NUM_CAMERAS] = {17, 27};
static const int cam_phase[NUM_CAMERAS] = {0, 180};

static uint32_t *g_slackA;

void callback(int gpio, int level, uint32_t tick)
{
   static int inited = 0, drift = 0, count = 0;

   int i;
   int slack; /* how many microseconds for slack pulse */
   int offby; /* microseconds off from 0 */
   uint32_t stamp_micro, stamp_tick;
   uint32_t pulse_tick, now_tick;
   uint32_t tick1, tick2, tick_diff;
   uint32_t nextPulse, nextPulseTick, delay, fixed;
   struct timespec tp;

   /*
      Seconds boundary has arrived.

      Make several attempts at finding the relationship between the
      system tick and the clock microsecond.

      Do so by bracketing the call to the clock with calls to get
      the system tick.

      Escape the loop early if the difference between the two
      system ticks is zero (can't do any better).
   */

   pulse_tick = rawWaveGetIn(0); /* tick read at pulse start */
   now_tick = gpioTick();        /* just for interest, to get an idea
                                    of scheduling delays */

   tick_diff = 10000000;

   for (i=0; i<10; i++)
   {
      tick1 = gpioTick();

      clock_gettime(CLOCK_REALTIME, &tp);

      tick2 = gpioTick();

      if ((tick2 - tick1) < tick_diff)
      {
         tick_diff = tick2 - tick1;

         stamp_tick  = tick1;

         stamp_micro = ((tp.tv_sec % 1) * 1000000) +
                        ((tp.tv_nsec+500) / 1000);

         if (tick_diff == 0) break;
      }
   }

   if (inited)
   {
      /* correct if early */
      if (stamp_micro > (INTERVAL / 2)) stamp_micro -= INTERVAL;
      offby  = stamp_micro - (stamp_tick - pulse_tick);
      drift += offby/2; /* correct drift, bit of lag */
   }
   else
   {
      offby = 0;
      drift = 0;
   }

   nextPulse = INTERVAL - stamp_micro;
   nextPulseTick = stamp_tick + nextPulse - drift;

   delay = nextPulseTick - pulse_tick;

   fixed = INTERVAL - SLACK;
   slack = delay - fixed;

   if (slack < 0) slack += INTERVAL;
   if (!slack) slack = 1;
   *g_slackA = (slack * 4);

   if (inited)
   {
      printf("%8d %5d %5d %5d %5d\n",
         count++, drift, offby, now_tick - pulse_tick, slack);
   }
   else
   {
      printf("#  count drift offby sched slack\n");
      inited = 1;
   }
}

int main(int argc, char *argv[])
{
   int off;
   int wave_id;
   rawWave_t pps[3];
   rawWaveInfo_t winf;

   if (gpioInitialise()<0) return -1;

   gpioSetAlertFunc(PPS_GPIO, callback);     /* set pps callback */

   gpioSetMode(PPS_GPIO, PI_OUTPUT);

   pps[0].gpioOn  = 0;
   pps[0].gpioOff = (1<<PPS_GPIO);
   pps[0].usDelay = SLACK;
   pps[0].flags   = 0;

   pps[1].gpioOn = (1<<PPS_GPIO);
   pps[1].gpioOff = 0;
   pps[1].usDelay = PPS_PULSE;
   pps[1].flags   = WAVE_FLAG_TICK;    /* read tick at start of pulse */

   pps[2].gpioOn  = 0;
   pps[2].gpioOff = (1<<PPS_GPIO);
   pps[2].usDelay = INTERVAL - (PPS_PULSE + SLACK);
   pps[2].flags   = 0;

   gpioWaveClear();            /* clear all waveforms */

   rawWaveAddGeneric(3, pps); /* add data to waveform */

   for (int cam_idx = 0; cam_idx < NUM_CAMERAS; cam_idx++) {
      gpioSetMode(cam_gpio[cam_idx], PI_OUTPUT);
      rawWave_t camera[22];

      int phase = (1000000 / 10) / 360.0 * cam_phase[cam_idx];

      camera[0].gpioOn  = 0;
      camera[0].gpioOff = (1<<cam_gpio[cam_idx]);
      camera[0].usDelay = SLACK;
      camera[0].flags   = 0;

      camera[1].gpioOn  = 0;
      camera[1].gpioOff = (1<<cam_gpio[cam_idx]);
      camera[1].usDelay = phase;
      camera[1].flags   = 0;

      for (int i = 0; i < 10; i++) {
         camera[2*i+2].gpioOn = (1<<cam_gpio[cam_idx]);
         camera[2*i+2].gpioOff = 0;
         camera[2*i+2].usDelay = TRIGGER_PULSE;
         camera[2*i+2].flags   = 0;

         if (i != 9) {
            camera[2*i+3].gpioOn  = 0;
            camera[2*i+3].gpioOff = (1<<cam_gpio[cam_idx]);
            camera[2*i+3].usDelay = (1000000 / 10) - TRIGGER_PULSE;
            camera[2*i+3].flags   = 0;
         } else {
            camera[2*i+3].gpioOn  = 0;
            camera[2*i+3].gpioOff = (1<<cam_gpio[cam_idx]);
            camera[2*i+3].usDelay = (1000000 / 10) - TRIGGER_PULSE - SLACK - phase;
            camera[2*i+3].flags   = 0;
         }
      }

      printf("%d\n", rawWaveAddGeneric(22, camera) /* add data to waveform */);
   }

   wave_id = gpioWaveCreate(); /* create waveform from added data */

   if (wave_id >= 0)
   {
      gpioWaveTxSend(wave_id, PI_WAVE_MODE_REPEAT);

      winf = rawWaveInfo(wave_id);
      /* get address of slack length */
      g_slackA  = &(rawWaveCBAdr(winf.botCB+2)->length);

      while (1) sleep(1);
   }

   gpioTerminate();
}