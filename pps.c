#include <arpa/inet.h>
#include <netinet/in.h>
#include <pigpio.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/timex.h>
#include <time.h>
#include <unistd.h>


/*

pps.c

gcc -o pps pps.c -lpigpio

sudo ./pps

*/

#define PPS_GPIO 4          /* gpio for output pulse */
#define UDP_TRIGGER_GPIO 30 /* gpio for output UDP trigger signal */
#define PPS_PULSE 10000     /* pulse length in microseconds */
#define TRIGGER_PULSE 200   /* pulse length in microseconds */
#define INTERVAL 1000000    /* pulse every second */
#define SLACK 200           /* slack period to correct time */
#define NUM_CAMERAS 2       /* number of cameras used */

// GPIO 30 is reserved
static const int cam_gpio[NUM_CAMERAS] = {17, 27};
static const int cam_phase[NUM_CAMERAS] = {0, 180};

static uint32_t *g_slackA;

int sock;
time_t timer;
struct tm *tm;

void callback(int gpio, int level, uint32_t tick) {
  static int inited = 0, drift = 0, count = 0;

  int i;
  int slack; /* how many microseconds for slack pulse */
  int offby; /* microseconds off from 0 */
  uint32_t stamp_micro, stamp_tick;
  uint32_t pulse_tick, now_tick;
  uint32_t tick1, tick2, tick_diff;
  uint32_t nextPulse, nextPulseTick, delay, fixed;
  uint32_t secondsToday;
  struct timespec tp;
  

  if (level) {
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

    tick_diff = 10000000;

    for (i = 0; i < 10; i++) {
      tick1 = gpioTick();

      clock_gettime(CLOCK_REALTIME, &tp);

      tick2 = gpioTick();

      if ((tick2 - tick1) < tick_diff) {
        tick_diff = tick2 - tick1;

        stamp_tick = tick1;

        stamp_micro = (tp.tv_nsec + 500) / 1000;

        if (tick_diff == 0) break;
      }
    }

    /* correct if early */
    if (stamp_micro > (INTERVAL / 2)) stamp_micro -= INTERVAL;
    offby = stamp_micro - (stamp_tick - pulse_tick);

    if (inited) {
      drift += offby / 2; /* correct drift, bit of lag */
    } else {
      drift = 0;
    }

    slack = SLACK - offby - drift;

    if (slack < 0) {
      slack += INTERVAL;
    }
    if (!slack) {
      slack = 1;
    }
    *g_slackA = (slack * 4);

    if (inited) {
      printf("%8d %5d %5d %5d\n", count++, drift, offby, slack);

      //setup logger
      char fileName [20];
      char filePath[40];
      char timeBuffer[80];
      struct tm *time = localtime(&tp.tv_sec);
      strftime(fileName, sizeof fileName, "log%d-%m-%Y.txt", localtime(&tp.tv_sec));
      sprintf(filePath, "/home/pi/Raspi_Triggerbox/log/%s", fileName);
      FILE *log = fopen(filePath, "a"); 
      if(log == NULL){
        perror("Error: ");
      }
      else{
        secondsToday = time->tm_hour*3600 + time->tm_min*60 + time->tm_sec;

        fprintf(log, "%lu %5d %5d %5d\n", secondsToday, drift, offby, slack);
        fclose(log);
      }


    } else {
      printf("#  count drift offby slack\n");
      inited = 1;
    }
  }
}

// Connect to udp broadcast socket
int udp_connect(char *host, int port) {
  int sock;
  struct sockaddr_in addr;
  int broadcast = 1;

  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("socket");
    return -1;
  }

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = inet_addr(host);

  // UDP is connectionless, so we don't need to close the socket afterwards!
  // This only specifies a default host endpoint.
  if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("connect");
    return -1;
  }

  return sock;
}

// Calculate NMEA checksum
unsigned char nmea_checksum(char *sentence) {
  unsigned char checksum = 0;
  char *p = sentence;

  while (*p) {
    checksum ^= *p++;
  }

  return checksum;
}

// Send NMEA sentence
int nmea_send(int sock, char *sentence) {
  char buffer[1024];
  int len;

  sprintf(buffer, "$%s*%02X\r\n", sentence, nmea_checksum(sentence));
  len = strlen(buffer);

  return send(sock, buffer, len, 0);
}

void send_nmea_time(int gpio, int level, uint32_t tick) {
  if (level) {
    char nmea[1024];

    time(&timer);
    tm = localtime(&timer);
    sprintf(nmea,
            "GPRMC,%02d%02d%02d,A,0000.000,N,00000.000,E,000.0,000.0,%02d%02d%"
            "02d,000.0,W,S",
            tm->tm_hour, tm->tm_min, tm->tm_sec, tm->tm_mday, tm->tm_mon + 1,
            (tm->tm_year + 1900) % 100);
    nmea_send(sock, nmea);
  }
}

int main(int argc, char *argv[]) {
  int off;
  int wave_id;
  rawWave_t pps[3];
  rawWave_t nmea[3];
  rawWaveInfo_t winf;

  // Connect to UDP socket
  if (sock = udp_connect("1.1.1.1", 10110) < 0) {
    perror("udp_connect");
    return -1;
  }

  if (gpioInitialise() < 0) {
    perror("gpioInitialise");
    return -1;
  }
  

  gpioSetAlertFunc(PPS_GPIO, callback);               /* set pps callback */
  gpioSetAlertFunc(UDP_TRIGGER_GPIO, send_nmea_time); /* set udp callback */

  gpioSetMode(PPS_GPIO, PI_OUTPUT);

  pps[0].gpioOn = 0;
  pps[0].gpioOff = (1 << PPS_GPIO);
  pps[0].usDelay = SLACK;
  pps[0].flags = 0;

  pps[1].gpioOn = (1 << PPS_GPIO);
  pps[1].gpioOff = 0;
  pps[1].usDelay = PPS_PULSE;
  pps[1].flags = WAVE_FLAG_TICK; /* read tick at start of pulse */

  pps[2].gpioOn = 0;
  pps[2].gpioOff = (1 << PPS_GPIO);
  pps[2].usDelay = INTERVAL - (PPS_PULSE + SLACK);
  pps[2].flags = 0;

  gpioWaveClear(); /* clear all waveforms */

  rawWaveAddGeneric(3, pps); /* add data to waveform */

  gpioSetMode(UDP_TRIGGER_GPIO, PI_OUTPUT);

  nmea[0].gpioOn = 0;
  nmea[0].gpioOff = (1 << UDP_TRIGGER_GPIO);
  nmea[0].usDelay = SLACK;
  nmea[0].flags = 0;

  nmea[1].gpioOn = 0;
  nmea[1].gpioOff = (1 << UDP_TRIGGER_GPIO);
  nmea[1].usDelay = PPS_PULSE + 100000;  // 100ms
  nmea[1].flags = 0;

  nmea[2].gpioOn = (1 << UDP_TRIGGER_GPIO);
  nmea[2].gpioOff = 0;
  nmea[2].usDelay = INTERVAL - (SLACK + PPS_PULSE + 100000);
  nmea[2].flags = 0;

  rawWaveAddGeneric(3, nmea); /* add data to waveform */

  for (int cam_idx = 0; cam_idx < NUM_CAMERAS; cam_idx++) {
    gpioSetMode(cam_gpio[cam_idx], PI_OUTPUT);
    rawWave_t camera[22];

    int phase = (1000000 / 10) / 360.0 * cam_phase[cam_idx];

    camera[0].gpioOn = 0;
    camera[0].gpioOff = (1 << cam_gpio[cam_idx]);
    camera[0].usDelay = SLACK;
    camera[0].flags = 0;

    camera[1].gpioOn = 0;
    camera[1].gpioOff = (1 << cam_gpio[cam_idx]);
    camera[1].usDelay = phase;
    camera[1].flags = 0;

    for (int i = 0; i < 10; i++) {
      camera[2 * i + 2].gpioOn = (1 << cam_gpio[cam_idx]);
      camera[2 * i + 2].gpioOff = 0;
      camera[2 * i + 2].usDelay = TRIGGER_PULSE;
      camera[2 * i + 2].flags = 0;

      if (i != 9) {
        camera[2 * i + 3].gpioOn = 0;
        camera[2 * i + 3].gpioOff = (1 << cam_gpio[cam_idx]);
        camera[2 * i + 3].usDelay = (1000000 / 10) - TRIGGER_PULSE;
        camera[2 * i + 3].flags = 0;
      } else {
        camera[2 * i + 3].gpioOn = 0;
        camera[2 * i + 3].gpioOff = (1 << cam_gpio[cam_idx]);
        camera[2 * i + 3].usDelay =
            (1000000 / 10) - TRIGGER_PULSE - SLACK - phase;
        camera[2 * i + 3].flags = 0;
      }
    }
  }

  wave_id = gpioWaveCreate(); /* create waveform from added data */

  if (wave_id >= 0) {
    gpioWaveTxSend(wave_id, PI_WAVE_MODE_REPEAT);

    winf = rawWaveInfo(wave_id);
    /* get address of slack length */
    g_slackA = &(rawWaveCBAdr(winf.botCB + 2)->length);

    while (1) sleep(1);
  }

  gpioTerminate();
}
