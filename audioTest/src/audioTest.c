#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

#include "mraa/pwm.h"

#define MAX_SIZE 64

int chipid = 0;
int pin = 0;
int duty_fp = -1;

static int
mraa_pwm_setup_duty_fp(mraa_pwm_context dev)
{
    char bu[MAX_SIZE];
    snprintf(bu,MAX_SIZE, "/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle", chipid, pin);

    duty_fp = open(bu, O_RDWR);
    if (duty_fp == -1) {
        return 1;
    }
    return 0;
}

static mraa_result_t
mraa_pwm_write_period(mraa_pwm_context dev, int period)
{
    char bu[MAX_SIZE];
    snprintf(bu,MAX_SIZE ,"/sys/class/pwm/pwmchip%d/pwm%d/period", chipid, pin);

    int period_f = open(bu, O_RDWR);
    if (period_f == -1) {
        printf("pwm: Failed to open period for writing\n");
        return MRAA_ERROR_INVALID_RESOURCE;
    }
    char out[MAX_SIZE];
    int length = snprintf(out, MAX_SIZE, "%d", period);
    if (write(period_f, out, length*sizeof(char)) == -1) {
        close(period_f);
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    close(period_f);
    return MRAA_SUCCESS;
}

static mraa_result_t
mraa_pwm_write_duty(mraa_pwm_context dev, int duty)
{
    if (duty_fp == -1) {
        if (mraa_pwm_setup_duty_fp(dev) == 1) {
            return MRAA_ERROR_INVALID_HANDLE;
        }
    }
    char bu[64];
    int length = sprintf(bu, "%d", duty);
    if (write(duty_fp, bu, length * sizeof(char)) == -1)
        return MRAA_ERROR_INVALID_RESOURCE;
    return MRAA_SUCCESS;
}

#include "sample11025-u8.h"
const int SAMPLERATE = 11025;

void playSoundInfinitely()
{
	mraa_pwm_context pwm;
	pwm = mraa_pwm_init(3);
	if (pwm == NULL) {
		return;
	}
	int periodNSec = 10000;
	int periodUSec = 10;
	mraa_pwm_period_us(pwm, periodUSec);
	//mraa_pwm_write_period(pwm, periodNSec);
	mraa_pwm_enable(pwm, 1);

	double startTime = 0;
	while (1)
	{
		struct timespec t;
		clock_gettime(CLOCK_REALTIME, &t);
		double time = (double)t.tv_sec + (double)t.tv_nsec / 1.0e9;

		if (startTime == 0)
			startTime = time;

		double sampleIndex = (time - startTime) * SAMPLERATE;
		float value = audioData[(int)sampleIndex % sizeof(audioData)] / 255.0f;
		//value = sin(time*hertz * 6.28) * 0.5 + 0.5;

		static int lastPlayedSample = 0;
		if (lastPlayedSample != (int)sampleIndex)
		{
			lastPlayedSample = (int)sampleIndex;

			static int samplesPlayed = 0;
			static double lastTime = 0;
			static double samplesTooLate = 0.0;
			static double maxSampleTooLate = 0.0;
			static int numLateSamples = 0;

			samplesPlayed++;
			// Timing error measurement: How far into the sample are we already?
			double sampleTooLate = sampleIndex - (int)sampleIndex;
			samplesTooLate += sampleTooLate;
			if (sampleTooLate > maxSampleTooLate)
				maxSampleTooLate = sampleTooLate;
			if (sampleTooLate > 0.1)
				numLateSamples++;

			if (time - lastTime > 10.0)
			{
				printf("%f samples per second, timing precision: avg %f, max %f, late %f%%\n",
						samplesPlayed / (time - lastTime),
						samplesTooLate / samplesPlayed, maxSampleTooLate,
						100.0 * numLateSamples / samplesPlayed);
				lastTime = time;
				samplesPlayed = 0;
				samplesTooLate = 0.0;
				maxSampleTooLate = 0.0;
				numLateSamples = 0;
			}

			//mraa_pwm_write(pwm, value);
			//mraa_pwm_pulsewidth_us(pwm, (int) (value * periodUSec));
			mraa_pwm_write_duty(pwm, (int) (value * periodNSec));

			// usleep sleeps to long, need to investigate kernel scheduler (maybe SCHED_FIFO?)
			//usleep(1000000 / SAMPLERATE);
			//usleep(1);
		}
	}
}

void* audioThreadMain(void* arg)
{
	playSoundInfinitely();
	return 0;
}

volatile int dummy = 0;

int main(int argc, char **argv)
{
#if 1 // USE_THREAD
	pthread_t audioThread = 0;
	int err = pthread_create(&audioThread, NULL, &audioThreadMain, NULL);
	if (err != 0)
		printf("Can't create thread: %s\n", strerror(err));

	for (;;)
	{
		// Simulate some load.
		const int s = (int)(10000000.0 * rand() / RAND_MAX);
		void* p = malloc(s);
		memset(p, 23, s);
		free(p);
		dummy += dummy * dummy;
	}
#else
	playSoundInfinitely();
#endif
	return 0;
}
