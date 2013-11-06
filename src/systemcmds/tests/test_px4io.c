/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include "tests.h"

#include <drivers/drv_gpio.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_pwm_output.h>
#include <systemlib/err.h>

/**
 * @file test_px4io.c
 *
 * Tests to validate PX4IO, needs peripherals / or a jig
 */

int test_px4io(int argc, char *argv[])
{
	int fd = open(PX4IO_DEVICE_PATH, O_RDWR);
	int ret = OK;
	struct rc_input_values rc;			/**< r/c channel data */

	bool rc_updated;

	if (fd < 0) {
		warnx("can't read from PX4 IO. Forgot 'px4io start' command?");
		ret = 1;
		goto errout_with_dev;
	}

	struct pwm_output_values servos;
	servos.values[0] = 1000;
	servos.values[1] = 1100;
	servos.values[2] = 1200;
	servos.values[3] = 1300;
	servos.values[4] = 1400;
	servos.values[5] = 1500;
	servos.values[6] = 1600;
	servos.values[7] = 1700;
	servos.channel_count = 8;

	/* make PWM values effective now */
	ioctl(fd, PWM_SERVO_SET_DISARMED_PWM, (long unsigned int)&servos);

	/* sleep to allow propagation */
	usleep(80000);

	/* read back servo values */
	ioctl(fd, RC_INPUT_GET, (long unsigned int)&rc);

	/* check the PX4IO PWM / RC output */

	// XXX check for mismatch

	for (unsigned i = 0; i < rc.channel_count; i++)
	{
		printf("channel %u: %u", i, rc.values[i]);
	}


	printf("\t PX4IO test successful.\n");

errout_with_dev:

	if (fd != 0)
		close(fd);

	return ret;
}
