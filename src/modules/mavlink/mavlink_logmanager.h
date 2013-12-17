/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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

/**
 * @file mavlink_logmanager.h
 * MAVLink log transfer definitions
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#pragma once

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

__BEGIN_DECLS

#include "mavlink_bridge_header.h"
#include "orb_topics.h"
#include "util.h"
#include "mavlink_logmanager.h"

__END_DECLS

class MAVLinkLogManager
{
public:
	/**
	 * Constructor
	 */
	MAVLinkLogManager();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~MAVLinkLogManager();

	/**
	 * Handle a telemetry message
	 *
	 * @return		true if message was handled / consumed
	 */
	bool		handle_message(const mavlink_message_t *msg);

protected:

	/**
	 * Count the number of logs in the log directory
	 */
	unsigned	count_logs();

private:

	unsigned current_index;		///< Current index in the transmission list

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

};


