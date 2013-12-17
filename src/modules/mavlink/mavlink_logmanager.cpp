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
 * @file mavlink_logmanager.cpp
 * MAVLink log transfer
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

__BEGIN_DECLS

#include "mavlink_bridge_header.h"
#include "orb_topics.h"
#include "mavlink_logmanager.h"

__END_DECLS

MAVLinkLogManager::MAVLinkLogManager() :
current_index(0)
{
	
}

MAVLinkLogManager::~MAVLinkLogManager()
{
	
}

unsigned MAVLinkLogManager::count_logs()
{
	return 1;
}

bool MAVLinkLogManager::handle_message(const mavlink_message_t *msg)
{
	bool consumed = true;

	switch (msg->msgid) {
		case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
		{
			mavlink_log_request_list_t list;
			mavlink_msg_log_request_list_decode(msg, &list);

			if (1 == 1/*list.target_system == && list.target_component == */) {
				// list.start // First log id (0 for first available)
				// list.end // Last log id (0xffff for last available)

				// Run through all folders on the microSD card
				// and assign a linear index

				unsigned log_count = count_logs();

				for (unsigned i = 0; i < log_count; i++) {
					struct {
						unsigned index;
						unsigned timestamp;
						unsigned size;
					} curr;

					curr.index = 1000;
					curr.timestamp = 10000;
					curr.size = 0;

					// Die-hard send full list
					if (curr.index >= list.start && curr.index < list.end) {
						mavlink_msg_log_entry_send(MAVLINK_COMM_0, curr.index,
							log_count,
							log_count - 1,
							curr.timestamp,
							curr.size);
					}
				}
			}
		}
		break;

		case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
		{
			mavlink_log_request_data_t request;
			mavlink_msg_log_request_data_decode(msg, &request);
		}
		break;

		case MAVLINK_MSG_ID_LOG_ERASE:
		{
			mavlink_log_erase_t erase;
			mavlink_msg_log_erase_decode(msg, &erase);
		}
		break;

		case MAVLINK_MSG_ID_LOG_REQUEST_END:
		{

		}
		break;

		default:
		consumed = false;
		break;

		// LOG_REQUEST_LIST LOG_REQUEST_DATA LOG_DATA LOG_ERASE LOG_REQUEST_END
	}

	return consumed;
}
