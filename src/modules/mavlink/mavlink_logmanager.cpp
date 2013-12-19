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
#include <stdio.h>
#include <sys/stat.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

__BEGIN_DECLS

#include "mavlink_bridge_header.h"
#include "orb_topics.h"
#include "mavlink_logmanager.h"

__END_DECLS


// XXX magic number
#define MAX_NO_LOGFOLDER 999
#define MAX_NO_LOGFILE 999
#define mountpoint "/fs/microsd"

MAVLinkLogManager::MAVLinkLogManager() :
current_index(0)
{
	
}

MAVLinkLogManager::~MAVLinkLogManager()
{
	
}

unsigned MAVLinkLogManager::log_foreach(unsigned int (MAVLinkLogManager::* func)(void *arg, const struct stat *st, const char* path), void *arg)
{
	unsigned count = 0;

	/* make folder on sdcard */
	unsigned folder_number = 1; // start with folder sess001
	char folder_path[64];

	/* look for the next folder that does not exist */
	while (folder_number <= MAX_NO_LOGFOLDER) {
		unsigned file_number = 1; // start with file log001

		sprintf(folder_path, "%s/sess%03u", mountpoint, folder_number);

		// Abort on the first file name miss.
		// XXX We need to get a real listing here.
		struct stat buffer;
		if (stat(folder_path, &buffer) != 0)
			break;

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {

			/* set up folder path: e.g. /fs/microsd/sess001 */
			sprintf(folder_path, "%s/sess%03u/log%03u.bin", mountpoint, folder_number, file_number);

			// XXX we need to use a real listing here

			if (stat(folder_path, &buffer) == 0)
			{
				count++;

				// Apparently the file exists run the function on it
				if (func)
					(this->*func)(arg, &buffer, folder_path);

			} else {

				// File names are linear, if we missed one there are no more files
				break;
			}

			file_number++;
		}

		folder_number++;
	}

	if (folder_number >= MAX_NO_LOGFOLDER) {
		/* we should not end up here, either we have more than MAX_NO_LOGFOLDER on the SD card, or another problem */
		warnx("all %d possible folders exist already.", MAX_NO_LOGFOLDER);
		return -1;
	}

	return count;
}

unsigned MAVLinkLogManager::count_logs()
{
	return log_foreach(NULL, NULL);
}

unsigned MAVLinkLogManager::send_logentry(void *loginfo, const struct stat *buffer, const char* path)
{

	MAVLinkLogManager::log_info *info = (MAVLinkLogManager::log_info *)loginfo;

	mavlink_msg_log_entry_send(MAVLINK_COMM_0, info->index,
							info->log_count,
							info->log_count - 1,
							info->timestamp,
							info->size);

	return 0;
}

unsigned MAVLinkLogManager::send_list(MAVLinkLogManager::log_info *info)
{
		mavlink_msg_log_entry_send(MAVLINK_COMM_0, info->index,
							info->log_count,
							info->log_count - 1,
							info->timestamp,
							info->size);
	
	return log_foreach(&MAVLinkLogManager::send_logentry, info);
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

				MAVLinkLogManager::log_info info;
				info.log_count = count_logs();
				info.log_min = list.start;
				info.log_max = list.end;

				int ret = send_list(&info);

				warnx("sent %d log entries", ret);
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
