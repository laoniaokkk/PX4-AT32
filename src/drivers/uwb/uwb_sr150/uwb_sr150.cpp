/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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


/* This is a driver for the NXP SR150 UWB Chip on MK UWB Shield 2
 *	This Driver handles the Communication to the UWB Board.
 *	For Information about HW and SW contact Mobile Knowledge:
 *	https://www.themobileknowledge.com
 * */

#include "uwb_sr150.h"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/cli.h>
#include <errno.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <ctype.h>
#include <string.h>

// Timeout between bytes. If there is more time than this between bytes, then this driver assumes
// that it is the boundary between messages.
// See uwb_sr150::run() for more detailed explanation.
#define BYTE_TIMEOUT_US 5000

// Amount of time to wait for a new message. If more time than this passes between messages, then this
// driver assumes that the UWB_SR150 module is disconnected.
// (Right now it does not do anything about this)
#define MESSAGE_TIMEOUT_S 10  //wait 10 seconds.
#define MESSAGE_TIMEOUT_US 1

// The default baudrate of the uwb_sr150 module before configuration
#define DEFAULT_BAUD B115200

const uint8_t CMD_RANGING_STOP[UWB_CMD_LEN ] = {UWB_CMD, 0x00, 0x02, UWB_DRONE_CTL, UWB_CMD_STOP};
const uint8_t CMD_RANGING_START[UWB_CMD_LEN ] = {UWB_CMD, 0x00, 0x02, UWB_DRONE_CTL, UWB_CMD_START};
const uint8_t CMD_APP_START[UWB_CMD_LEN ] = {0x01, 0x00, 0x02, UWB_APP_START, UWB_PRECNAV_APP};
const uint8_t CMD_APP_STOP[0x04 ] = {0x01, 0x00, 0x01, UWB_APP_STOP};

extern "C" __EXPORT int uwb_sr150_main(int argc, char *argv[]);

UWB_SR150::UWB_SR150(const char *device_name, speed_t baudrate, bool uwb_pos_debug):
	ModuleParams(nullptr),
	_read_count_perf(perf_alloc(PC_COUNT, "uwb_sr150_count")),
	_read_err_perf(perf_alloc(PC_COUNT, "uwb_sr150_err"))
{
	_uwb_pos_debug = uwb_pos_debug;
	// start serial port
	_uart = open(device_name, O_RDWR | O_NOCTTY);

	if (_uart < 0) { err(1, "could not open %s", device_name); }

	int ret = 0;
	struct termios uart_config {};
	ret = tcgetattr(_uart, &uart_config);

	if (ret < 0) { err(1, "failed to get attr"); }

	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	ret = cfsetispeed(&uart_config, baudrate);

	if (ret < 0) { err(1, "failed to set input speed"); }

	ret = cfsetospeed(&uart_config, baudrate);

	if (ret < 0) { err(1, "failed to set output speed"); }

	ret = tcsetattr(_uart, TCSANOW, &uart_config);

	if (ret < 0) { err(1, "failed to set attr"); }
}

UWB_SR150::~UWB_SR150()
{
	printf("UWB: Ranging Stopped\t\n");
	int written = write(_uart, &CMD_APP_STOP, sizeof(CMD_APP_STOP));

	if (written < (int) sizeof(CMD_APP_STOP)) {
		PX4_ERR("Only wrote %d bytes out of %d.", written, (int) sizeof(CMD_APP_STOP));
	}

	perf_free(_read_err_perf);
	perf_free(_read_count_perf);

	close(_uart);
}

void UWB_SR150::run()
{
	// Subscribe to parameter_update message
	parameters_update();

	//TODO replace with BLE grid configuration
	grid_info_read(&_uwb_grid_info.target_pos);
	_uwb_grid_info.num_anchors = 4;
	_uwb_grid_info.initator_time = hrt_absolute_time();
	_uwb_grid_info.mac_mode = 0x00;

	/* Grid Info Message*/
	_uwb_grid.timestamp = hrt_absolute_time();
	_uwb_grid.initator_time = _uwb_grid_info.initator_time;
	_uwb_grid.num_anchors = _uwb_grid_info.num_anchors;

	memcpy(&_uwb_grid.target_pos, &_uwb_grid_info.target_pos, sizeof(position_t) * 5); //write Grid positions

	_uwb_grid_pub.publish(_uwb_grid);

	/* Ranging  Command */
	int written = write(_uart, CMD_RANGING_START, UWB_CMD_LEN);

	if (written <  UWB_CMD_LEN) {
		PX4_ERR("Only wrote %d bytes out of %d.", written, (int) sizeof(uint8_t) *  UWB_CMD_LEN);
	}

	while (!should_exit()) {
		written = UWB_SR150::distance(); //evaluate Ranging Messages until Stop
	}

	if (!written) { printf("ERROR: Distance Failed"); }

	// Automatic Stop. This should not be reachable
	written = write(_uart, &CMD_RANGING_STOP, UWB_CMD_LEN);

	if (written < (int) sizeof(CMD_RANGING_STOP)) {
		PX4_ERR("Only wrote %d bytes out of %d.", written, (int) sizeof(CMD_RANGING_STOP));
	}
}

void UWB_SR150::grid_info_read(position_t *grid)
{
	//place holder, until UWB initiator can respond with Grid info
	/*	This Reads the position of each Anchor in the Grid.
		Right now the Grid configuration is saved on the SD card.
		In the future, we would like, that the Initiator responds with the Grid Information (Including Position). */
	PX4_INFO("Reading UWB GRID from SD... \t\n");
	FILE *file;
	file = fopen(UWB_GRID_CONFIG, "r");

	int  bread = 0;

	for (int i = 0; i < 5; i++) {
		bread += fscanf(file, "%hd,%hd,%hd\n", &grid[i].x, &grid[i].y, &grid[i].z);
	}

	if (bread == 5 * 3) {
		PX4_INFO("GRID INFO READ! bytes read: %d \t\n", bread);

	} else { //use UUID from Grid survey
		PX4_INFO("GRID INFO Missing! bytes read: %d \t\n Using standrd Grid \t\n", bread);
		position_t grid_setup[5] = {{0x0, 0x0, 0x0}, {(int16_t)0xff68, 0x0, 0x0a}, {0x99, 0x0, 0x0a}, {0x0, 0x96, 0x64}, {0x0, (int16_t)0xff6a, 0x63}};
		memcpy(grid, &grid_setup, sizeof(grid_setup));
		PX4_INFO("Insert \"uwb_grid_config.csv\" containing gridinfo in cm at \"/fs/microsd/etc/\".\t\n");
		PX4_INFO("n + 1 Anchor Positions starting with Landing Target. Int16 Format:  \"x,y,z\" \t\n");
	}

	fclose(file);
}

int UWB_SR150::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command.");
}

int UWB_SR150::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("uwb", "driver");
	PRINT_MODULE_DESCRIPTION(R"DESC_STR(
### Description

Driver for NXP UWB_SR150 UWB positioning system. This driver publishes a `uwb_distance` message
whenever the UWB_SR150 has a position measurement available.

### Example

Start the driver with a given device:

$ uwb start -d /dev/ttyS2
	)DESC_STR");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Name of device for serial communication with UWB", false);
	PRINT_MODULE_USAGE_PARAM_STRING('b', nullptr, "<int>", "Baudrate for serial communication", false);
	PRINT_MODULE_USAGE_PARAM_STRING('p', nullptr, "<int>", "Position Debug: displays errors in Multilateration", false);
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	return 0;
}

int UWB_SR150::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd(
			      "uwb_driver",
			      SCHED_DEFAULT,
			      SCHED_PRIORITY_DEFAULT,
			      2048,
			      &run_trampoline,
			      argv
		      );

	if (task_id < 0) {
		return -errno;

	} else {
		_task_id = task_id;
		return 0;
	}
}

UWB_SR150 *UWB_SR150::instantiate(int argc, char *argv[])
{
	int ch;
	int option_index = 1;
	const char *option_arg;
	const char *device_name = nullptr;
	bool error_flag = false;
	int baudrate = 0;
	bool uwb_pos_debug = false; // Display UWB position calculation debug Messages

	while ((ch = px4_getopt(argc, argv, "d:b:p", &option_index, &option_arg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = option_arg;
			break;

		case 'b':
			px4_get_parameter_value(option_arg, baudrate);
			break;

		case 'p':

			uwb_pos_debug = true;
			break;

		default:
			PX4_WARN("Unrecognized flag: %c", ch);
			error_flag = true;
			break;
		}
	}

	if (!error_flag && device_name == nullptr) {
		print_usage("Device name not provided. Using default Device: TEL1:/dev/ttyS4 \n");
		device_name = "TEL2";
		error_flag = true;
	}

	if (!error_flag && baudrate == 0) {
		printf("Baudrate not provided. Using default Baud: 115200 \n");
		baudrate = B115200;
	}

	if (!error_flag && uwb_pos_debug == true) {
		printf("UWB Position algorithm Debugging \n");
	}

	if (error_flag) {
		PX4_WARN("Failed to start UWB driver. \n");
		return nullptr;

	} else {
		PX4_INFO("Constructing UWB_SR150. Device: %s", device_name);
		return new UWB_SR150(device_name, baudrate, uwb_pos_debug);
	}
}

int UWB_SR150::distance()
{
	_uwb_init_offset_v3f = matrix::Vector3f(_uwb_init_off_x.get(), _uwb_init_off_y.get(),
						_uwb_init_off_z.get()); //set offset at the start
	uint8_t *buffer = (uint8_t *) &_distance_result_msg;

	FD_ZERO(&_uart_set);
	FD_SET(_uart, &_uart_set);
	_uart_timeout.tv_sec = MESSAGE_TIMEOUT_S ;
	_uart_timeout.tv_usec = MESSAGE_TIMEOUT_US;

	size_t buffer_location = 0;
	// There is a atleast 2000 clock cycles between 2 msg (20000/80mhz = 200uS)
	// Messages are only delimited by time. There is a chance that this driver starts up in the middle
	// of a message, with no way to know this other than time. There is also always the possibility of
	// transmission errors causing a dropped byte.
	// Here is the process for dealing with that:
	//  - Wait up to 1 second to start receiving a message
	//  - Once receiving a message, keep going until EITHER:
	//    - There is too large of a gap between bytes (Currently set to 5ms).
	//      This means the message is incomplete. Throw it out and start over.
	//    - 46 bytes are received (the size of the whole message).

	while (buffer_location < sizeof(_distance_result_msg)
	       && select(_uart + 1, &_uart_set, nullptr, nullptr, &_uart_timeout) > 0) {

		int bytes_read = read(_uart, &buffer[buffer_location], sizeof(_distance_result_msg) - buffer_location);

		if (bytes_read > 0) {
			buffer_location += bytes_read;

		} else {
			break;
		}

		FD_ZERO(&_uart_set);
		FD_SET(_uart, &_uart_set);
		_uart_timeout.tv_sec = 0;
		// Setting this timeout too high (> 37ms) will cause problems because the next message will start
		//  coming in, and overlap with the current message.
		// Setting this timeout too low (< 1ms) will cause problems because there is some delay between
		//  the individual bytes of a message, and a too-short timeout will cause the message to be truncated.
		// The current value of 5ms was found experimentally to never cut off a message prematurely.
		// Strictly speaking, there are no downsides to setting this timeout as high as possible (Just under 37ms),
		// because if this process is waiting, it means that the last message was incomplete, so there is no current
		// data waiting to be published. But we would rather set this timeout lower in case the UWB_SR150 board is
		// updated to publish data faster.
		_uart_timeout.tv_usec = BYTE_TIMEOUT_US;
	}

	perf_count(_read_count_perf);

	// All of the following criteria must be met for the message to be acceptable:
	//  - Size of message == sizeof(distance_msg_t) (51 bytes)
	//  - status == 0x00
	//  - Values of all 3 position measurements are reasonable
	//      (If one or more anchors is missed, then position might be an unreasonably large number.)
	bool ok = (buffer_location == sizeof(distance_msg_t) && _distance_result_msg.stop == 0x1b); //||
	//(buffer_location == sizeof(grid_msg_t) && _distance_result_msg.stop == 0x1b)
	//);

	if (ok) {

		/* Ranging Message*/
		_uwb_distance.timestamp = hrt_absolute_time();
		_uwb_distance.time_uwb_ms = _distance_result_msg.time_uwb_ms;
		_uwb_distance.counter = _distance_result_msg.seq_ctr;
		_uwb_distance.sessionid = _distance_result_msg.sessionId;
		_uwb_distance.time_offset = _distance_result_msg.range_interval;

		for (int i = 0; i < 4; i++) {
			_uwb_distance.anchor_distance[i] = _distance_result_msg.measurements[i].distance;
			_uwb_distance.nlos[i] = _distance_result_msg.measurements[i].nLos;
			_uwb_distance.aoafirst[i] = float(_distance_result_msg.measurements[i].aoaFirst) /
						    128; // Angle of Arrival has Format Q9.7; dividing by 2^7 results in the correct value
		}

		// Algorithm goes here
		UWB_POS_ERROR_CODES UWB_POS_ERROR = UWB_SR150::localization();

		_uwb_distance.status = UWB_POS_ERROR;

		if (UWB_OK == UWB_POS_ERROR) {

			_uwb_distance.position[0] = _rel_pos(0);
			_uwb_distance.position[1] = _rel_pos(1);
			_uwb_distance.position[2] = _rel_pos(2);

		} else {
			//only print the error if debug is enabled
			if (_uwb_pos_debug) {
				switch (UWB_POS_ERROR) { //UWB POSITION ALGORItHM Errors
				case UWB_ANC_BELOW_THREE:
					PX4_INFO("UWB not enough anchors for doing localization");
					break;

				case UWB_LIN_DEP_FOR_THREE:
					PX4_INFO("UWB localization: linear dependent with 3 Anchors");
					break;

				case UWB_ANC_ON_ONE_LEVEL:
					PX4_INFO("UWB localization: Anchors are on a X,Y Plane and there are not enought Anchors");
					break;

				case UWB_LIN_DEP_FOR_FOUR:
					PX4_INFO("UWB localization: linear dependent with four or more Anchors");
					break;

				case UWB_RANK_ZERO:
					PX4_INFO("UWB localization: rank is zero");
					break;

				default:
					PX4_INFO("UWB localization: Unknown failure in Position Algorithm");
					break;
				}
			}
		}

		_uwb_distance_pub.publish(_uwb_distance);

	} else {
		//PX4_ERR("Read %d bytes instead of %d.", (int) buffer_location, (int) sizeof(distance_msg_t));
		perf_count(_read_err_perf);

		if (buffer_location == 0) {
			PX4_WARN("UWB module is not responding.");

			//TODO add retry Ranging Start Message. Sometimes the UWB devices Crashes. (Check Power)
		}
	}

	return 1;
}

UWB_POS_ERROR_CODES UWB_SR150::localization()
{
// 			WIP
	/******************************************************
	 ****************** 3D Localization *******************
	 *****************************************************/

	/*!@brief: This function calculates the 3D position of the initiator from the anchor distances and positions using least squared errors.
	 *	 	   The function expects more than 4 anchors. The used equation system looks like follows:\n
	 \verbatim
	  		    -					-
	  		   | M_11	M_12	M_13 |	 x	  b[0]
	  		   | M_12	M_22	M_23 | * y	= b[1]
	  		   | M_23	M_13	M_33 |	 z	  b[2]
	  		    -					-
	 \endverbatim
	 * @param distances_cm_in_pt: 			Pointer to array that contains the distances to the anchors in cm (including invalid results)
	 * @param no_distances: 				Number of valid distances in distance array (it's not the size of the array)
	 * @param anchor_pos: 	Pointer to array that contains anchor positions in cm (including positions related to invalid results)
	 * @param no_anc_positions: 			Number of valid anchor positions in the position array (it's not the size of the array)
	 * @param position_result_pt: 			Pointer toposition. position_t variable that holds the result of this calculation
	 * @return: The function returns a status code. */

	/* 		Algorithm used:
	 *		Linear Least Sqaures to solve Multilateration
	 * 		with a Special case if there are only 3 Anchors.
	 * 		Output is the Coordinates of the Initiator in relation to Anchor 0 in NEU (North-East-Up) Framing
	 * 		In cm
	 */

	/* Resulting Position Vector*/
	int64_t x_pos = 0;
	int64_t y_pos = 0;
	int64_t z_pos = 0;
	/* Matrix components (3*3 Matrix resulting from least square error method) [cm^2] */
	int64_t M_11 = 0;
	int64_t M_12 = 0;																						// = M_21
	int64_t M_13 = 0;																						// = M_31
	int64_t M_22 = 0;
	int64_t M_23 = 0;																						// = M_23
	int64_t M_33 = 0;

	/* Vector components (3*1 Vector resulting from least square error method) [cm^3] */
	int64_t b[3] = {0};

	/* Miscellaneous variables */
	int64_t temp = 0;
	int64_t temp2 = 0;
	int64_t nominator = 0;
	int64_t denominator = 0;
	bool	anchors_on_x_y_plane = true;																		// Is true, if all anchors are on the same height => x-y-plane
	bool	lin_dep = true;																						// All vectors are linear dependent, if this variable is true
	uint8_t ind_y_indi =
		0;	//numberr of independet vectors																					// First anchor index, for which the second row entry of the matrix [(x_1 - x_0) (x_2 - x_0) ... ; (y_1 - x_0) (y_2 - x_0) ...] is non-zero => linear independent

	/* Arrays for used distances and anchor positions (without rejected ones) */
	uint8_t 	no_distances = _uwb_grid_info.num_anchors;
	uint32_t 	distances_cm[no_distances];
	position_t 	anchor_pos[no_distances]; //position in CM
	uint8_t		no_valid_distances = 0;

	/* Reject invalid distances (including related anchor position) */
	for (int i = 0; i < no_distances; i++) {
		if (_distance_result_msg.measurements[i].distance != 0xFFFFu) {
			//excludes any distance that is 0xFFFFU (int16 Maximum Value)
			distances_cm[no_valid_distances] 		= _distance_result_msg.measurements[i].distance;
			anchor_pos[no_valid_distances] 	= _uwb_grid_info.anchor_pos[i];
			no_valid_distances++;

		}
	}

	/* Check, if there are enough valid results for doing the localization at all */
	if (no_valid_distances < 3) {
		return UWB_ANC_BELOW_THREE;
	}

	/* Check, if anchors are on the same x-y plane */
	for (int i = 1; i < no_valid_distances; i++) {
		if (anchor_pos[i].z != anchor_pos[0].z) {
			anchors_on_x_y_plane = false;
			break;
		}
	}

	/**** Check, if there are enough linear independent anchor positions ****/

	/* Check, if the matrix |(x_1 - x_0) (x_2 - x_0) ... | has rank 2
	 * 			|(y_1 - y_0) (y_2 - y_0) ... | 				*/

	for (ind_y_indi = 2; ((ind_y_indi < no_valid_distances) && (lin_dep == true)); ind_y_indi++) {
		temp = ((int64_t)anchor_pos[ind_y_indi].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[1].x -
				(int64_t)anchor_pos[0].x);
		temp2 = ((int64_t)anchor_pos[1].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[ind_y_indi].x -
				(int64_t)anchor_pos[0].x);

		if ((temp - temp2) != 0) {
			lin_dep = false;
			break;
		}
	}

	/* Leave function, if rank is below 2 */
	if (lin_dep == true) {
		return UWB_LIN_DEP_FOR_THREE;
	}

	/* If the anchors are not on the same plane, three vectors must be independent => check */
	if (!anchors_on_x_y_plane) {
		/* Check, if there are enough valid results for doing the localization */
		if (no_valid_distances < 4) {
			return UWB_ANC_ON_ONE_LEVEL;
		}

		/* Check, if the matrix |(x_1 - x_0) (x_2 - x_0) (x_3 - x_0) ... | has rank 3 (Rank y, y already checked)
		 * 			|(y_1 - y_0) (y_2 - y_0) (y_3 - y_0) ... |
		 * 			|(z_1 - z_0) (z_2 - z_0) (z_3 - z_0) ... |											*/
		lin_dep = true;

		for (int i = 2; ((i < no_valid_distances) && (lin_dep == true)); i++) {
			if (i != ind_y_indi) {
				/* (x_1 - x_0)*[(y_2 - y_0)(z_n - z_0) - (y_n - y_0)(z_2 - z_0)] */
				temp 	= ((int64_t)anchor_pos[ind_y_indi].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[i].z -
						(int64_t)anchor_pos[0].z);
				temp 	-= ((int64_t)anchor_pos[i].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[ind_y_indi].z -
						(int64_t)anchor_pos[0].z);
				temp2 	= ((int64_t)anchor_pos[1].x - (int64_t)anchor_pos[0].x) * temp;

				/* Add (x_2 - x_0)*[(y_n - y_0)(z_1 - z_0) - (y_1 - y_0)(z_n - z_0)] */
				temp 	= ((int64_t)anchor_pos[i].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[1].z - (int64_t)anchor_pos[0].z);
				temp 	-= ((int64_t)anchor_pos[1].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[i].z - (int64_t)anchor_pos[0].z);
				temp2 	+= ((int64_t)anchor_pos[ind_y_indi].x - (int64_t)anchor_pos[0].x) * temp;

				/* Add (x_n - x_0)*[(y_1 - y_0)(z_2 - z_0) - (y_2 - y_0)(z_1 - z_0)] */
				temp 	= ((int64_t)anchor_pos[1].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[ind_y_indi].z -
						(int64_t)anchor_pos[0].z);
				temp 	-= ((int64_t)anchor_pos[ind_y_indi].y - (int64_t)anchor_pos[0].y) * ((int64_t)anchor_pos[1].z -
						(int64_t)anchor_pos[0].z);
				temp2 	+= ((int64_t)anchor_pos[i].x - (int64_t)anchor_pos[0].x) * temp;

				if (temp2 != 0) { lin_dep = false; }
			}
		}

		/* Leave function, if rank is below 3 */
		if (lin_dep == true) {
			return UWB_LIN_DEP_FOR_FOUR;
		}
	}

	/************************************************** Algorithm ***********************************************************************/

	/* Writing values resulting from least square error method (A_trans*A*x = A_trans*r; row 0 was used to remove x^2,y^2,z^2 entries => index starts at 1) */
	for (int i = 1; i < no_valid_distances; i++) {
		/* Matrix (needed to be multiplied with 2, afterwards) */
		M_11 += sq((int64_t)(anchor_pos[i].x - anchor_pos[0].x));
		M_12 += (int64_t)((int64_t)(anchor_pos[i].x - anchor_pos[0].x) * (int64_t)(anchor_pos[i].y - anchor_pos[0].y));
		M_13 += (int64_t)((int64_t)(anchor_pos[i].x - anchor_pos[0].x) * (int64_t)(anchor_pos[i].z - anchor_pos[0].z));
		M_22 += sq((int64_t)(anchor_pos[i].y - anchor_pos[0].y));
		M_23 += (int64_t)((int64_t)(anchor_pos[i].y - anchor_pos[0].y) * (int64_t)(anchor_pos[i].z - anchor_pos[0].z));
		M_33 += sq((int64_t)(anchor_pos[i].z - anchor_pos[0].z));

		/* Vector */
		temp = sq(distances_cm[0]) - sq(distances_cm[i])
		       + sq(anchor_pos[i].x) + sq(anchor_pos[i].y)
		       + sq(anchor_pos[i].z) - sq(anchor_pos[0].x)
		       - sq(anchor_pos[0].y) - sq(anchor_pos[0].z);

		b[0] += (int64_t)((int64_t)(anchor_pos[i].x - anchor_pos[0].x) * temp);
		b[1] += (int64_t)((int64_t)(anchor_pos[i].y - anchor_pos[0].y) * temp);
		b[2] += (int64_t)((int64_t)(anchor_pos[i].z - anchor_pos[0].z) * temp);
	}

	M_11 = 2 * M_11;
	M_12 = 2 * M_12;
	M_13 = 2 * M_13;
	M_22 = 2 * M_22;
	M_23 = 2 * M_23;
	M_33 = 2 * M_33;

	/* Calculating the z-position, if calculation is possible (at least one anchor at z != 0) */
	if (anchors_on_x_y_plane == false) {
		nominator = b[0] * (M_12 * M_23 - M_13 * M_22) + b[1] * (M_12 * M_13 - M_11 * M_23) + b[2] *
			    (M_11 * M_22 - M_12 * M_12);			// [cm^7]
		denominator = M_11 * (M_33 * M_22 - M_23 * M_23) + 2 * M_12 * M_13 * M_23 - M_33 * M_12 * M_12 - M_22 * M_13 *
			      M_13;				// [cm^6]

		/* Check, if denominator is zero (Rank of matrix not high enough) */
		if (denominator == 0) {
			return UWB_RANK_ZERO;
		}

		z_pos = ((nominator * 10) / denominator + 5) / 10;	// [cm]
	}

	/* Else prepare for different calculation approach (after x and y were calculated) */
	else {
		z_pos = 0;
	}

	/* Calculating the y-position */
	nominator = b[1] * M_11 - b[0] * M_12 - (z_pos * (M_11 * M_23 - M_12 * M_13));	// [cm^5]
	denominator = M_11 * M_22 - M_12 * M_12;// [cm^4]

	/* Check, if denominator is zero (Rank of matrix not high enough) */
	if (denominator == 0) {
		return UWB_RANK_ZERO;
	}

	y_pos = ((nominator * 10) / denominator + 5) / 10;	// [cm]

	/* Calculating the x-position */
	nominator = b[0] - z_pos * M_13 - y_pos * M_12;	// [cm^3]
	denominator = M_11;	// [cm^2]

	x_pos = ((nominator * 10) / denominator + 5) / 10;// [cm]

	/* Calculate z-position form x and y coordinates, if z can't be determined by previous steps (All anchors at z_n = 0) */
	if (anchors_on_x_y_plane == true) {
		/* Calculate z-positon relative to the anchor grid's height */
		for (int i = 0; i < no_distances; i++) {
			/* z² = dis_meas_n² - (x - x_anc_n)² - (y - y_anc_n)² */
			temp = (int64_t)((int64_t)pow(distances_cm[i], 2)
					 - (int64_t)pow((x_pos - (int64_t)anchor_pos[i].x), 2)
					 - (int64_t)pow((y_pos - (int64_t)anchor_pos[i].y), 2));

			/* z² must be positive, else x and y must be wrong => calculate positive sqrt and sum up all calculated heights, if positive */
			if (temp >= 0) {
				z_pos += (int64_t)sqrt(temp);

			} else {
				z_pos = 0;
			}
		}

		z_pos = z_pos / no_distances;										// Divide sum by number of distances to get the average

		/* Add height of the anchor grid's height */
		z_pos += anchor_pos[0].z;
	}

	//Output is the Coordinates of the Initiator in relation to 0,0,0 in NEU (North-East-Up) Framing
	// The end goal of this math is to get the position relative to the landing point in the NED frame.
	_current_position_uwb_init = matrix::Vector3f(x_pos, y_pos, z_pos);

	// Construct the rotation from the UWB_R4frame to the NWU frame.
	// The UWB_SR150 frame is just NWU, rotated by some amount about the Z (up) axis. (Parameter Yaw offset)
	// To get back to NWU, just rotate by negative this amount about Z.
	_uwb_init_to_nwu = matrix::Dcmf(matrix::Eulerf(0.0f, 0.0f,
					-(_uwb_init_off_yaw.get() * M_PI_F / 180.0f))); //
	// The actual conversion:
	//  - Subtract _landing_point to get the position relative to the landing point, in UWB_R4 frame
	//  - Rotate by _rddrone_to_nwu to get into the NWU frame
	//  - Rotate by _nwu_to_ned to get into the NED frame
	_current_position_ned = _nwu_to_ned * _uwb_init_to_nwu * _current_position_uwb_init;

	// Now the position is the landing point relative to the vehicle.
	// so the only thing left is to convert cm to Meters and to add the Initiator offset
	_rel_pos =  _current_position_ned / 100 + _uwb_init_offset_v3f;

	// The UWB report contains the position of the vehicle relative to the landing point.

	return UWB_OK;
}

int uwb_sr150_main(int argc, char *argv[])
{
	return UWB_SR150::main(argc, argv);
}

void UWB_SR150::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}
