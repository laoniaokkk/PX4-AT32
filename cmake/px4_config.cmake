############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# find PX4 config
#  look for in tree board config that matches CONFIG input

if(NOT CONFIG)
	# default to px4_ros2_default if building within a ROS2 colcon environment
	if(("$ENV{COLCON}" MATCHES "1") AND ("$ENV{ROS_VERSION}" MATCHES "2"))
		set(CONFIG "px4_ros2_default" CACHE STRING "desired configuration")
	else()
		set(CONFIG "px4_sitl_default" CACHE STRING "desired configuration")
	endif()
endif()

if(NOT PX4_CONFIG_FILE)

	file(GLOB_RECURSE board_configs
		RELATIVE "${PX4_SOURCE_DIR}/boards"
		"boards/*.px4board"
		)

	foreach(filename ${board_configs})
		# parse input CONFIG into components to match with existing in tree configs
		#  the platform prefix (eg nuttx_) is historical, and removed if present
		string(REPLACE ".px4board" "" filename_stripped ${filename})
		string(REPLACE "/" ";" config ${filename_stripped})
		list(LENGTH config config_len)

		if(${config_len} EQUAL 3)
			list(GET config 0 vendor)
			list(GET config 1 model)
			list(GET config 2 label)

			set(board "${vendor}${model}")

			# <VENDOR>_<MODEL>_<LABEL> (eg px4_fmu-v2_default)
			# <VENDOR>_<MODEL>_default (eg px4_fmu-v2) # allow skipping label if "default"
			if ((${CONFIG} MATCHES "${vendor}_${model}_${label}") OR # match full vendor, model, label
			    ((${label} STREQUAL "default") AND (${CONFIG} STREQUAL "${vendor}_${model}")) # default label can be omitted
			)
				set(PX4_CONFIG_FILE "${PX4_SOURCE_DIR}/boards/${filename}" CACHE FILEPATH "path to PX4 CONFIG file" FORCE)
				set(PX4_BOARD_DIR "${PX4_SOURCE_DIR}/boards/${vendor}/${model}" CACHE STRING "PX4 board directory" FORCE)
				set(MODEL "${model}" CACHE STRING "PX4 board model" FORCE)
				set(VENDOR "${vendor}" CACHE STRING "PX4 board vendor" FORCE)
				set(LABEL "${label}" CACHE STRING "PX4 board vendor" FORCE)
				break()
			endif()

			# <BOARD>_<LABEL> (eg px4_fmu-v2_default)
			# <BOARD>_default (eg px4_fmu-v2) # allow skipping label if "default"
			if ((${CONFIG} MATCHES "${board}_${label}") OR # match full board, label
			    ((${label} STREQUAL "default") AND (${CONFIG} STREQUAL "${board}")) # default label can be omitted
			)
				set(PX4_CONFIG_FILE "${PX4_SOURCE_DIR}/boards/${filename}" CACHE FILEPATH "path to PX4 CONFIG file" FORCE)
				set(PX4_BOARD_DIR "${PX4_SOURCE_DIR}/boards/${vendor}/${model}" CACHE STRING "PX4 board directory" FORCE)
				set(MODEL "${model}" CACHE STRING "PX4 board model" FORCE)
				set(VENDOR "${vendor}" CACHE STRING "PX4 board vendor" FORCE)
				set(LABEL "${label}" CACHE STRING "PX4 board vendor" FORCE)
				break()
			endif()
		endif()
	endforeach()
endif()

message(STATUS "PX4 config file: ${PX4_CONFIG_FILE}")

include_directories(${PX4_BOARD_DIR}/src)

set(PX4_BOARD ${VENDOR}_${MODEL} CACHE STRING "PX4 board" FORCE)

# board name is uppercase with no underscores when used as a define
string(TOUPPER ${PX4_BOARD} PX4_BOARD_NAME)
string(REPLACE "-" "_" PX4_BOARD_NAME ${PX4_BOARD_NAME})
set(PX4_BOARD_NAME ${PX4_BOARD_NAME} CACHE STRING "PX4 board define" FORCE)

set(PX4_BOARD_VENDOR ${VENDOR} CACHE STRING "PX4 board vendor" FORCE)
set(PX4_BOARD_MODEL ${MODEL} CACHE STRING "PX4 board model" FORCE)

set(PX4_BOARD_LABEL ${LABEL} CACHE STRING "PX4 board label" FORCE)

set(PX4_CONFIG "${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_${PX4_BOARD_LABEL}" CACHE STRING "PX4 config" FORCE)

if(EXISTS "${PX4_BOARD_DIR}/uavcan_board_identity")
	include ("${PX4_BOARD_DIR}/uavcan_board_identity")
endif()

if(EXISTS "${PX4_BOARD_DIR}/sitl.cmake")
	include ("${PX4_BOARD_DIR}/sitl.cmake")
endif()
