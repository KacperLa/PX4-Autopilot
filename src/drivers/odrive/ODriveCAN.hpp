/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include "stm32_can.h"
#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include <fcntl.h>
#include <poll.h>

// #include <nuttx/net/netdev.h>
// #include <nuttx/net/can.h>

//#include "uavcan_driver.hpp"
//#include <uavcan_nuttx/socketcan.hpp>

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/odrive_heartbeat.h>
#include <uORB/topics/odrive_encoder.h>
#include <uORB/topics/actuator_armed.h>

using namespace time_literals;

typedef struct {
	uint64_t timestamp_usec;
	uint32_t can_id;
	size_t      payload_size;
	const void * payload;
} CanFrame;


// typedef struct EncoderCountsMsg_t {
// 	int32_t shadowCount = 0;
// 	int32_t countInCPR = 0;

// 	void parseMessage(const CanFrame &inMsg) {
//         memcpy(&(shadowCount), &inMsg.payload[0], 4);
//         memcpy(&(countInCPR), &inMsg.payload[4], 4);
// 	}
// } EncoderCountsMsg_t;

// typedef struct IqMsg_t {
// 	float iqSetpoint = 0;
// 	float iqMeasured = 0;

// 	void parseMessage(const CanFrame &inMsg) {
//         memcpy(&(iqSetpoint), &inMsg.payload[0], 4);
//         memcpy(&(iqMeasured), &inMsg.payload[4], 4);
// 	}
// } IqMsg_t;

// typedef struct SensorlessEstimatesMsg_t {
// 	float posEstimate = 0;
// 	float velEstimate = 0;

// 	void parseMessage(const CanFrame &inMsg) {
//         memcpy(&(posEstimate), &inMsg.payload[0], 4);
//         memcpy(&(velEstimate), &inMsg.payload[4], 4);
// 	}
// } SensorlessEstimatesMsg_t;

class ODriveCAN : public ModuleBase<ODriveCAN>, public px4::ScheduledWorkItem{
public:
	static bool taskShouldExit;


    enum AxisState_t {
        AXIS_STATE_UNDEFINED = 0,           //<! will fall through to idle
        AXIS_STATE_IDLE = 1,                //<! disable PWM and do nothing
        AXIS_STATE_STARTUP_SEQUENCE = 2, //<! the actual sequence is defined by the config.startup_... flags
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
        AXIS_STATE_MOTOR_CALIBRATION = 4,   //<! run motor calibration
        AXIS_STATE_SENSORLESS_CONTROL = 5,  //<! run sensorless control
        AXIS_STATE_ENCODER_INDEX_SEARCH = 6, //<! run encoder index search
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
        AXIS_STATE_CLOSED_LOOP_CONTROL = 8  //<! run closed loop control
    };

	enum ControlMode_t {
		VOLTAGE_CONTROL = 0,
		TORQUE_CONTROL = 1,
		VELOCITY_CONTROL = 2,
		POSITION_CONTROL = 3
	};

	enum InputMode_t {
		INACTIVE = 0,
		PASSTHROUGH = 1,
		VEL_RAMP = 2,
		POS_FILTER = 3,
		MIX_CHANNELS = 4,
		TRAP_TRAJ = 5,
		TORQUE_RAMP = 6,
		MIRROR = 7,
		TUNING = 8
	};

    enum CommandId_t {
        CMD_ID_CANOPEN_NMT_MESSAGE = 0x000,
        CMD_ID_ODRIVE_HEARTBEAT_MESSAGE = 0x001,
        CMD_ID_ODRIVE_ESTOP_MESSAGE = 0x002,
        CMD_ID_GET_MOTOR_ERROR = 0x003,
        CMD_ID_GET_ENCODER_ERROR = 0x004,
        CMD_ID_GET_SENSORLESS_ERROR = 0x005,
        CMD_ID_SET_AXIS_NODE_ID = 0x006,
        CMD_ID_SET_AXIS_REQUESTED_STATE = 0x007,
        CMD_ID_SET_AXIS_STARTUP_CONFIG = 0x008,
        CMD_ID_GET_ENCODER_ESTIMATES = 0x009,
        CMD_ID_GET_ENCODER_COUNT = 0x00A,
        CMD_ID_SET_CONTROLLER_MODES = 0x00B,
        CMD_ID_SET_INPUT_POS = 0x00C,
        CMD_ID_SET_INPUT_VEL = 0x00D,
        CMD_ID_SET_INPUT_TORQUE = 0x00E,
		CMD_ID_SET_LIMITS = 0x00F,
        CMD_ID_START_ANTICOGGING = 0x010,
        CMD_ID_SET_TRAJ_VEL_LIMIT = 0x011,
        CMD_ID_SET_TRAJ_ACCEL_LIMITS = 0x012,
        CMD_ID_SET_TRAJ_INERTIA = 0x013,
        CMD_ID_GET_IQ = 0x014,
        CMD_ID_GET_SENSORLESS_ESTIMATES = 0x015,
        CMD_ID_REBOOT_ODRIVE = 0x016,
        CMD_ID_GET_VBUS_VOLTAGE = 0x017,
        CMD_ID_CLEAR_ERRORS = 0x018,
		CMD_ID_SET_LINEAR_COUNT = 0x019,
		CMD_ID_SET_POS_GAIN = 0x01A,
		CMD_ID_SET_VEL_GAINS = 0x01B,
		CMD_ID_GET_ADC_VOLTAGE = 0x01C,
		CMD_ID_GET_CONTROLLER_ERROR = 0x01D,
        CMD_ID_CANOPEN_HEARTBEAT_MESSAGE = 0x700
    };

    ODriveCAN();

    //virtual ~ODriveCAN();
    ~ODriveCAN();

    	static int print_usage(const char *reason = nullptr);
	static int custom_command(int argc, char *argv[]);

	static int task_spawn(int argc, char *argv[]);

	int start();

	void process_frame();

	int16_t receive(CanFrame *received_frame);

	int16_t transmit(struct can_msg_s transmit_msg, int timeout_ms);

    	void sendMessage(int axis_id, int cmd_id, bool remote_function, int length, uint8_t *signal_bytes);


    	void Heartbeat(CanFrame & inMsg, odrive_heartbeat_s * returnVals);
    	void encoderEstimate(CanFrame & inMsg, odrive_encoder_s * returnVals);
	bool RunState(int axis_id, int requested_state);

    // Setters
	// void SetAxisNodeId(int axis_id, int node_id);
	// void SetControllerModes(int axis_id, int control_mode, int input_mode);
    // void SetPosition(int axis_id, float position);
    // void SetPosition(int axis_id, float position, float velocity_feedforward);
    // void SetPosition(int axis_id, float position, float velocity_feedforward, float current_feedforward);
    //void SetVelocity(int axis_id, float velocity);
    //void SetVelocity(int axis_id, float velocity, float current_feedforward);
    // void SetTorque(int axis_id, float torque);
	// void SetLimits(int axis_id, float velocity_limit, float current_limit);
	// void SetTrajVelLimit(int axis_id, float traj_vel_limit);
	// void SetTrajAccelLimits(int axis_id, float traj_accel_limit, float traj_decel_limit);
	// void SetTrajInertia(int axis_id, float traj_inertia);
	// void SetLinearCount(int axis_id, int linear_count);
	// void SetPositionGain(int axis_id, float position_gain);
	// void SetVelocityGains(int axis_id, float velocity_gain, float velocity_integrator_gain);

    // Getters
    // void GetPositionVelocity(int axis_id);
    //void GetPositionVelocityResponse(EncoderEstimatesMsg_t &returnVal, struct can_frame &inMsg);
	//void GetEncoderCounts(int axis_id);
	// void GetEncoderCountsResponse(EncoderCountsMsg_t &returnVal, struct can_frame &inMsg);
	// void GetIq(int axis_id);
	// void GetIqResponse(IqMsg_t &returnVal, struct can_frame &inMsg);
	// void GetSensorlessEstimates(int axis_id);
	// void GetSensorlessEstimatesResponse(SensorlessEstimatesMsg_t &returnVal, struct can_frame &inMsg);
    // void GetMotorError(int axis_id);
	// uint64_t GetMotorErrorResponse(struct can_frame &inMsg);
    // void GetControllerError(int axis_id);
    // uint32_t GetControllerErrorResponse(struct can_frame &inMsg);
    // void GetEncoderError(int axis_id);
    // uint32_t GetEncoderErrorResponse(struct can_frame &inMsg);
	//void GetVbusVoltage(int axis_id);  //Can be sent to either axis
	//float GetVbusVoltageResponse(struct can_frame &inMsg);
	// void GetADCVoltage(int axis_id, uint8_t gpio_num);
	// float GetADCVoltageResponse(struct can_frame &inMsg);

	// Other functions
	// void Estop(int axis_id);
	// void StartAnticogging(int axis_id);
	// void RebootOdrive(int axis_id);  //Can be sent to either axis
	// void ClearErrors(int axis_id);

    // State helper
    // bool RunState(int axis_id, int requested_state);


private:
    static constexpr uint32_t SAMPLE_RATE{100}; // samples per second (10ms)
	static constexpr size_t TAIL_BYTE_START_OF_TRANSFER{128};

	void Run() override;

	int _can_fd{-1};

	int _armedSub{-1};
	actuator_armed_s _actuatorArmed;

	bool _initialized{false};

	uORB::Publication<odrive_heartbeat_s> _odrive_heartbeat_pub{ORB_ID::odrive_heartbeat};
	uORB::Publication<odrive_encoder_s> _odrive_encoder_pub{ORB_ID::odrive_encoder};

};
