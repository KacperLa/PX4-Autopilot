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

/**
 * @file ODriveCAN.cpp
 * @author Kacper Laska <klaskak@gmail.com>
 *
 * Driver for the ODrive motor controller over CAN.
 *
 * This driver simply decodes the CAN frames based on the specification
 * as provided in the Tattu datasheet DOC 001 REV D, which is highly
 * specific to the 12S 1600mAh battery. Other models of Tattu batteries
 * will NOT work with this driver in its current form.
 *
 */

#include "ODriveCAN.hpp"

extern orb_advert_t mavlink_log_pub;

bool ODriveCAN::taskShouldExit = false;

static const int CommandIDLength = 5;

ODriveCAN::ODriveCAN() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan)
{
}

ODriveCAN::~ODriveCAN()
{
    //perf_free(_sample_perf);
	//perf_free(_comms_errors);
}

void ODriveCAN::Heartbeat(CanFrame & inMsg, odrive_heartbeat_s * returnVals)
{
    memcpy(&(returnVals->axis_error), &(((uint8_t *)inMsg.payload)[0]), 4);

    returnVals->current_state = (((uint8_t *)inMsg.payload)[4]);
    returnVals->motor_flag =  (((uint8_t *)inMsg.payload)[5]);
    returnVals->encoder_flag =  (((uint8_t *)inMsg.payload)[6]);
    returnVals->controller_flag =  (((uint8_t *)inMsg.payload)[7]) & 1UL;
    returnVals->trejectory_done = ( (((uint8_t *)inMsg.payload)[7]) >> 7) & 1UL;
}

void ODriveCAN::encoderEstimate(CanFrame & inMsg, odrive_encoder_s * returnVals)
{
    memcpy(&(returnVals->pos_estimate), &(((uint8_t *)inMsg.payload)[0]), 4);
    memcpy(&(returnVals->vel_estimate), &(((uint8_t *)inMsg.payload)[4]), 4);
}

//////////// State helper ///////////

bool ODriveCAN::RunState(int axis_id, int requested_state) {
    sendMessage(axis_id, CMD_ID_SET_AXIS_REQUESTED_STATE, false, 4, (uint8_t*) &requested_state);
    return true;
}


void ODriveCAN::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {

	    struct can_dev_s *can = stm32_caninitialize(1);

		if (can == nullptr) {
			PX4_ERR("Failed to get CAN interface");
			return;

		}
		/* Register the CAN driver at "/dev/can0" */
		int ret = can_register("/dev/can0", can);
		if (ret < 0) {
			PX4_ERR("can_register failed: %d", ret);
			return;
		}

		// open /dev/can0
		_can_fd = ::open("/dev/can0", O_RDWR | O_NONBLOCK);
		if (_can_fd < 0) {
			PX4_ERR("FAILED TO OPEN /dev/can0");
			return;
		} else {
			PX4_INFO("CAN opened successfully on /dev/can0");
		}

		_initialized = true;
	}
	//Subscribe to armed cmd
	_armedSub = orb_subscribe(ORB_ID(actuator_armed));

	pollfd fds[2];
	fds[0].fd = _armedSub;
	fds[0].events = POLLIN;
	fds[1].fd = _can_fd;
	fds[1].events = POLLIN;


	while (!taskShouldExit) {
		//Blocks with a timeout
		int pret = poll(fds, sizeof(fds) / sizeof(pollfd), 0);

		if (pret > 0)
		{
			if (fds[0].revents & POLLIN)
			{
				orb_copy(ORB_ID(actuator_armed), _armedSub, &_actuatorArmed);
				const bool disarmed = !_actuatorArmed.armed || _actuatorArmed.lockdown || _actuatorArmed.manual_lockdown
					      || _actuatorArmed.force_failsafe;

				if (disarmed) {
					RunState(0, AXIS_STATE_IDLE);
					RunState(1, AXIS_STATE_IDLE);
				} else {
					RunState(0, AXIS_STATE_CLOSED_LOOP_CONTROL);
					RunState(1, AXIS_STATE_CLOSED_LOOP_CONTROL);
				}
			}

			//Process can frame
			if (fds[1].revents & POLLIN)
			{
				process_frame();
			}
		}
	}
	orb_unsubscribe(_armedSub);

}

void ODriveCAN::process_frame(){
	uint8_t data[64] {};
	CanFrame received_frame{};
	received_frame.payload = &data;

	if (receive(&received_frame) > 0) {
		uint8_t axis_id = received_frame.can_id >> 5;
		uint8_t cmd_id = received_frame.can_id & 0x01F;

		PX4_INFO("can id: %u, cmd id: %u", axis_id, cmd_id);
		switch (cmd_id) {
			case (ODriveCAN::CMD_ID_ODRIVE_HEARTBEAT_MESSAGE):
			{
			PX4_INFO("got heartbeat!");
			odrive_heartbeat_s heart_beat = {};
			heart_beat.timestamp = hrt_absolute_time();
			heart_beat.axis_id = axis_id;
			Heartbeat(received_frame, &heart_beat);
			_odrive_heartbeat_pub.publish(heart_beat);
			break;
			}
			case (ODriveCAN::CMD_ID_GET_ENCODER_ESTIMATES):
			{
			PX4_INFO("got encoder!");
			odrive_encoder_s encoder_msg = {};
			encoder_msg.timestamp = hrt_absolute_time();
			encoder_msg.axis_id = axis_id;
			encoderEstimate(received_frame, &encoder_msg);
			_odrive_encoder_pub.publish(encoder_msg);
			break;

			//printf("get encoder reading from axis: %d \n", axis_id);
			//EncoderEstimatesMsg_t new_estimate;
			//odriveCAN.GetPositionVelocityResponse(new_estimate, inMsg);
			//if (axis_id == 1){
			//  printf("vel estimate: %f, %f\n", new_estimate.posEstimate, new_estimate.velEstimate);
			//}
			break;
			}
			case (ODriveCAN::CMD_ID_GET_ADC_VOLTAGE):
			{
			//float adcVoltage = odriveCAN.GetADCVoltageResponse(inMsg);
			break;
			}
			case (ODriveCAN::CMD_ID_GET_VBUS_VOLTAGE):
			{
			//float _vbusVoltage = odriveCAN.GetVbusVoltageResponse(inMsg);
			//printf("got voltage: %f \n", _vbusVoltage);
			break;
			}
			default:
			{
			break;
			}
		}
	}
}

int16_t ODriveCAN::receive(CanFrame *received_frame)
{
//     if ((_fd < 0) || (received_frame == nullptr)) {
// 		return -1;
// 	}

	// File desriptor for CAN.


	// Any recieved CAN messages will cause the poll statement to unblock and run
	// This way CAN read runs with minimal latency.
	// Note that multiple messages may be received in a short time, so this will try to read any availible in a loop
	//::poll(&fds, 1, 0);

	// Only execute this part if can0 is changed.
	//if (fds.revents & POLLIN) {
		// Try to read.
		struct can_msg_s receive_msg;
		const ssize_t nbytes = ::read(_can_fd, &receive_msg, sizeof(receive_msg));

		if (nbytes < 0 || (size_t)nbytes < CAN_MSGLEN(0) || (size_t)nbytes > sizeof(receive_msg)) {
			// error
			return -1;

		} else {
			received_frame->can_id = receive_msg.cm_hdr.ch_id;
			received_frame->payload_size = receive_msg.cm_hdr.ch_dlc;
			memcpy((void *)received_frame->payload, receive_msg.cm_data, receive_msg.cm_hdr.ch_dlc);
			return nbytes;
		}
	//}

	return 0;
}

int16_t ODriveCAN::transmit(struct can_msg_s transmit_msg, int timeout_ms)
{
	if (_can_fd < 0) {
		return -1;
	}

	struct pollfd fds {};

	fds.fd = _can_fd;

	fds.events |= POLLOUT;

	const int poll_result = poll(&fds, 1, timeout_ms);

	if (poll_result < 0) {
		return -1;
	}

	if (poll_result == 0) {
		return 0;
	}

	if ((fds.revents & POLLOUT) == 0) {
		return -1;
	}

	const size_t msg_len = CAN_MSGLEN(transmit_msg.cm_hdr.ch_dlc);

	const ssize_t nbytes = ::write(_can_fd, &transmit_msg, msg_len);

	if (nbytes < 0 || (size_t)nbytes != msg_len) {
		return -1;
	}

	return 1;
}

void ODriveCAN::sendMessage(int axis_id, int cmd_id, bool remote_request, int length, uint8_t *signal_bytes) {
	struct can_msg_s transmit_msg {};


	transmit_msg.cm_hdr.ch_id = (axis_id << CommandIDLength) + cmd_id;

	transmit_msg.cm_hdr.ch_rtr = remote_request;

	transmit_msg.cm_hdr.ch_dlc = length;

  	//transmit_msg.cm_hdr.ch_extid  = false;     /* Standard/Extend mode   */

	memcpy(transmit_msg.cm_data, signal_bytes, length);

    	transmit(transmit_msg, 100);
}

int ODriveCAN::start()
{
	// There is a race condition at boot that sometimes causes opening of
	// /dev/can0 to fail. We will delay 0.5s to be safe.
	uint32_t delay_us = 500000;
	ScheduleOnInterval(1000000 / SAMPLE_RATE, delay_us);
	return PX4_OK;
}

int ODriveCAN::task_spawn(int argc, char *argv[])
{
	ODriveCAN *instance = new ODriveCAN();

	if (!instance) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->start();
	return 0;
}

int ODriveCAN::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for interfacing with the ODrive motor controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("odrive_can", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int ODriveCAN::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("Unrecognized command.");
}

extern "C" __EXPORT int odrive_main(int argc, char *argv[])
{
	return ODriveCAN::main(argc, argv);
}










// static const int kMotorOffsetFloat = 2;
// static const int kMotorStrideFloat = 28;
// static const int kMotorOffsetInt32 = 0;
// static const int kMotorStrideInt32 = 4;
// static const int kMotorOffsetBool = 0;
// static const int kMotorStrideBool = 4;
// static const int kMotorOffsetUint16 = 0;
// static const int kMotorStrideUint16 = 2;

// static const int NodeIDLength = 6;

// static const float feedforwardFactor = 1 / 0.001;






// void ODriveCAN::SetAxisNodeId(int axis_id, int node_id) {
// 	byte* node_id_b = (byte*) &node_id;

// 	sendMessage(axis_id, CMD_ID_SET_AXIS_NODE_ID, false, 4, node_id_b);
// }

// void ODriveCAN::SetControllerModes(int axis_id, int control_mode, int input_mode) {
// 	byte* control_mode_b = (std::byte*) &control_mode;
// 	byte* input_mode_b = (std::byte*) &input_mode;
// 	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// 	msg_data[0] = control_mode_b[0];
// 	msg_data[1] = control_mode_b[1];
// 	msg_data[2] = control_mode_b[2];
// 	msg_data[3] = control_mode_b[3];
// 	msg_data[4] = input_mode_b[0];
// 	msg_data[5] = input_mode_b[1];
// 	msg_data[6] = input_mode_b[2];
// 	msg_data[7] = input_mode_b[3];

// 	sendMessage(axis_id, CMD_ID_SET_CONTROLLER_MODES, false, 8, msg_data);
// }

// void ODriveCAN::SetPosition(int axis_id, float position) {
//     SetPosition(axis_id, position, 0.0f, 0.0f);
// }

// void ODriveCAN::SetPosition(int axis_id, float position, float velocity_feedforward) {
//     SetPosition(axis_id, position, velocity_feedforward, 0.0f);
// }

// void ODriveCAN::SetPosition(int axis_id, float position, float velocity_feedforward, float current_feedforward) {
//     int16_t vel_ff = (int16_t) (feedforwardFactor * velocity_feedforward);
//     int16_t curr_ff = (int16_t) (feedforwardFactor * current_feedforward);

//     byte* position_b = (byte*) &position;
//     byte* velocity_feedforward_b = (byte*) &vel_ff;
//     byte* current_feedforward_b = (byte*) &curr_ff;
//     byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//     msg_data[0] = position_b[0];
//     msg_data[1] = position_b[1];
//     msg_data[2] = position_b[2];
//     msg_data[3] = position_b[3];
//     msg_data[4] = velocity_feedforward_b[0];
//     msg_data[5] = velocity_feedforward_b[1];
//     msg_data[6] = current_feedforward_b[0];
//     msg_data[7] = current_feedforward_b[1];

//     sendMessage(axis_id, CMD_ID_SET_INPUT_POS, false, 8, msg_data);
// }

// void ODriveCAN::SetVelocity(int axis_id, float velocity) {
//     SetVelocity(axis_id, velocity, 0.0f);
// }

// void ODriveCAN::SetVelocity(int axis_id, float velocity, float current_feedforward) {
//     uint8_t* velocity_b = (uint8_t*) &velocity;
//     uint8_t* current_feedforward_b = (uint8_t*) &current_feedforward;
//     uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//     msg_data[0] = velocity_b[0];
//     msg_data[1] = velocity_b[1];
//     msg_data[2] = velocity_b[2];
//     msg_data[3] = velocity_b[3];
//     msg_data[4] = current_feedforward_b[0];
//     msg_data[5] = current_feedforward_b[1];
//     msg_data[6] = current_feedforward_b[2];
//     msg_data[7] = current_feedforward_b[3];

//     sendMessage(axis_id, CMD_ID_SET_INPUT_VEL, false, 8, msg_data);
// }

// void ODriveCAN::SetTorque(int axis_id, float torque) {
//     byte* torque_b = (byte*) &torque;

//     sendMessage(axis_id, CMD_ID_SET_INPUT_TORQUE, false, 4, torque_b);
// }

// void ODriveCAN::SetLimits(int axis_id, float velocity_limit, float current_limit) {
//     byte* velocity_limit_b = (byte*) &velocity_limit;
// 	byte* current_limit_b = (byte*) &current_limit;
//     byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//     msg_data[0] = velocity_limit_b[0];
//     msg_data[1] = velocity_limit_b[1];
//     msg_data[2] = velocity_limit_b[2];
//     msg_data[3] = velocity_limit_b[3];
//     msg_data[4] = current_limit_b[0];
//     msg_data[5] = current_limit_b[1];
//     msg_data[6] = current_limit_b[2];
//     msg_data[7] = current_limit_b[3];

//     sendMessage(axis_id, CMD_ID_SET_LIMITS, false, 8, msg_data);
// }

// void ODriveCAN::SetTrajVelLimit(int axis_id, float traj_vel_limit) {
//     byte* traj_vel_limit_b = (byte*) &traj_vel_limit;

//     sendMessage(axis_id, CMD_ID_SET_TRAJ_VEL_LIMIT, false, 4, traj_vel_limit_b);
// }

// void ODriveCAN::SetTrajAccelLimits(int axis_id, float traj_accel_limit, float traj_decel_limit) {
// 	byte* traj_accel_limit_b = (byte*) &traj_accel_limit;
// 	byte* traj_decel_limit_b = (byte*) &traj_decel_limit;
// 	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// 	msg_data[0] = traj_accel_limit_b[0];
// 	msg_data[1] = traj_accel_limit_b[1];
// 	msg_data[2] = traj_accel_limit_b[2];
// 	msg_data[3] = traj_accel_limit_b[3];
// 	msg_data[4] = traj_decel_limit_b[0];
// 	msg_data[5] = traj_decel_limit_b[1];
// 	msg_data[6] = traj_decel_limit_b[2];
// 	msg_data[7] = traj_decel_limit_b[3];

// 	sendMessage(axis_id, CMD_ID_SET_TRAJ_ACCEL_LIMITS, false, 8, msg_data);
// }

// void ODriveCAN::SetTrajInertia(int axis_id, float traj_inertia) {
//     byte* traj_inertia_b = (byte*) &traj_inertia;

//     sendMessage(axis_id, CMD_ID_SET_TRAJ_INERTIA, false, 4, traj_inertia_b);
// }

// void ODriveCAN::SetLinearCount(int axis_id, int linear_count) {
//     byte* linear_count_b = (byte*) &linear_count;

//     sendMessage(axis_id, CMD_ID_SET_LINEAR_COUNT, false, 4, linear_count_b);
// }

// void ODriveCAN::SetPositionGain(int axis_id, float position_gain) {
//     byte* position_gain_b = (byte*) &position_gain;

//     sendMessage(axis_id, CMD_ID_SET_POS_GAIN, false, 4, position_gain_b);
// }

// void ODriveCAN::SetVelocityGains(int axis_id, float velocity_gain, float velocity_integrator_gain) {
//     byte* velocity_gain_b = (byte*) &velocity_gain;
// 	byte* velocity_integrator_gain_b = (byte*) &velocity_integrator_gain;
//     byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//     msg_data[0] = velocity_gain_b[0];
//     msg_data[1] = velocity_gain_b[1];
//     msg_data[2] = velocity_gain_b[2];
//     msg_data[3] = velocity_gain_b[3];
//     msg_data[4] = velocity_integrator_gain_b[0];
//     msg_data[5] = velocity_integrator_gain_b[1];
//     msg_data[6] = velocity_integrator_gain_b[2];
//     msg_data[7] = velocity_integrator_gain_b[3];

//     sendMessage(axis_id, CMD_ID_SET_VEL_GAINS, false, 8, msg_data);
// }

//////////// Get functions ///////////

// void ODriveCAN::GetPositionVelocity(int axis_id) {
//     byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//     sendMessage(axis_id, CMD_ID_GET_ENCODER_ESTIMATES, true, 8, msg_data);
// }

// void ODriveCAN::GetPositionVelocityResponse(EncoderEstimatesMsg_t &returnVal, struct can_frame &inMsg) {
// 	returnVal.parseMessage(inMsg);
// }

// void ODriveCAN::GetEncoderCounts(int axis_id) {
// 	uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//     sendMessage(axis_id, CMD_ID_GET_ENCODER_COUNT, true, 8, msg_data);
// }

// void ODriveCAN::GetEncoderCountsResponse(EncoderCountsMsg_t &returnVal, struct can_frame &inMsg) {
// 	returnVal.parseMessage(inMsg);
// }

// void ODriveCAN::GetIq(int axis_id) {
// 	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//     sendMessage(axis_id, CMD_ID_GET_IQ, true, 8, msg_data);
// }

// void ODriveCAN::GetIqResponse(IqMsg_t &returnVal, struct can_frame &inMsg) {
// 	returnVal.parseMessage(inMsg);
// }

// void ODriveCAN::GetSensorlessEstimates(int axis_id) {
// 	byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//     sendMessage(axis_id, CMD_ID_GET_SENSORLESS_ESTIMATES, true, 8, msg_data);
// }

// void ODriveCAN::GetSensorlessEstimatesResponse(SensorlessEstimatesMsg_t &returnVal, struct can_frame &inMsg) {
// 	returnVal.parseMessage(inMsg);
// }

// void ODriveCAN::GetMotorError(int axis_id) {
//     byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//     sendMessage(axis_id, CMD_ID_GET_MOTOR_ERROR, true, 8, msg_data);
// }

// uint64_t ODriveCAN::GetMotorErrorResponse(struct can_frame &inMsg) {
//     uint64_t output;
//     *((uint8_t *)(&output) + 0) = inMsg.buf[0];
//     *((uint8_t *)(&output) + 1) = inMsg.buf[1];
//     *((uint8_t *)(&output) + 2) = inMsg.buf[2];
//     *((uint8_t *)(&output) + 3) = inMsg.buf[3];
//     *((uint8_t *)(&output) + 0) = inMsg.buf[4];
//     *((uint8_t *)(&output) + 1) = inMsg.buf[5];
//     *((uint8_t *)(&output) + 2) = inMsg.buf[6];
//     *((uint8_t *)(&output) + 3) = inMsg.buf[7];
//     return output;
// }

// void ODriveCAN::GetControllerError(int axis_id) {
//     byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//     sendMessage(axis_id, CMD_ID_GET_CONTROLLER_ERROR, true, 8, msg_data);
// }

// uint32_t ODriveCAN::GetControllerErrorResponse(struct can_frame &inMsg) {
//     uint32_t output;
//     *((uint8_t *)(&output) + 0) = inMsg.buf[0];
//     *((uint8_t *)(&output) + 1) = inMsg.buf[1];
//     *((uint8_t *)(&output) + 2) = inMsg.buf[2];
//     *((uint8_t *)(&output) + 3) = inMsg.buf[3];
//     return output;
// }

// void ODriveCAN::GetEncoderError(int axis_id) {
//     byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//     sendMessage(axis_id, CMD_ID_GET_ENCODER_ERROR, true, 8, msg_data);
// }

// uint32_t ODriveCAN::GetEncoderErrorResponse(struct can_frame &inMsg) {
//     uint32_t output;
//     *((uint8_t *)(&output) + 0) = inMsg.buf[0];
//     *((uint8_t *)(&output) + 1) = inMsg.buf[1];
//     *((uint8_t *)(&output) + 2) = inMsg.buf[2];
//     *((uint8_t *)(&output) + 3) = inMsg.buf[3];
//     return output;
// }

//message can be sent to either axis
// void ODriveCAN::GetVbusVoltage(int axis_id) {
//     uint8_t msg_data[4] = {0, 0, 0, 0};

//     sendMessage(axis_id, CMD_ID_GET_VBUS_VOLTAGE, true, 4, msg_data);
// }

// float ODriveCAN::GetVbusVoltageResponse(struct can_frame &inMsg) {
//     float output;
//     *((uint8_t*)(&output) + 0) = inMsg.data[0];
//     *((uint8_t*)(&output) + 1) = inMsg.data[1];
//     *((uint8_t*)(&output) + 2) = inMsg.data[2];
//     *((uint8_t*)(&output) + 3) = inMsg.data[3];
//     return output;
// }

// void ODriveCAN::GetADCVoltage(int axis_id, uint8_t gpio_num) {
//     byte msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// 	msg_data[0] = gpio_num;

//     sendMessage(axis_id, CMD_ID_GET_ADC_VOLTAGE, true, 8, msg_data);  //RTR must be false!
// }

// float ODriveCAN::GetADCVoltageResponse(struct can_frame &inMsg) {
//     float_t output;
//     *((uint8_t *)(&output) + 0) = inMsg.buf[0];
//     *((uint8_t *)(&output) + 1) = inMsg.buf[1];
//     *((uint8_t *)(&output) + 2) = inMsg.buf[2];
//     *((uint8_t *)(&output) + 3) = inMsg.buf[3];
//     return output;
// }

//////////// Other functions ///////////

// void ODriveCAN::Estop(int axis_id) {
//     sendMessage(axis_id, CMD_ID_ODRIVE_ESTOP_MESSAGE, false, 0, 0);  //message requires no data, thus the 0, 0
// }

// void ODriveCAN::StartAnticogging(int axis_id) {
//     sendMessage(axis_id, CMD_ID_START_ANTICOGGING, false,0, 0);  //message requires no data, thus the 0, 0
// }

// void ODriveCAN::RebootOdrive(int axis_id) {  //message can be sent to either axis
//     sendMessage(axis_id, CMD_ID_REBOOT_ODRIVE, false,0, 0);
// }

// void ODriveCAN::ClearErrors(int axis_id) {
//     sendMessage(axis_id, CMD_ID_CLEAR_ERRORS, false, 0, 0);  //message requires no data, thus the 0, 0
// }


