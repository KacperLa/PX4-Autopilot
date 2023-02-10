# ODrive Heartbeat.

uint64 timestamp		# time since system start (microseconds)

# ODrive state definitions
uint8 AXIS_STATE_UNDEFINED = 0                    # will fall through to idle
uint8 AXIS_STATE_IDLE = 1                         # disable PWM and do nothing
uint8 AXIS_STATE_STARTUP_SEQUENCE = 2             # the actual sequence is defined by the config.startup_... flags
uint8 AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3    # run all calibration procedures then idle
uint8 AXIS_STATE_MOTOR_CALIBRATION = 4            # run motor calibration
uint8 AXIS_STATE_SENSORLESS_CONTROL = 5           # run sensorless control
uint8 AXIS_STATE_ENCODER_INDEX_SEARCH = 6         # run encoder index search
uint8 AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7   # run encoder offset calibration
uint8 AXIS_STATE_CLOSED_LOOP_CONTROL = 8          # run closed loop control

uint8 axis_id # can id of the motor
uint32 axis_error # current errors present on the odrive
uint8 current_state # current state of the odrive
uint8 motor_flag
uint8 encoder_flag
uint8 controller_flag
uint8 trejectory_done

uint8 ORB_QUEUE_LENGTH = 64     # needs to match BOARD_MAX_LEDS
