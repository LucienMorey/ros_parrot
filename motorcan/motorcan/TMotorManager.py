from motorcan.TMotorStructures import (
    MIT_command,
    TMotorState,
    TMotorControlState,
    MIT_Params,
    MIT_motor_state,
)
from motorcan.CAN_Manager import CAN_Manager
import numpy as np
from can import Listener, Message, BufferedReader
import time
from math import isfinite
from motorcan.helper import RepeatTimer


# the user-facing class that manages the motor.
class TMotorManager:
    """
    The user-facing class that manages the motor. This class should be
    used in the context of a with as block, in order to safely enter/exit
    control of the motor.
    """

    def __init__(
        self,
        motor_type: str,
        motor_ID: int,
        master_ID: int = 0,
        use_torque_compensation: bool = False,
    ):
        """
        Sets up the motor manager. Note the device will not be powered on by this method! You must
        call __enter__, mostly commonly by using a with block, before attempting to control the motor.

        Args:
            motor_type: The type of motor being controlled, ie AK80-9.
            motor_ID: The CAN ID of the motor.
            use_torque_compensation: Enables a more complex torque model to compensate for friction, if available
        """
        self.type = motor_type
        self.ID = motor_ID
        self.master_ID = master_ID

        self._motor_state = TMotorState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._command = MIT_command(0.0, 0.0, 0.0, 0.0, 0.0)
        self._control_state = TMotorControlState.IDLE
        self._times_past_position_limit = 0
        self._times_past_velocity_limit = 0
        self._angle_threshold = (
            MIT_Params[self.type]["P_max"] - 2.0
        )  # radians, only really matters if the motor's going super fast
        self._current_threshold = (
            self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"]) - 3.0
        )  # A, only really matters if the current changes quick
        self._velocity_threshold = (
            MIT_Params[self.type]["V_max"] - 2.0
        )  # radians, only really matters if the motor's going super fast

        self._last_update_time = time.time()
        self._last_command_time = self._last_update_time
        self.use_torque_compensation = use_torque_compensation
        self.SF = 1.0

        self.direction = 1.0
        self.encoder_direction = 1.0

        self._canman = CAN_Manager()
        self._canman.add_motor_listener(TMotorListener(self))

        self.power_on()
        self._send_command()

        # # start timer to request telemetry from the tmotors
        # # the power on function is used to just request state
        # # TODO: this causes weird buzzing and vibration during operation
        # # with frequent commands. find a way to stop a RESTARTABLE timer
        # # when we expect to start sending current commands, otherwise we
        # # won't be able to encoder feedback. TMotors send status back when
        # # they receive any command.
        # self.telemetry_timer = RepeatTimer(0.01, self.power_on)
        # self.telemetry_timer.start() # we want this to happen at the start so that at least we get some encoder feedback from T-motors before arming.

    # send the power on code
    def power_on(self) -> bool:
        """
        Sends the power on code to the motor.
        """
        result = self.send_MIT_message([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        return result

    # send the power off code
    def power_off(self) -> bool:
        """
        Sends the power off code to the motor.
        """
        # self.telemetry_timer.cancel()
        return self.send_MIT_message([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])

    # send the zeroing code. Like a scale, it takes about a second to zero the position
    def zero(self) -> bool:
        """
        Sends the zeroing code to the motor. This code will shut off communication with the motor for about a second.
        """
        self.send_MIT_message([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]) #230330 CW: enter motor control mode, maybe not doing this is causing some motors to react?
        result = self.send_MIT_message([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
        time.sleep(2.0)

        # send Tmotor a command so that we can get a response from it
        self.power_on()
        time.sleep(0.1) 

        result &= (abs(self.get_output_angle_radians()) < 3.1415/180)
        self._last_command_time = time.time()
        return result

    def set_control_mode(self, control_mode: TMotorControlState) -> bool:
        # if control_mode == TMotorControlState.CURRENT: self.telemetry_timer.cancel() # stops weird buzzing noises.
        self._control_state = control_mode
        return self._send_command()

    # uses plain impedance mode, will send 0.0 for current command.
    def set_impedance_gains_real_unit(self, kp=0, ki=0, K=0.08922, B=0.0038070, ff=0):
        """
        Uses plain impedance mode, will send 0.0 for current command in addition to position request.

        Args:
            kp: A dummy argument for backward compatibility with the dephy library.
            ki: A dummy argument for backward compatibility with the dephy library.
            K: The stiffness in Nm/rad
            B: The damping in Nm/(rad/s)
            ff: A dummy argument for backward compatibility with the dephy library.
        """
        assert (
            isfinite(K)
            and MIT_Params[self.type]["Kp_min"] <= K
            and K <= MIT_Params[self.type]["Kp_max"]
        )
        assert (
            isfinite(B)
            and MIT_Params[self.type]["Kd_min"] <= B
            and B <= MIT_Params[self.type]["Kd_max"]
        )
        self._command.kp = K
        self._command.kd = B
        self._command.velocity = 0.0

    # uses full MIT mode, will send whatever current command is set.
    def set_impedance_gains_real_unit_full_state_feedback(
        self, kp=0, ki=0, K=0.08922, B=0.0038070, ff=0
    ):
        """ "
        Uses full state feedback mode, will send whatever current command is set in addition to position request.

        Args:
            kp: A dummy argument for backward compatibility with the dephy library.
            ki: A dummy argument for backward compatibility with the dephy library.
            K: The stiffness in Nm/rad
            B: The damping in Nm/(rad/s)
            ff: A dummy argument for backward compatibility with the dephy library."""
        assert (
            isfinite(K)
            and MIT_Params[self.type]["Kp_min"] <= K
            and K <= MIT_Params[self.type]["Kp_max"]
        )
        assert (
            isfinite(B)
            and MIT_Params[self.type]["Kd_min"] <= B
            and B <= MIT_Params[self.type]["Kd_max"]
        )
        self._command.kp = K
        self._command.kd = B

    # uses plain current mode, will send 0.0 for position gains.
    def set_current_gains(self, kp=40, ki=400, ff=128, spoof=False):
        """
        Uses plain current mode, will send 0.0 for position gains in addition to requested current.

        Args:
            kp: A dummy argument for backward compatibility with the dephy library.
            ki: A dummy argument for backward compatibility with the dephy library.
            ff: A dummy argument for backward compatibility with the dephy library.
            spoof: A dummy argument for backward compatibility with the dephy library.
        """
        pass

    def set_speed_gains(self, kd=1.0):
        """
        Uses plain speed mode, will send 0.0 for position gain and for feed forward current.

        Args:
            kd: The gain for the speed controller. Control law will be (v_des - v_actual)*kd = iq
        """
        self._command.kd = kd

    # used for either impedance or MIT mode to set output angle
    def set_output_angle_radians(self, position) -> bool:
        """
        Used for either impedance or full state feedback mode to set output angle command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            position: The desired output position in rads
        """
        # position commands must be within a certain range :/
        # position = (np.abs(position) % MIT_Params[self.type]["P_max"])*np.sign(position) # this doesn't work because it will unwind itself!
        # CANNOT Control using impedance mode for angles greater than 12.5 rad!!
        if np.abs(position) >= MIT_Params[self.type]["P_max"]:
            raise RuntimeError(
                "Cannot control using impedance mode for angles with magnitude greater than "
                + str(MIT_Params[self.type]["P_max"])
                + "rad!"
            )

        if self._control_state not in [
            TMotorControlState.IMPEDANCE,
            TMotorControlState.FULL_STATE,
        ]:
            raise RuntimeError(
                "Attempted to send position command without gains for device "
                + self.device_info_string()
            )
        self._command.position = position * self.direction

        return self._send_command()

    def set_output_velocity_radians_per_second(self, velocity):
        """
        Used for either speed or full state feedback mode to set output velocity command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            velocity: The desired output speed in rad/s
        """
        if np.abs(velocity) >= MIT_Params[self.type]["V_max"]:
            raise RuntimeError(
                "Cannot control using speed mode for angles with magnitude greater than "
                + str(MIT_Params[self.type]["V_max"])
                + "rad/s!"
            )

        if self._control_state not in [
            TMotorControlState.SPEED,
            TMotorControlState.FULL_STATE,
        ]:
            raise RuntimeError(
                "Attempted to send speed command without gains for device "
                + self.device_info_string()
            )
        self._command.velocity = velocity * self.direction
        return self._send_command()

    # used for either current MIT mode to set current
    def set_motor_current_qaxis_amps(self, current) -> bool:
        """
        Used for either current or full state feedback mode to set current command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            current: the desired current in amps.
        """
        if self._control_state not in [
            TMotorControlState.CURRENT,
            TMotorControlState.FULL_STATE,
        ]:
            raise RuntimeError(
                "Attempted to send current command before entering current mode for device "
                + self.device_info_string()
            )
        self._command.current = current * self.direction
        return self._send_command()

    # used for either current or MIT Mode to set current, based on desired torque
    def set_output_torque_newton_meters(self, torque) -> bool:
        """
        Used for either current or MIT Mode to set current, based on desired torque.
        If a more complicated torque model is available for the motor, that will be used.
        Otherwise it will just use the motor's torque constant.

        Args:
            torque: The desired output torque in Nm.
        """
        if (
            MIT_Params[self.type]["Use_derived_torque_constants"]
            and self.use_torque_compensation
        ):
            a_hat = MIT_Params[self.type]["a_hat"]
            kt = MIT_Params[self.type]["Kt_actual"]
            gr = MIT_Params[self.type]["GEAR_RATIO"]
            ϵ = 1.0
            i = self.get_current_qaxis_amps()
            v = self.get_motor_velocity_radians_per_second()
            bias = -a_hat[0]
            friction = (
                self.SF * (v / (ϵ + np.abs(v))) * (a_hat[3] + a_hat[4] * np.abs(i))
            )
            torque_constant = gr * (a_hat[1] * kt - a_hat[2] * np.abs(i))
            Iq_des = (torque - bias + friction) / torque_constant
            return self.set_motor_current_qaxis_amps(Iq_des)
        else:
            return self.set_motor_current_qaxis_amps(
                (
                    torque
                    / MIT_Params[self.type]["Kt_actual"]
                    / MIT_Params[self.type]["GEAR_RATIO"]
                )
            )

    # motor-side functions to account for the gear ratio
    def set_motor_torque_newton_meters(self, torque) -> bool:
        """
        Version of set_output_torque that accounts for gear ratio to control motor-side torque

        Args:
            torque: The desired motor-side torque in Nm.
        """
        return self.set_output_torque_newton_meters(
            torque * MIT_Params[self.type]["Kt_actual"]
        )

    def set_motor_angle_radians(self, position) -> bool:
        """
        Wrapper for set_output_angle that accounts for gear ratio to control motor-side angle

        Args:
            position: The desired motor-side position in rad.
        """
        return self.set_output_angle_radians(
            position / (MIT_Params[self.type]["GEAR_RATIO"])
        )

    def set_motor_velocity_radians_per_second(self, velocity) -> bool:
        """
        Wrapper for set_output_velocity that accounts for gear ratio to control motor-side velocity

        Args:
            velocity: The desired motor-side velocity in rad/s.
        """
        return self.set_output_velocity_radians_per_second(
            velocity / (MIT_Params[self.type]["GEAR_RATIO"])
        )

        # getters for motor state

    def get_temperature_celsius(self) -> float:
        """
        Returns:
        The most recently updated motor temperature in degrees C.
        """
        return self._motor_state.temperature

    def get_motor_error_code(self) -> int:
        """
        Returns:
        The most recently updated motor error code.
        Note the program should throw a runtime error before you get a chance to read
        this value if it is ever anything besides 0.

        Codes:
        - 0 : 'No Error',
        - 1 : 'Over temperature fault',
        - 2 : 'Over current fault',
        - 3 : 'Over voltage fault',
        - 4 : 'Under voltage fault',
        - 5 : 'Encoder fault',
        - 6 : 'Phase current unbalance fault (The hardware may be damaged)'
        """
        return self._motor_state.error

    def get_current_qaxis_amps(self) -> float:
        """
        Returns:
        The most recently updated qaxis current in amps
        """
        return self._motor_state.current * self.direction

    def get_output_angle_radians(self) -> float:
        """
        Returns:
        The most recently updated output angle in radians
        """
        return self._motor_state.position * self.encoder_direction

    def get_output_velocity_radians_per_second(self) -> float:
        """
        Returns:
            The most recently updated output velocity in radians per second
        """
        return self._motor_state.velocity * self.encoder_direction

    def get_output_acceleration_radians_per_second_squared(self) -> float:
        """
        Returns:
            The most recently updated output acceleration in radians per second per second
        """
        return self._motor_state.acceleration * self.encoder_direction

    def get_output_torque_newton_meters(self) -> float:
        """
        Returns:
            the most recently updated output torque in Nm
        """
        if (
            MIT_Params[self.type]["Use_derived_torque_constants"]
            and self.use_torque_compensation
        ):
            a_hat = MIT_Params[self.type]["a_hat"]
            kt = MIT_Params[self.type]["Kt_actual"]
            gr = MIT_Params[self.type]["GEAR_RATIO"]
            ϵ = 0.1
            i = self.get_current_qaxis_amps()
            v = self.get_motor_velocity_radians_per_second()

            return (
                a_hat[0]
                + a_hat[1] * gr * kt * i
                - a_hat[2] * gr * np.abs(i) * i
                - a_hat[3] * np.sign(v) * (np.abs(v) / (ϵ + np.abs(v)))
                - a_hat[4] * np.abs(i) * np.sign(v) * (np.abs(v) / (ϵ + np.abs(v)))
            )
        else:
            return (
                self.get_current_qaxis_amps()
                * MIT_Params[self.type]["Kt_actual"]
                * MIT_Params[self.type]["GEAR_RATIO"]
            )

    def get_motor_angle_radians(self) -> float:
        """
        Wrapper for get_output_angle that accounts for gear ratio to get motor-side angle

        Returns:
            The most recently updated motor-side angle in rad.
        """
        return self._motor_state.position * MIT_Params[self.type]["GEAR_RATIO"]

    def get_motor_velocity_radians_per_second(self) -> float:
        """
        Wrapper for get_output_velocity that accounts for gear ratio to get motor-side velocity

        Returns:
            The most recently updated motor-side velocity in rad/s.
        """
        return self._motor_state.velocity * MIT_Params[self.type]["GEAR_RATIO"]

    def get_motor_acceleration_radians_per_second_squared(self) -> float:
        """
        Wrapper for get_output_acceleration that accounts for gear ratio to get motor-side acceleration

        Returns:
            The most recently updated motor-side acceleration in rad/s/s.
        """
        return self._motor_state.acceleration * MIT_Params[self.type]["GEAR_RATIO"]

    def get_motor_torque_newton_meters(self) -> float:
        """
        Wrapper for get_output_torque that accounts for gear ratio to get motor-side torque

        Returns:
            The most recently updated motor-side torque in Nm.
        """
        return (
            self.get_output_torque_newton_meters() * MIT_Params[self.type]["GEAR_RATIO"]
        )

        # this method is called by the handler every time a message is recieved on the bus

    # from this motor, to store the most recent state information for later
    def _update_state_async(self, MIT_state: MIT_motor_state):
        """
        This method is called by the handler every time a message is recieved on the bus
        from this motor, to store the most recent state information for later

        Args:
            MIT_state: The MIT_Motor_State namedtuple with the most recent motor state.

        Raises:
            RuntimeError when device sends back an error code that is not 0 (0 meaning no error)
        """
        if MIT_state.error is not None:
            if MIT_state.error != 0:
                raise RuntimeError(
                    "Driver board error for device: "
                    + self.device_info_string()
                    + ": "
                    + MIT_Params["ERROR_CODES"][MIT_state.error]
                )

        now = time.time()
        dt = self._last_update_time - now
        self._last_update_time = now
        acceleration = (MIT_state.velocity - self._motor_state.velocity) / dt

        # The "Current" supplied by the controller is actually current*Kt, which approximates torque.
        new_motor_state = TMotorState(
            MIT_state.position,
            MIT_state.velocity,
            self.TMotor_current_to_qaxis_current(MIT_state.current),
            MIT_state.temperature,
            MIT_state.error,
            acceleration,
        )

        self._times_past_position_limit += self._check_past_pos_limit(
            self._motor_state.position, new_motor_state.position
        )
        self._times_past_velocity_limit += self._check_past_vel_limit(
            self._motor_state.velocity, new_motor_state.velocity
        )
        actual_current = self._check_past_curr_limit(
            self._motor_state.current, new_motor_state.current, self._command.current
        )

        self._motor_state = new_motor_state

        # add on number of times wrapped to motor state
        self._motor_state.position += (
            self._times_past_position_limit * 2 * MIT_Params[self.type]["P_max"]
        )
        self._motor_state.current = actual_current
        self._motor_state.velocity += (
            self._times_past_velocity_limit * 2 * MIT_Params[self.type]["V_max"]
        )

    def _check_past_pos_limit(self, old_pos: float, new_pos: float) -> int:
        # artificially extending the range of the position, current, and velocity that we track
        P_max = MIT_Params[self.type]["P_max"] + 0.01

        # The TMotor will wrap around to -max at the limits for all values it returns!! Account for this
        if (self._angle_threshold <= new_pos and new_pos <= P_max) and (
            -P_max <= old_pos and old_pos <= -self._angle_threshold
        ):
            return -1
        elif (self._angle_threshold <= old_pos and old_pos <= P_max) and (
            -P_max <= new_pos and new_pos <= -self._angle_threshold
        ):
            return 1
        return 0

    def _check_past_vel_limit(self, old_vel: float, new_vel: float) -> int:
        V_max = MIT_Params[self.type]["V_max"] + 0.01
        # velocity should work the same as position
        if (self._velocity_threshold <= new_vel and new_vel <= V_max) and (
            -V_max <= old_vel and old_vel <= -self._velocity_threshold
        ):
            return -1
        elif (self._velocity_threshold <= old_vel and old_vel <= V_max) and (
            -V_max <= new_vel and new_vel <= -self._velocity_threshold
        ):
            return 1

        return 0

    def _check_past_curr_limit(
        self, old_curr: float, new_curr: float, curr_command: float
    ) -> float:
        I_max = (
            self.TMotor_current_to_qaxis_current(MIT_Params[self.type]["T_max"]) + 1.0
        )

        actual_current = new_curr

        # current is basically the same as position, but if you instantly command a switch it can actually change fast enough
        # to throw this off, so that is accounted for too. We just put a hard limit on the current to solve current jitter problems.
        if (self._current_threshold <= new_curr and new_curr <= I_max) and (
            -I_max <= old_curr and old_curr <= -self._current_threshold
        ):

            if curr_command > 0:
                actual_current = self.TMotor_current_to_qaxis_current(
                    MIT_Params[self.type]["T_max"]
                )
            else:
                actual_current = -self.TMotor_current_to_qaxis_current(
                    MIT_Params[self.type]["T_max"]
                )
            new_curr = actual_current
        elif (self._current_threshold <= old_curr and old_curr <= I_max) and (
            -I_max <= new_curr and new_curr <= -self._current_threshold
        ):

            if curr_command < 0:
                actual_current = -self.TMotor_current_to_qaxis_current(
                    MIT_Params[self.type]["T_max"]
                )
            else:
                actual_current = self.TMotor_current_to_qaxis_current(
                    MIT_Params[self.type]["T_max"]
                )
        return actual_current

    # sends a command to the motor depending on whats controlm mode the motor is in
    def _send_command(self) -> bool:
        """
        Sends a command to the motor depending on whats controlm mode the motor is in. This method
        is called by update(), and should only be called on its own if you don't want to update the motor state info.

        Notably, the current is converted to amps from the reported 'torque' value, which is i*Kt.
        This allows control based on actual q-axis current, rather than estimated torque, which
        doesn't account for friction losses.
        """
        now = time.time()
        result = False
        if self._control_state == TMotorControlState.FULL_STATE:
            result = self.MIT_controller(
                self.type,
                self._command.position,
                self._command.velocity,
                self._command.kp,
                self._command.kd,
                self.qaxis_current_to_TMotor_current(self._command.current),
            )
        elif self._control_state == TMotorControlState.IMPEDANCE:
            result = self.MIT_controller(
                self.type,
                self._command.position,
                self._command.velocity,
                self._command.kp,
                self._command.kd,
                0.0,
            )
        elif self._control_state == TMotorControlState.CURRENT:
            result = self.MIT_controller(
                self.type,
                0.0,
                0.0,
                0.0,
                0.0,
                self.qaxis_current_to_TMotor_current(self._command.current),
            )
        elif self._control_state == TMotorControlState.IDLE:
            result = self.MIT_controller(self.type, 0.0, 0.0, 0.0, 0.0, 0.0)
        elif self._control_state == TMotorControlState.SPEED:
            result = self.MIT_controller(
                self.type,
                0.0,
                self._command.velocity,
                0.0,
                self._command.kd,
                0.0,
            )
        else:
            raise RuntimeError(
                "UNDEFINED STATE for device " + self.device_info_string()
            )
        self._last_command_time = time.time()
        return result

    def device_info_string(self) -> str:
        """Prints the motor's ID and device type."""
        return str(self.type) + "  ID: " + str(self.ID)

    # Checks the motor connection by sending a 10 commands and making sure the motor responds.
    def check_can_connection(self) -> bool:
        """
        Checks the motor's connection by attempting to send 10 startup messages.
        If it gets 10 replies, then the connection is confirmed.

        Returns:
            True if a connection is established and False otherwise.
        """
        Listener = BufferedReader()
        self._canman.notifier.add_listener(Listener)
        for i in range(10):
            self.power_on()
            time.sleep(0.001)
        success = True
        time.sleep(0.1)
        for i in range(10):
            if Listener.get_message(timeout=0.1) is None:
                success = False
        self._canman.notifier.remove_listener(Listener)
        return success

    # Locks value between min and max
    @staticmethod
    def limit_value(value: int, min: int, max: int) -> int:
        """
        Limits value to be between min and max

        Args:
            value: The value to be limited.
            min: The lowest number allowed (inclusive) for value
            max: The highest number allowed (inclusive) for value
        """
        if value >= max:
            return max
        elif value <= min:
            return min
        else:
            return value

    # interpolates a floating point number to fill some amount of the max size of unsigned int,
    # as specified with the num_bits
    @staticmethod
    def float_to_uint(x: float, x_min: float, x_max: float, num_bits: int) -> int:
        """
        Interpolates a floating point number to an unsigned integer of num_bits length.
        A number of x_max will be the largest integer of num_bits, and x_min would be 0.

        args:
            x: The floating point number to convert
            x_min: The minimum value for the floating point number
            x_max: The maximum value for the floating point number
            num_bits: The number of bits for the unsigned integer
        """
        span = x_max - x_min
        bitratio = float((1 << num_bits) / span)
        x = TMotorManager.limit_value(x, x_min, x_max - (2 / bitratio))
        # (x - x_min)*(2^num_bits)/span

        return TMotorManager.limit_value(
            int((x - x_min) * (bitratio)), 0, int((x_max - x_min) * bitratio)
        )

    # undoes the above method
    @staticmethod
    def uint_to_float(x: int, x_min: float, x_max: float, num_bits: int) -> float:
        """
        Interpolates an unsigned integer of num_bits length to a floating point number between x_min and x_max.

        args:
            x: The floating point number to convert
            x_min: The minimum value for the floating point number
            x_max: The maximum value for the floating point number
            num_bits: The number of bits for the unsigned integer
        """
        span = x_max - x_min
        # (x*span/(2^num_bits -1)) + x_min
        return float(x * span / ((1 << num_bits) - 1) + x_min)

        # sends a message to the motor (when the motor is in MIT mode)

    def TMotor_current_to_qaxis_current(self, iTM) -> float:
        return (
            MIT_Params[self.type]["Current_Factor"]
            * iTM
            / (MIT_Params[self.type]["GEAR_RATIO"] * MIT_Params[self.type]["Kt_TMotor"])
        )

    def qaxis_current_to_TMotor_current(self, iq) -> float:
        return (
            iq
            * (MIT_Params[self.type]["GEAR_RATIO"] * MIT_Params[self.type]["Kt_TMotor"])
            / MIT_Params[self.type]["Current_Factor"]
        )

    def send_MIT_message(self, data: "list[int]") -> bool:
        """
        Sends an MIT Mode message to the motor with data array of data

        Args:
            data: An array of integers or bytes of data to send.
        """
        assert len(data) <= 8, "Data too long in message for motor " + str(self.ID)

        message = Message(arbitration_id=self.ID, data=data, is_extended_id=False)

        return self._canman.send(message)

    # send an MIT control signal, consisting of desired position, velocity, and current, and gains for position and velocity control
    # basically an impedance controller
    def MIT_controller(self, motor_type, position, velocity, Kp, Kd, I) -> bool:
        """
        Sends an MIT style control signal to the motor. This signal will be used to generate a
        current for the field-oriented controller on the motor control chip, given by this expression:

            q_control = Kp*(position - current_position) + Kd*(velocity - current_velocity) + I

        Args:
            motor_id: The CAN ID of the motor to send the message to
            motor_type: A string noting the type of motor, ie 'AK80-9'
            position: The desired position in rad
            velocity: The desired velocity in rad/s
            Kp: The position gain
            Kd: The velocity gain
            I: The additional current
        """
        position_uint16 = TMotorManager.float_to_uint(
            position,
            MIT_Params[motor_type]["P_min"],
            MIT_Params[motor_type]["P_max"],
            16,
        )
        velocity_uint12 = TMotorManager.float_to_uint(
            velocity,
            MIT_Params[motor_type]["V_min"],
            MIT_Params[motor_type]["V_max"],
            12,
        )
        Kp_uint12 = TMotorManager.float_to_uint(
            Kp, MIT_Params[motor_type]["Kp_min"], MIT_Params[motor_type]["Kp_max"], 12
        )
        Kd_uint12 = TMotorManager.float_to_uint(
            Kd, MIT_Params[motor_type]["Kd_min"], MIT_Params[motor_type]["Kd_max"], 12
        )
        I_uint12 = TMotorManager.float_to_uint(
            I, MIT_Params[motor_type]["T_min"], MIT_Params[motor_type]["T_max"], 12
        )

        data = [
            position_uint16 >> 8,
            position_uint16 & 0x00FF,
            (velocity_uint12) >> 4,
            ((velocity_uint12 & 0x00F) << 4) | (Kp_uint12) >> 8,
            (Kp_uint12 & 0x0FF),
            (Kd_uint12) >> 4,
            ((Kd_uint12 & 0x00F) << 4) | (I_uint12) >> 8,
            (I_uint12 & 0x0FF),
        ]
        return self.send_MIT_message(data)

    def invert_direction(self) -> bool:
        self.direction = -self.direction

    def invert_encoder(self) -> bool:
        self.encoder_direction = -self.encoder_direction


# python-can listener object, with handler to be called upon reception of a message on the CAN bus
class TMotorListener(Listener):
    """Python-can listener object, with handler to be called upon reception of a message on the CAN bus"""

    def __init__(self, motor: TMotorManager):
        """
        Sets stores can manager and motor object references

        Args:
            canman: The CanManager object to get messages from
            motor: The TMotorCANManager object to update
        """
        self.motor = motor

    def on_message_received(self, msg: Message):
        """
        Updates this listener's motor with the info contained in msg, if that message was for this motor.

        args:
            msg: A python-can CAN message
        """

        if msg.arbitration_id == self.motor.master_ID:
            data = bytes(msg.data)
            ID = data[0]

            if ID == self.motor.ID:
                self.motor._update_state_async(self.parse_MIT_message(data))

        # convert data recieved from motor in byte format back into floating point numbers in real units

    def parse_MIT_message(self, data: bytes) -> MIT_motor_state:
        """
        Takes a RAW MIT message and formats it into readable floating point numbers.

        Args:
            data: the bytes of data from a python-can message object to be parsed
            motor_type: A string noting the type of motor, ie 'AK80-9'

        Returns:
            An MIT_Motor_State namedtuple that contains floating point values for the
            position, velocity, current, temperature, and error in rad, rad/s, amps, and *C.
            0 means no error.

            Notably, the current is converted to amps from the reported
            'torque' value, which is i*Kt. This allows control based on actual q-axis current,
            rather than estimated torque, which doesn't account for friction losses.
        """
        assert (
            len(data) == 8 or len(data) == 6
        ), "Tried to parse a CAN message that was not Motor State in MIT Mode"
        temp = None
        error = None
        position_uint = data[1] << 8 | data[2]
        velocity_uint = ((data[3] << 8) | (data[4] >> 4) << 4) >> 4
        current_uint = (data[4] & 0x0F) << 8 | data[5]

        if len(data) == 8:
            temp = int(data[6])
            error = int(data[7])

        position = TMotorManager.uint_to_float(
            position_uint,
            MIT_Params[self.motor.type]["P_min"],
            MIT_Params[self.motor.type]["P_max"],
            16,
        )
        velocity = TMotorManager.uint_to_float(
            velocity_uint,
            MIT_Params[self.motor.type]["V_min"],
            MIT_Params[self.motor.type]["V_max"],
            12,
        )
        current = TMotorManager.uint_to_float(
            current_uint,
            MIT_Params[self.motor.type]["T_min"],
            MIT_Params[self.motor.type]["T_max"],
            12,
        )

        # returns the Tmotor "current" which is really a torque estimate
        return MIT_motor_state(position, velocity, current, temp, error)
