"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import time
import numpy as np
from frame_utils import euler2RM

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1.0
MAX_TILT_ANGLE = 0.8

class NonlinearController(object):

    def __init__(self, z_k_p=4.0,
                z_k_d=2.5,
                x_k_p=5.0,
                x_k_d=4.0,
                y_k_p=5.0,
                y_k_d=4.0,
                k_p_roll=6.0,
                k_p_pitch=6.0,
                k_p_yaw=4.0,
                k_p_p=25.0,
                k_p_q=25.0,
                k_p_r=6.0):

        """Initialize the controller object and control gains"""
        self.z_k_p = z_k_p
        self.z_k_d = z_k_d
        self.x_k_p = x_k_p
        self.x_k_d = x_k_d
        self.y_k_p = y_k_p
        self.y_k_d = y_k_d
        self.k_p_roll = k_p_roll
        self.k_p_pitch = k_p_pitch
        self.k_p_yaw = k_p_yaw
        self.k_p_p = k_p_p
        self.k_p_q = k_p_q
        self.k_p_r = k_p_r
        """Initialize the controller object and control gains"""

    def trajectory_control(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        """Generate a commanded position, velocity and yaw based on the trajectory

        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the position and yaw commands
            current_time: float corresponding to the current time in seconds

        Returns: tuple (commanded position, commanded velocity, commanded yaw)

        """

        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]


        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]

            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]

        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]

                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]

        position_cmd = (position1 - position0) * \
                        (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)

        return (position_cmd, velocity_cmd, yaw_cmd)

    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                               acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command

        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """

        # x lateral position PD controller
        x_err = local_position_cmd[0] - local_position[0]
        x_err_dot = local_velocity_cmd[0] - local_velocity[0]

        p_term_x = self.x_k_p * x_err
        d_term_x = self.x_k_d * x_err_dot

        x_dot_dot_command = p_term_x + d_term_x + acceleration_ff[0]

        # y lateral position PD controller
        y_err = local_position_cmd[1] - local_position[1]
        y_err_dot = local_velocity_cmd[1] - local_velocity[1]

        p_term_y = self.y_k_p * y_err
        d_term_y = self.y_k_d * y_err_dot

        y_dot_dot_command = p_term_y + d_term_y + acceleration_ff[1]

        return np.array([x_dot_dot_command, y_dot_dot_command])

    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            attitude: the vehicle's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
            acceleration_ff: feedforward acceleration command (+up)

        Returns: thrust command for the vehicle (+up)
        """

        # Vertical position and velocity errors
        z_err = altitude_cmd - altitude
        z_err_dot = vertical_velocity_cmd - vertical_velocity

        p_term = self.z_k_p * z_err
        d_term = self.z_k_d * z_err_dot

        # PD altitude controller with feed forward acceleration
        u_1_bar = p_term + d_term + acceleration_ff

        rot_mat = euler2RM(attitude[0], attitude[1], attitude[2])
        b_z = rot_mat[2, 2]

        #c = (u_1_bar + GRAVITY) / b_z

        #Convert linear acceleration to Thrust
        c = (u_1_bar) / b_z
        thrust = c*DRONE_MASS_KG

        return (np.clip(thrust, 0, MAX_THRUST))

    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame

        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thruts command in Newton

        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """

        b_x_c_target = -np.clip(acceleration_cmd[0] / (thrust_cmd / DRONE_MASS_KG), -MAX_TILT_ANGLE, MAX_TILT_ANGLE)
        b_y_c_target = -np.clip(acceleration_cmd[1] / (thrust_cmd / DRONE_MASS_KG), -MAX_TILT_ANGLE, MAX_TILT_ANGLE)

        #b_x_c_target = -acceleration_cmd[0] / (thrust_cmd / DRONE_MASS_KG)
        #b_y_c_target = -acceleration_cmd[1] / (thrust_cmd / DRONE_MASS_KG)

        # Calculate rotation matrix
        rot_mat = euler2RM(attitude[0], attitude[1], attitude[2])

        # Roll P Controller
        b_x = rot_mat[0, 2]
        b_x_err = b_x_c_target - b_x
        b_x_p_term = self.k_p_roll * b_x_err

        # Pitch P Controller
        b_y = rot_mat[1, 2]
        b_y_err = b_y_c_target - b_y
        b_y_p_term = self.k_p_pitch * b_y_err

        b_x_commanded_dot = b_x_p_term
        b_y_commanded_dot = b_y_p_term

        # Account for non-linear transformations from local accelerations to body rates
        rot_mat1 = np.array([[rot_mat[1, 0], -rot_mat[0, 0]], [rot_mat[1, 1], -rot_mat[0, 1]]]) / rot_mat[2, 2]

        rot_rate = np.matmul(rot_mat1, np.array([b_x_commanded_dot, b_y_commanded_dot]).T)
        p_c = rot_rate[0]
        q_c = rot_rate[1]

        return np.array([p_c, q_c])


    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame

        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2

        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """

        # P controller for roll in Newtons*meters
        p_err = body_rate_cmd[0] - body_rate[0]
        u_bar_p = MOI[0]*self.k_p_p * p_err

        # P controller for pitch in Newtons*meters
        q_err = body_rate_cmd[1] - body_rate[1]
        u_bar_q = MOI[1]*self.k_p_q * q_err

        # P controller for yaw in Newtons*meters
        r_err = body_rate_cmd[2] - body_rate[2]
        u_bar_r = MOI[2]*self.k_p_r * r_err

        #Clip torque to MAX_TORQUE
        return np.array([np.clip(u_bar_p,-MAX_TORQUE, MAX_TORQUE), np.clip(u_bar_q, -MAX_TORQUE, MAX_TORQUE), np.clip(u_bar_r, -MAX_TORQUE, MAX_TORQUE)])

    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate

        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians

        Returns: target yawrate in radians/sec
        """

        # Target is within range of 0 to 2*pi
        yaw_cmd = np.mod(yaw_cmd, 2.0 * np.pi)

        psi_err = yaw_cmd - yaw
        if psi_err > np.pi:
            psi_err = psi_err - 2.0 * np.pi
        elif psi_err < -np.pi:
            psi_err = psi_err + 2.0 * np.pi

        # P controller for yaw
        r_c = self.k_p_yaw * psi_err

        return r_c



