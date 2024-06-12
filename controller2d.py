#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('integral_previous', 0.0)
        self.vars.create_var('previous_error', 0.0)


        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
          """
        # PID controller for longitudinal contol
        #--------------------------------------------------------------------------
            kp = 1.0
            ki = 0.1 
            kd = 0.05 

            error = v_desired - v
            delta_time = t - self.vars.t_previous

            #Proportional Term
            proportional_term = kp * error

            #Integral Term
            integral = self.vars.integral_previous + error * delta_time 
            integral_term = ki * integral

            #Derivative Term
            derivative = (error - self.vars.previous_error) / delta_time
            derivative_term = kd * derivative

            # acceleration (PID output)
            acc = proportional_term + integral_term + derivative_term 

            # Convert accelartion to throttle
            max_acc = 4.5
            throttle_output = max(0,min(max_acc , acc)) / max_acc
            # print(" Desired v =",v_desired, " error " , error, " throttle = ", throttle_output)
            print()
            print(" velocity ",v ," Desired v ",v_desired, " error " , error, " throttle = ", throttle_output)

        # -------------------------------------------------------------------------
            
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            # throttle_output = 0
            brake_output    = 0

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            # Stanley controlller for steering angle
            #---------------------------------------------------------------------

            #Calclate distance from car to each waypint
            waypoints = np.asarray(waypoints)
            dx = waypoints[:, 0] - x
            dy = waypoints[:, 1] - y
            distances = np.sqrt(dx**2 + dy**2)

            #Find shortest one 
            nearest_index = np.argmin(distances)

            # I will use two poins to find path heading, the closest path point to my car and another one ( the previous or the next one )
            previous_waypoint = waypoints[nearest_index]

            if nearest_index < len(waypoints) -1 :
                next_waypoint = waypoints[nearest_index+1]
            else:
                next_waypoint = previous_waypoint
                previous_waypoint = waypoints[nearest_index-1]

            
            # Heading Error Calcualtion
            path_heading = np.arctan2( next_waypoint[1]-previous_waypoint[1] , next_waypoint[0]-previous_waypoint[0] )
            heading_error = path_heading - yaw
            # Normalize the heading error to make i between pi and - pi
            heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

            # Cross Track Error
            x1, y1, _ = previous_waypoint
            x2, y2, _ = next_waypoint
            xc, yc = np.array([x,y])
            
            # Calculate the coefficients of the line ax + by + c = 0
            a = y2 - y1
            b = x1 - x2
            c = x2 * y1 - x1 * y2
            
            # Calculate the cross-track error
            cte = (a * xc + b * yc + c) / np.sqrt(a**2 + b**2)

            k = 0.5 

            # Stanley lateral control
            stering_angle = heading_error + np.arctan(k * cte / v)
            #limited steering to +-1.22
            stering_angle = max ( -1.22 , min(1.22 , stering_angle) )

            print(" Yaw ",yaw ," path_heading ",path_heading, " stering_angle " , stering_angle)

            #---------------------------------------------------------------------
            
            # Change the steer output with the lateral controller. 
            steer_output    = stering_angle

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step

        # ------------------------------------------------------------------
        self.vars.previous_error = error
        self.vars.integral_previous = integral
        self.vars.t_previous = t
        # ------------------------------------------------------------------
