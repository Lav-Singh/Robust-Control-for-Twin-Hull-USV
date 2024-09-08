#!/usr/bin/env python3

import queue
import numpy as np

################################################################################

class PIDController:
    '''
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    '''
    def __init__(self, kP, kI, kD, kS):
        self.kP       = kP # Proportional gain
        self.kI       = kI # Integral gain
        self.kD       = kD # Derivative gain
        self.kS       = kS # Saturation constant (error history buffer size)
        self.err_int  = 0 # Error integral
        self.err_dif  = 0 # Error difference
        self.err_prev = 0 # Previous error
        self.err_hist = queue.Queue(self.kS) # Limited buffer of error history
        self.t_prev   = 0 # Previous time

    def control(self, err, t):
        '''
        Generate PID controller output.
        :param err: Instantaneous error in control variable w.r.t. setpoint
        :param t  : Current timestamp
        :return u: PID controller output
        '''
        dt = t - self.t_prev # Timestep
        if dt > 0.0:
            self.err_hist.put(err)
            self.err_int += err
            if self.err_hist.full(): # Jacketing logic to prevent integral windup
                self.err_int -= self.err_hist.get() # Rolling FIFO buffer
            self.err_dif = (err - self.err_prev)
            u = (self.kP * err) + (self.kI * self.err_int * dt) + (self.kD * self.err_dif / dt)
            self.err_prev = err
            self.t_prev = t
            return u # Control signal
 


# Kalman Filter for Angular Velocity
class KalmanFilter:
    def __init__(self,dt):
        # State vector [angular_velocity, angular_acceleration]
        self.x = np.array([[0], [0]])  # Initial state: zero angular velocity and acceleration

        # Covariance matrix
        self.P = np.eye(2) * 0.1  # Small initial uncertainty

        # Process noise covariance matrix
        self.Q = np.array([[0.001, 0], [0, 0.001]])  # Process noise for velocity and acceleration

        # Measurement noise covariance matrix
        self.R = np.array([[0.1]])  # Measurement noise for angular velocity

        # State transition matrix
        self.A = np.array([[1, dt], [0, 1]])  # Discretized state transition model

        # Measurement matrix
        self.H = np.array([[1, 0]])  # We only measure angular velocity (not acceleration)

        # Control input matrix (for angular acceleration)
        self.B = np.array([[0], [1]])  # Control input affects angular acceleration

    def predict(self, control_input):
        # Predict the next state
        self.x = self.A @ self.x + self.B * control_input

        # Predict the next covariance
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, measurement):
        # Compute the Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)

        # Update the state with the measurement
        self.x = self.x + K @ (measurement - self.H @ self.x)

        # Update the covariance matrix
        self.P = (np.eye(2) - K @ self.H) @ self.P

        return self.x[0, 0]  # Return the filtered angular velocity