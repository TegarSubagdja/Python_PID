class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # Konstanta Proportional
        self.Ki = Ki  # Konstanta Integral
        self.Kd = Kd  # Konstanta Derivatif

        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt != 0 else 0

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        self.prev_error = error
        return output
