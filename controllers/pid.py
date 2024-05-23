from . import BaseController


class Controller(BaseController):
  """
  A PID controller that adjusts the control output based on the error between the target and current lateral acceleration
  """
  def __init__(self, kp=1.0, ki=1.0, kd=1.0, max_acc_errors=5):
    self.kp = kp  # Proportional gain
    self.ki = ki  # Integral gain
    self.kd = kd  # Derivative gain
    self.error_sum = 0  # Accumulated error
    self.amount_of_errors = 0  
    self.prev_error = 0  # Previous error
    self.max_acc_errors = max_acc_errors
    
  def update(self, target_lataccel, current_lataccel, state):
    error = target_lataccel - current_lataccel  # Calculate the error
    if self.amount_of_errors > self.max_acc_errors:
      self.error_sum = 0
      self.amount_of_errors = 0
      
    self.error_sum += error  # Accumulate the error
    self.amount_of_errors += 1
    
    error_diff = error - self.prev_error  # Calculate the error difference
    self.prev_error = error  # Update the previous error

    p = self.kp * error  # Proportional term
    i = self.ki * self.error_sum  # Integral term
    d = self.kd * error_diff  # Derivative term
    
    control_output = (p + i + d) * 0.03
    return control_output
