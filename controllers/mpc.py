from . import BaseController
import numpy as np
import cvxpy as cp

class Controller(BaseController):
  def __init__(self, horizon=5, Q=np.eye(1), R=np.eye(1)):
    self.B = np.array([[0.5]])
    self.horizon = horizon
    self.Q = Q
    self.R = R
    
  def update(self, target_lataccel, current_lataccel, state):
      x = cp.Variable((1, self.horizon + 1))
      u = cp.Variable((1, self.horizon))
      cost = 0
      constraints = [x[:, 0] == current_lataccel] # NOTE first state == current_lataccel
      for t in range(self.horizon):
          cost += cp.quad_form(x[:, t] - target_lataccel, self.Q) + cp.quad_form(u[:, t], self.R)
          constraints += [x[:, t + 1] == self.B @ u[:, t]]
      cost += cp.quad_form(x[:, self.horizon] - target_lataccel, self.Q)
      # Define the optimization problem
      problem = cp.Problem(cp.Minimize(cost), constraints)
      # Solve the optimization problem
      problem.solve()
      # Extract the optimal control input
      optimal_u = u.value
      # Implement the first control action
      u0 = optimal_u[0]
      print(u0[0])
      return u0[0]
