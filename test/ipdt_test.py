import numpy as np
from dde_ivp import solve_ddeivp

import matplotlib.pyplot as plt


# System parameters
K = 1.0  # Process gain
T = 1.0  # Process time constant
L = 1.0  # Dead time
Kp = np.pi/2.0*L  # Proportional gain

# Reference input (step response)
R = 1.0

# Define the DDE
def ipdt_with_p_control(t,Y):
    # Current output
    #print(Y)
    #y = Y(t)

    # Delayed output (due to dead time L)
    y_delayed = Y(t - L)
    #print(Y,Y(t))

    #print('y_delayed',y_delayed)
    # Control action
    u = Kp * (R - y_delayed)

    # Process dynamics
    return (K * u) / T


# History function
def history(t):
    return np.array([0])   # Assume initial output is zero


# Time span

t_span = (0,60)
t = np.linspace(t_span[0], t_span[1], 1000)
# Solve the DDE
solution = solve_ddeivp(ipdt_with_p_control, t_span, history,max_step=1.0,method='RK23')

print(solution.sol([0,1]))

# Plot results
plt.figure(figsize=(10, 6))
plt.plot(t, solution.sol(t)[0], label="System Output (y(t))")
plt.plot(solution.t, solution.y[0], label="System Output (y(t))")
plt.axhline(R, color='r', linestyle='--', label="Reference Input (R)")
plt.title("IPDT Model with Proportional Control")
plt.xlabel("Time")
plt.ylabel("Output")
plt.legend()
plt.grid()
plt.show()
