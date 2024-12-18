# DDE_IVP

[![PyPI version](https://badge.fury.io/py/dde_ivp.svg)](https://badge.fury.io/py/dde_ivp)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A Python package for solving Delay Differential Equations (DDEs) with custom event handling and interpolation. This package provides a simple and efficient interface for DDE problems using advanced numerical methods.

Inspired by the **`ddeint`** package, this implementation builds upon `scipy.integrate.solve_ivp`, which offers modern numerical solvers such as `RK23` and `RK45` with adaptive step size. The package also allows configurable maximum step size. (Note: the step size should not exceed the minimum delay to avoid extrapolation and instability).

---

## Features

- Solve delay differential equations with ease.
- Event handling for detecting zero-crossings and termination.
- Built-in support for numerical methods like `RK23` and `RK45`.
- Compatible with SciPy's `solve_ivp` interface for familiarity.
- Adaptive step size and configurable maximum step size for precision and stability.

---


## Installation

Install the package directly from PyPI:

```bash
pip install dde_ivp
```

Or install the latest development version from GitHub:

```bash
pip install git+https://https://github.com/Menginventor/dde_ivp.git
```

---

## Usage

### Example: Solving a Delay Differential Equation

Here's an example of solving an IPDT (Integrating Plus Dead Time) model with proportional control:

```python
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

def ipdt_with_p_control(t, Y):
    y_delayed = Y(t - L)
    u = Kp * (R - y_delayed)
    return (K * u) / T


# History function
def history(t):
    return np.array([0])   # Assume initial output is zero

def zc_event(t,y):
    return y[0]-1
# Time span

t_span = (0,60)
t = np.linspace(t_span[0], t_span[1], 1000)
# Solve the DDE
solution = solve_ddeivp(ipdt_with_p_control, t_span, history,max_step=1.0,method='RK23',events=zc_event)

print(solution)

# Plot results
plt.figure(figsize=(10, 6))
plt.plot(t, solution.sol(t)[0], label="System Output (y(t))")
plt.plot(solution.t, solution.y[0],'.', label="System Output (y(t))")
plt.plot(solution.t_events[0],solution.y_events[0],'x')
plt.axhline(R, color='r', linestyle='--', label="Reference Input (R)")
plt.title("IPDT Model with Proportional Control")
plt.xlabel("Time")
plt.ylabel("Output")
plt.legend()
plt.grid()
plt.show()

```

---

## API Reference

### `solve_ddeivp`

```python
solve_ddeivp(fun, t_span, history_func, method='RK45', t_eval=None, max_step=None, events=None, vectorized=False, args=None, **options)
```

Solve delay differential equations with event handling.

- **`fun`**: Callable. The system of differential equations.
- **`t_span`**: Tuple. Start and end times.
- **`history_func`**: Callable. Provides the initial condition or history.
- **`method`**: String. Numerical method to use (`RK23` or `RK45`).
- **`t_eval`**: Array. Time points to evaluate the solution.
- **`events`**: Callable or list of callables. Event functions for detecting zero-crossings.
- **`vectorized`**: Bool. Whether `fun` is vectorized.

Returns:
- An `OptimizeResult` object containing the solution, events, and status.

---

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Make your changes and test them.
4. Submit a pull request.

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---

## Acknowledgements

This package leverages numerical methods and optimizations from SciPy's `solve_ivp` and related tools.

---

## Contact

For questions or feedback, open an issue on the [GitHub repository](https://github.com/Menginventor/dde_ivp).
