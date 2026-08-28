# NMPC Takeover & Drag Racing

Nonlinear Model Predictive Control (NMPC) controllers for two autonomous-vehicle
scenarios, formulated with [CasADi](https://web.casadi.org/) and solved with IPOPT.

## Contents

| File | Description |
| --- | --- |
| `nmpc_takeover_student.py` | NMPC controller for a lane-change / takeover maneuver against a leader vehicle. 4-state kinematic bicycle model, 17 s horizon at 0.1 s. |
| `nmpc_dragracing_student.py` | NMPC controller for a drag-racing / lap scenario. 6-state dynamic bicycle model with tire slip, normal load transfer and aerodynamic drag. 30 s horizon at 0.1 s. |
| `case_1.mat` … `case_5.mat` | Test scenarios used to evaluate the controllers. |
| `metadata.yml` | Original submission metadata. |

## Models

**Takeover** — kinematic bicycle model with states `[x, y, psi, v]` relative to the
leader vehicle, controls `[a, delta]`. Parameters are provided per-solve: initial
condition, leader velocity, desired speed, and the previously applied steering angle
(for rate limiting).

**Drag racing** — dynamic bicycle model with states
`[v_x, v_y, r, e_psi, e_y, s]` and algebraic tire variables
`[F_yf, F_yr, F_muf, F_mur]`, controls `[F_x, delta]`. Includes slip-angle
computation, normal-load transfer and rolling/aero drag.

## Requirements

```bash
pip install casadi numpy matplotlib scipy
```

The drag-racing controller additionally imports `sim` and `utils` modules from the
original course simulation framework, which are not included here.

## Usage

```python
from nmpc_takeover_student import nmpc_controller

solver, lbx, ubx, lbg, ubg = nmpc_controller()
```

Each module builds and returns a CasADi NLP solver together with its variable and
constraint bounds; the solver is then called in a receding-horizon loop with the
current state as a parameter.
