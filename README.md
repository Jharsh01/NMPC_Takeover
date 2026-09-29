# NMPC Takeover & Drag Racing

Nonlinear Model Predictive Control (NMPC) formulations for two autonomous-driving
scenarios, built with [CasADi](https://web.casadi.org/) and intended to be solved
with IPOPT in a receding-horizon loop.

| Scenario | Model | Horizon |
| --- | --- | --- |
| **Takeover** — overtake a leader vehicle and return to lane | 4-state kinematic bicycle | 17 s @ 0.1 s (N = 170) |
| **Drag racing** — high-speed maneuver at the limits of tire friction | 6-state dynamic bicycle with tire forces | 30 s @ 0.1 s (N = 300) |

## Repository layout

| File | Description |
| --- | --- |
| `nmpc_takeover_student.py` | NLP formulation for the takeover maneuver. |
| `nmpc_dragracing_student.py` | NLP formulation for the drag-racing scenario. |
| `case_1.mat` … `case_5.mat` | Warm-start data for the drag-racing NLP (see [Test cases](#test-cases)). |
| `metadata.yml` | Original course submission metadata. |

## Takeover controller

`nmpc_takeover_student.py`

**State** (relative to the leader car): `x = [Δx, Δy, ψ, v]`
**Control:** `u = [a, δ]` (acceleration, steering angle)
**Parameters:** `p = [x_init (4), v_leader (2), v_des, δ_last]`

Dynamics — kinematic bicycle (`L_f = L_r = 1 m`), expressed in the leader's frame and
discretized with forward Euler:

```
β  = atan( L_r / (L_f + L_r) · tan δ )
Δẋ = v cos(ψ + β) − v_leader,x
Δẏ = v sin(ψ + β) − v_leader,y
ψ̇  = v / L_r · sin β
v̇  = a
```

**Cost** — track the desired longitudinal speed, penalize lateral/yaw motion and
lateral offset, and regularize control effort.

**Constraints**

| Constraint | Expression |
| --- | --- |
| Collision avoidance | ellipse around the leader: `Δx²/900 + Δy²/4 ≥ 1` (30 m × 2 m semi-axes) |
| Lateral acceleration | `|ψ̇ · v| ≤ 0.5 · 0.6 · g` |
| Lane keeping | `−1 ≤ Δy ≤ 3` |
| Steering rate | `|Δδ / h| ≤ 0.6 rad/s` (first step uses `δ_last`) |
| Input bounds | `−10 ≤ a ≤ 4 m/s²`, `|δ| ≤ 0.6 rad` |

## Drag-racing controller

`nmpc_dragracing_student.py`

**State:** `x = [U_x, U_y, r, X, Y, ψ]` (body-frame velocities, yaw rate, global pose)
**Control:** `u = [F_x, δ]` (total longitudinal force, steering angle)
**Auxiliary variables:** `z = [F_yf, F_yr, s_f, s_r]` — lateral tire forces and
friction-cone slack variables
**Parameters:** `p` = initial state (6)

The model includes slip-angle computation, longitudinal load transfer, front/rear
force distribution, a tire model with friction saturation, and rolling + aerodynamic
drag. Lateral tire forces are enforced as algebraic constraints on `z`.

**Constraints**

- Minimum speed `U_x ≥ 2 m/s`
- Engine power limit `F_x ≤ P_eng / U_x`
- Stay outside a 10 m-radius circle about the origin: `X² + Y² ≥ 100`
- Friction cone per axle: `F_x² + F_y² ≤ (μ F_z)² + s²`
- Steering bound `|δ| ≤ δ_max`

**Cost** — penalizes `Y` and `ψ` along the horizon and at the terminal state. The
tire-saturation and slack-variable penalty terms are left as `TODO` stubs.

## Requirements

- Python 3.8+
- `casadi`, `numpy`, `scipy`, `matplotlib`

```bash
pip install casadi numpy scipy matplotlib
```

The drag-racing module imports `sim` and `utils` (vehicle parameters `param`, tire
model, slip-angle and load-transfer helpers) from the original course simulation
framework. Those modules are **not** included in this repository, so
`nmpc_dragracing_student.py` will not import on its own.

## Usage

Both modules expose `nmpc_controller()`, which returns the NLP definition and its
bounds rather than a solver instance:

```python
import casadi as ca
import numpy as np
from nmpc_takeover_student import nmpc_controller

prob, N, n_vars, n_cons, n_par, lb_var, ub_var, lb_cons, ub_cons = nmpc_controller()

solver = ca.nlpsol("solver", "ipopt", prob, {"ipopt.print_level": 0, "print_time": 0})

# p = [x_init (Δx, Δy, ψ, v), v_leader (vx, vy), v_des, δ_last]
p = np.array([-40.0, 0.0, 0.0, 20.0, 15.0, 0.0, 25.0, 0.0])

sol = solver(x0=np.zeros(n_vars), p=p,
             lbx=lb_var, ubx=ub_var, lbg=lb_cons, ubg=ub_cons)

w = sol["x"].full().ravel()
u = w[: 2 * N].reshape(N, 2)   # decision vector is ordered [u, x(, z)]
a0, delta0 = u[0]              # apply first input, then re-solve next step
```

The decision vector is laid out as `[u (Dim_ctrl·N), x (Dim_state·(N+1))]` for the
takeover problem, with an additional `z (4·N)` block appended for drag racing.

## Test cases

`case_1.mat` … `case_5.mat` each contain a primal/dual warm start for the
drag-racing NLP:

| Key | Shape | Meaning |
| --- | --- | --- |
| `x0_nlp` | 366 × 1 | Initial guess for the decision vector |
| `lamx0_nlp` | 366 × 1 | Multipliers for variable bounds |
| `lamg0_nlp` | 396 × 1 | Multipliers for constraints |

These sizes correspond to a 30-step horizon (`12N + 6` variables, `13N + 6`
constraints), so the horizon in `nmpc_dragracing_student.py` must be set to
`N = 30` (e.g. `T = 3`) to use them.

```python
from scipy.io import loadmat
case = loadmat("case_1.mat")
sol = solver(x0=case["x0_nlp"], lam_x0=case["lamx0_nlp"], lam_g0=case["lamg0_nlp"],
             p=p, lbx=lb_var, ubx=ub_var, lbg=lb_cons, ubg=ub_cons)
```

## Known limitations

- The drag-racing friction-cone constraints and saturation cost compute slip angles
  from the symbolic model state `xm` instead of the horizon state `x[:, k]`.
- The drag-racing terminal cost uses `x[5, k]` (last loop index) rather than `x[5, N]`.
- Tire-saturation and slack penalty terms in the drag-racing cost are unimplemented.
