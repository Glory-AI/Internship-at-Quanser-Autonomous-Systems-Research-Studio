# Model Predictive Control (MPC) & Disturbance-Aware Trajectory Tracking of the Quanser Qube-Servo 3

A comprehensive experimental study, mathematical overview, and control engineering implementation comparing **Model Predictive Control (MPC)** against a baseline **Linear Quadratic Regulator (LQR)** state-feedback controller on the Quanser Qube-Servo 3.

The project evaluates both a **QLabs Virtual Twin** and **physical Qube-Servo 3 hardware**, with particular emphasis on actuator constraints, disturbance handling, computational requirements, state estimation, and real-time control performance.

---

##  Project Overview

| Field                     | Details                                                                                |
| ------------------------- | -------------------------------------------------------------------------------------- |
| **Project Title**         | Model Predictive Control and Disturbance-Aware Trajectory Tracking of the Qube-Servo 3 |
| **Project Start Date**    | September 30, 2026                                                                     |
| **Author / Researcher**   | Glory                                                                                  |
| **Platform**              | Quanser Qube-Servo 3                                                                   |
| **Virtual Environment**   | QLabs Virtual Twin                                                                     |
| **Primary Controllers**   | MPC and LQR                                                                            |
| **Control Approach**      | State-space modeling and optimization-based control                                    |
| **MPC Solver**            | OSQP through CVXPY                                                                     |
| **Nominal Sampling Rate** | $500$ Hz ($T_s = 2$ ms)                                                                |

---

##  Project Motivation

When controlling complex, fast-moving physical systems such as an inverted pendulum, traditional linear state-feedback controllers such as LQR can perform effectively under ideal conditions.

However, real hardware introduces constraints and uncertainties that are not explicitly represented in a standard LQR formulation.

These include:

* Actuator voltage limits.
* Encoder wire drag.
* Joint friction.
* Sensor noise.
* External disturbances.
* Model mismatch.
* Computational and communication delays.

The project investigates whether **Model Predictive Control** can explicitly account for these realities while maintaining stable and accurate tracking.

### Core Engineering Question

> How can a model-based predictive controller use the mathematical model of the Qube-Servo 3 to predict future behavior, choose control actions subject to actuator constraints, and maintain desired motion in the presence of disturbances?

### Key Engineering Questions

1. **Model Usage:** How can a model-based predictive controller use the state-space dynamics of the Qube-Servo 3 to forecast future system states over a prediction horizon?
2. **Constraint Enforcement:** How does the optimization engine explicitly account for physical bounds, such as motor voltage saturation at $\pm 10$ V and rotary arm limits at $\pm 60^\circ$, before sending control signals?
3. **Disturbance Handling:** How does online receding-horizon optimization compensate for unexpected physical disturbances and unmodeled hardware behavior compared with static state feedback?
4. **Real-Time Execution:** Can the optimization problem be solved quickly enough to satisfy the strict timing requirements of the physical system?

---

##  Features

* State-space modeling of the Qube-Servo 3.
* Baseline LQR state-feedback control.
* Model Predictive Control using online quadratic programming.
* Explicit motor voltage constraints of $\pm 10$ V.
* Rotary arm workspace constraints of $\pm 60^\circ$.
* Prediction over a configurable receding horizon.
* Regulation and trajectory-tracking analysis.
* Disturbance-aware control evaluation.
* Comparison between QLabs simulation and physical hardware.
* Analysis of actuator saturation.
* Analysis of encoder quantization and velocity-estimation noise.
* Investigation of computational overhead and control-loop latency.
* Evaluation of practical requirements for real-time MPC implementation.

---

##  Control System Architecture

Control systems can be classified according to their representation, domain, and control strategy:

```text
CONTROL SYSTEM ARCHITECTURES
│
├── Classical Control
│   └── Transfer Functions / Frequency Domain
│       ├── SISO
│       │   └── PID
│       └── MIMO
│           └── Decoupled PID
│
└── Modern Control
    └── State-Space / Time Domain
        ├── State Feedback
        │   ├── LQR
        │   └── Pole Placement
        │
        └── Optimization-Based
            └── MPC
```

This project focuses on the transition from conventional state-feedback control to optimization-based predictive control.

---

##  LQR Baseline

### 1. Linear Quadratic Regulator

The Linear Quadratic Regulator provides a model-based state-feedback solution to the regulation problem.

LQR minimizes an infinite-horizon quadratic cost function:

$$
J =
\int_{0}^{\infty}
\left(
x^T Q x + u^T R u
\right)dt
$$

Where:

* $Q$ penalizes state errors, such as pendulum angle and rotary arm displacement.
* $R$ penalizes control effort, such as voltage usage or power consumption.

### LQR Execution

The optimal state-feedback gain matrix $K$ is calculated offline by solving the **Continuous Algebraic Riccati Equation (CARE)**.

Once $K$ has been calculated, the controller executes a simple matrix multiplication during runtime:

$$
u(t) = -Kx(t)
$$

For the four-state Qube-Servo 3 model:

$$
u(t)
=
-\left(
K_1\theta
+
K_2\alpha
+
K_3\dot{\theta}
+
K_4\dot{\alpha}
\right)
$$


The resulting computation is lightweight and can execute in microseconds on the target hardware.

### Example LQR Gains

The investigated controller included gains such as:

$$
K_1 = -5.2
$$

and

$$
K_3 = 45.8
$$

The larger magnitude of $K_3$ reflects the strong correction required for pendulum-angle deviations.

### LQR Constraint Limitation

A conventional LQR controller does not explicitly incorporate actuator saturation into its optimization problem.

For example, following a sufficiently large disturbance, the unconstrained control law could request:

$$
u = +15\text{ V}
$$

while the physical amplifier is limited to:

$$
-10\text{ V} \leq u \leq +10\text{ V}
$$

The amplifier therefore saturates the requested command.

The controller itself does not account for this clipping when computing its control action. This mismatch can introduce phase lag and contribute to loss of stability under sufficiently large disturbances.

---

## 🚀 Model Predictive Control

Model Predictive Control replaces a static feedback gain with an optimization problem that is solved repeatedly over a moving prediction horizon.

At each sampling instant, MPC:

1. Measures or estimates the current system state.
2. Uses the plant model to predict future states.
3. Optimizes a sequence of future control inputs.
4. Enforces physical constraints during optimization.
5. Applies the first control action.
6. Moves the prediction horizon forward.
7. Repeats the process at the next sampling instant.

This is known as **receding-horizon control**.

### MPC Feedback Structure

```text
                         ┌─────────────────────────────┐
                         │     Receding Horizon MPC    │
                         │           Feedback          │
                         └──────────────┬──────────────┘
                                        │
                                        ▼
[ Measurements ] → [ State Estimation ] → [ MPC Optimizer ]
                                              │
                                              ▼
                                      [ Plant Model ]
                                              │
                                              ▼
                                      [ Control Input ]
                                              │
                                              ▼
                                      [ Qube-Servo 3 ]
                                              │
                                              └──────────────→ Feedback
```

The manipulated variable is the motor voltage $u$.

The primary measured outputs are:

* Rotary arm position $\theta$.
* Pendulum angle $\alpha$.

---

## 📐 Mathematical Formulation

### State-Space Model

The Qube-Servo 3 is represented using the continuous-time state-space model:

$$
\dot{x}(t) = Ax(t) + Bu(t)
$$

$$
y(t) = Cx(t) + Du(t)
$$

Where:

* $x(t)$ is the system state.
* $u(t)$ is the control input.
* $y(t)$ is the measured output.
* $A$ is the state matrix.
* $B$ is the input matrix.
* $C$ is the output matrix.
* $D$ is the feedthrough matrix.

### General System Dynamics

A general nonlinear system can be represented as:

$$
\frac{dx}{dt} = f(x,u,t)
$$

where the state evolution depends on the current state, input, and time.

### Linear Time-Varying Model

A continuous linear time-varying system is represented as:

$$
\dot{x} = A(t)x + B(t)u
$$

$$
y = C(t)x + D(t)u
$$

### Linear Time-Invariant Model

For a time-invariant system:

$$
\dot{x} = Ax + Bu
$$

$$
y = Cx + Du
$$

### Stochastic Model with Disturbances

To account for disturbances and measurement noise, the model can be extended to:

$$
x' = Ax + Bu + Gw
$$

$$
y = Cx + Du + V
$$

Where:

* $w$ represents a process disturbance acting on the state transition.
* $V$ represents measurement noise affecting the measured output.
* $G$ maps the process disturbance into the system dynamics.

---

## 🎯 Regulation Task

Regulation is a fundamental benchmark problem in control theory.

The objective is to drive the system's state deviation toward zero and maintain it there.

For the Qube-Servo 3, this can include minimizing deviations in:

* Rotary arm position.
* Pendulum angle.
* Rotary arm velocity.
* Pendulum angular velocity.

Although real physical systems are nonlinear and subject to disturbances, deterministic linear models provide a useful starting point for controller design because they allow systematic analysis, optimization, and extension toward more complex control strategies.

---

## 📊 MPC Cost Function

MPC and LQR both use quadratic penalties to balance state performance against control effort.

A representative quadratic cost function is:

$$
J =
\int_{0}^{\infty}
\left(
x^T Qx + u^T Ru
\right)dt
$$

For MPC, the finite prediction horizon is used to optimize future control actions.

A discrete prediction-horizon formulation can be expressed as:

$$
\min_{u}
\sum_{t=0}^{N_p}
\left(
x_t^T Q_{\text{mpc}}x_t
+
u_t^T R_{\text{mpc}}u_t
\right)
$$

Where:

* $Q_{\text{mpc}}$ penalizes predicted state deviations.
* $R_{\text{mpc}}$ penalizes control effort.
* $N_p$ is the prediction horizon.
* $u_t$ represents the future control sequence.

---

## 🔒 Explicit MPC Constraints

One of the defining features of MPC is the ability to incorporate physical constraints directly into the optimization problem.

For the Qube-Servo 3, the motor voltage is constrained by:

$$
-10\text{ V} \leq u_t \leq +10\text{ V}
$$

The rotary arm is constrained by:

$$
-60^\circ \leq \theta_t \leq +60^\circ
$$

These constraints are considered during optimization rather than being applied only after the controller has generated an unconstrained command.

The manipulated-variable constraint can also be represented as:

$$
c =
\begin{bmatrix}
u \\
-u
\end{bmatrix}
$$

with:

$$
-10\text{ V} \leq u \leq +10\text{ V}
$$

---

## 🎛️ Qube-Servo 3 MPC Design Parameters

| Parameter                      | Description                                                                              |
| ------------------------------ | ---------------------------------------------------------------------------------------- |
| **Manipulated Variable ($u$)** | Motor voltage driving the rotary arm                                                     |
| **Voltage Constraint**         | $u \in [-10\text{ V}, +10\text{ V}]$                                                     |
| **Plant Model**                | Mathematical representation of arm dynamics, angular acceleration, gravity, and friction |
| **Measured Outputs**           | Rotary arm position $\theta$ and pendulum angle $\alpha$                                 |
| **Sample Time ($T_s$)**        | $2$ ms at $500$ Hz                                                                       |
| **Prediction Horizon ($N_p$)** | Number of future steps over which MPC predicts system behavior                           |
| **Control Horizon ($N_c$)**    | Number of future control moves optimized at each step                                    |
| **Arm Operating Window**       | $\pm 60^\circ$                                                                           |
| **Pendulum Operating Window**  | $\pm 10^\circ$ or approximately $\pm 0.175$ rad                                          |

The investigated control architecture uses a nominal sampling rate of:

$$
f_s = 500\text{ Hz}
$$

with:

$$
T_s = 0.002\text{ s} = 2\text{ ms}
$$

The Qube-Servo 3 inverted pendulum requires sufficiently fast sampling for balancing, making computational execution time a critical consideration for MPC.

---

## 🔄 Receding-Horizon Optimization

At every sampling instant, the MPC controller solves a constrained optimization problem over the prediction horizon.

The general process is:

```text
Current State
     │
     ▼
State Estimation
     │
     ▼
Future State Prediction
     │
     ▼
Quadratic Program (QP)
     │
     ├── State Cost
     ├── Control Cost
     ├── Voltage Constraints
     └── Position Constraints
     │
     ▼
Optimal Control Sequence
     │
     ▼
Apply First Control Input
     │
     ▼
Measure / Estimate New State
     │
     └───────────────→ Repeat
```

Only the first control action of the optimized sequence is applied before the entire optimization process is repeated using the newly measured or estimated state.

---

## 🔬 Experimental Evaluation

The MPC controller was evaluated in two environments:

1. **QLabs Virtual Twin**
2. **Physical Quanser Qube-Servo 3**

This comparison was used to investigate the gap between ideal simulation behavior and real-world control performance.

### Virtual Twin vs. Physical Hardware

| QLabs Virtual Twin            | Physical Hardware                   |
| ----------------------------- | ----------------------------------- |
| Ideal mathematical model      | Model mismatch and static friction  |
| Low-noise simulated outputs   | High-frequency encoder quantization |
| Ideal $500$ Hz execution loop | CPU spikes and USB latency          |
| Smooth upright balance        | Limit cycles and wobbling chatter   |

---

## 🧪 Experimental Findings

### 1. Actuator Constraint Handling

MPC successfully enforced the motor voltage constraint:

$$
-10\text{ V} \leq u \leq +10\text{ V}
$$

under large disturbances.

This differs from an unconstrained LQR formulation, where the calculated control action can exceed the physical actuator range before the amplifier clips the command.

MPC also respected the rotary arm workspace constraint:

$$
-60^\circ \leq \theta \leq +60^\circ
$$

### 2. Computational Overhead and Phase Lag

The MPC implementation used **CVXPY with the OSQP solver** to solve the quadratic program online.

At the nominal sampling period:

$$
T_s = 2\text{ ms}
$$

the controller must complete state processing, optimization, communication, and actuation within the available real-time interval.

Running the optimization loop in interpreted Python pushed execution times close to or beyond this boundary.

CPU processing spikes introduced additional control-loop delay and phase lag, which degraded balance stability on physical hardware.

### 3. Velocity Estimation and Encoder Noise

The physical Qube-Servo 3 produced high-frequency encoder quantization and measurement noise.

Numerical differentiation and derivative filtering using `ddt_filter` amplified some of these high-frequency effects.

The resulting velocity jitter affected the MPC optimization and caused the solver to produce control chatter.

This demonstrated that a practical physical MPC implementation requires improved state estimation rather than relying solely on numerical differentiation.

A **Kalman Filter** is therefore identified as an appropriate direction for obtaining cleaner velocity estimates from noisy encoder measurements.

### 4. Model Mismatch and Physical Disturbances

The physical system exhibited effects that were not fully represented by the idealized model, including:

* Encoder cable drag.
* Static friction.
* Joint friction.
* Sensor noise.
* External disturbances.
* Communication latency.
* Computational delay.

These effects explain part of the performance difference observed between the QLabs Virtual Twin and the physical Qube-Servo 3.

---

## 🛠️ Software Stack & Prerequisites

| Component                | Technology                     |
| ------------------------ | ------------------------------ |
| **Language**             | Python 3.10+                   |
| **Numerical Computing**  | NumPy                          |
| **Scientific Computing** | SciPy                          |
| **Optimization**         | CVXPY                          |
| **QP Solver**            | OSQP                           |
| **Quanser Interface**    | PAL (Python Abstraction Layer) |
| **Visualization**        | PyQtGraph                      |
| **GUI Framework**        | PyQt5                          |
| **Virtual Environment**  | QLabs                          |
| **Physical Interface**   | Qube-Servo 3 USB DAQ           |

---

## ⚙️ Installation

### Prerequisites

Install:

* Python 3.10 or later.
* Quanser QLabs for Virtual Twin operation.
* A compatible Qube-Servo 3 and USB DAQ board for physical operation.

Install the Python dependencies:

```bash
pip install numpy scipy cvxpy osqp pyqtgraph PyQt5
```

The Quanser PAL interface should be installed according to the Quanser software environment used with the Qube-Servo 3.

---

## 🚀 Usage

The controller supports both Virtual Twin and physical hardware operation.

### Virtual Twin Mode

Run the controller against the QLabs simulation environment:

```bash
python main_mpc_qube.py --mode virtual
```

The virtual mode uses:

```text
hardware = 0
```

### Physical Hardware Mode

Ensure that the Qube-Servo 3 USB DAQ board is connected and the system is calibrated before launching the controller:

```bash
python main_mpc_qube.py --mode hardware
```

The physical mode uses:

```text
hardware = 1
```

### Physical Hardware Startup

Before launching the balance controller:

1. Position the rotary arm at approximately $0^\circ$.
2. Hold the pendulum vertically.
3. Ensure the pendulum is within approximately $\pm 10^\circ$ of the upright position.
4. Confirm that the USB DAQ connection is active.
5. Launch the controller in hardware mode.

---

## 📈 LQR vs. MPC

| Characteristic            | LQR                                             | MPC                                                     |
| ------------------------- | ----------------------------------------------- | ------------------------------------------------------- |
| **Control Law**           | $u=-Kx$                                         | Online optimization                                     |
| **Gain Calculation**      | Offline                                         | Recomputed online                                       |
| **Actuator Constraints**  | Not explicitly included in standard formulation | Explicitly enforced                                     |
| **Workspace Constraints** | Not explicitly included                         | Explicitly enforced                                     |
| **Computational Cost**    | Very low                                        | High                                                    |
| **Execution Time**        | Microsecond-scale matrix operation              | Depends on QP solution time                             |
| **Disturbance Response**  | Depends on fixed feedback gains                 | Re-optimizes using current state                        |
| **Model Usage**           | State-feedback model                            | Predictive model over a horizon                         |
| **Hardware Sensitivity**  | Affected by saturation and model mismatch       | Also affected by model mismatch, noise, and computation |
| **Real-Time Requirement** | Relatively lightweight                          | Strict timing requirement                               |
| **Primary Limitation**    | No explicit constraint awareness                | Computational overhead                                  |

---

## 🔍 Engineering Interpretation

The experimental comparison highlights a fundamental difference between LQR and MPC.

LQR calculates a fixed state-feedback law:

$$
u = -Kx
$$

Once the gain matrix has been computed, the runtime controller only needs to evaluate the state-feedback equation.

MPC instead repeatedly solves an optimization problem using the current system state and a model of future system behavior.

This gives MPC the ability to explicitly incorporate constraints such as:

$$
-10\text{ V} \leq u \leq +10\text{ V}
$$

and:

$$
-60^\circ \leq \theta \leq +60^\circ
$$

However, this additional capability comes at the cost of substantially higher computational requirements.

The physical experiments therefore showed that successful MPC deployment is not determined solely by the quality of the mathematical controller. **State estimation, computation time, communication latency, actuator limits, and model accuracy are all part of the practical control system.**

---

## ⚠️ Practical Challenges Identified

### Actuator Saturation

The physical amplifier limits the available motor voltage to approximately $\pm 10$ V.

An unconstrained controller can request a larger value, but the hardware cannot deliver it.

MPC incorporates this limitation directly into the optimization problem.

### Encoder Quantization

Optical encoder measurements introduce quantization effects that become particularly problematic when numerical derivatives are used to estimate velocity.

This can produce noisy velocity states and undesirable control chatter.

### State Estimation

The experiments indicate that physical MPC requires a more robust state-estimation strategy.

A Kalman Filter is identified as a suitable direction for obtaining cleaner velocity estimates from noisy encoder measurements.

### Real-Time Computation

At $500$ Hz, the available control-loop period is:

$$
T_s = 2\text{ ms}
$$

Therefore, the complete control cycle must execute within approximately $2$ ms to maintain the nominal sampling rate.

Optimization overhead from the Python/CVXPY implementation can approach or exceed this limit.

---

## 📌 Summary of Findings

* **LQR** provides extremely fast runtime execution through the static feedback law $u=-Kx$.
* Standard LQR does not explicitly enforce actuator voltage or workspace constraints.
* **MPC** explicitly incorporates physical constraints into its optimization problem.
* MPC successfully maintained the investigated voltage and workspace constraints under disturbances.
* Online QP optimization introduces significant computational overhead.
* Python-based CVXPY execution can approach or exceed the $2$ ms real-time boundary required for $500$ Hz operation.
* Physical encoder quantization creates velocity-estimation noise when numerical differentiation is used.
* Control chatter observed on hardware demonstrates the importance of robust state estimation.
* The QLabs Virtual Twin provides smoother and more ideal behavior than the physical system.
* Physical implementation requires consideration of model mismatch, friction, cable drag, sensor noise, communication latency, and computation time.
* A **Kalman Filter** is identified as a practical next step for state estimation.
* Faster compiled QP solvers, including **C/C++ implementations**, are identified as a potential solution for reducing Python-loop latency.

---

## 🧭 Future Development

The experimental results motivate several directions for further development:

* Implement Kalman filtering for state estimation.
* Replace raw numerical differentiation with estimated velocity states.
* Investigate faster compiled QP solver implementations.
* Optimize the MPC prediction and control horizons.
* Profile the complete control loop to identify computational bottlenecks.
* Further evaluate disturbance rejection on physical hardware.
* Investigate the effect of model mismatch on prediction accuracy.
* Compare trajectory-tracking performance under controlled external disturbances.

---

## 📚 Project Scope

This project combines:

* Classical control concepts.
* State-space modeling.
* LQR state feedback.
* Model Predictive Control.
* Quadratic optimization.
* Physical actuator constraints.
* Disturbance-aware control.
* State estimation.
* Real-time embedded control.
* Simulation-to-hardware validation.

The central engineering objective is not simply to implement MPC, but to understand the relationship between the **mathematical optimization problem**, the **physical plant**, and the **real-time computing system** required to execute the controller.

---

## 🤝 Contributing

Contributions, experiments, improvements, and technical discussions are welcome.

When contributing:

1. Clearly describe the control strategy or modification.
2. Document changes to the mathematical model.
3. Include relevant simulation or hardware results where possible.
4. Report changes to computational performance.
5. Clearly distinguish simulation results from physical-hardware results.

---

## 📄 License

No license has been specified for this project yet.

If this repository is intended for public open-source use, add an appropriate license file such as `MIT`, `Apache-2.0`, or another license that matches the intended distribution and usage terms.
