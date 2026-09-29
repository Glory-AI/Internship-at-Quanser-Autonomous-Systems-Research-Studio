Day 5: Rotary Inverted Pendulum (Furuta Pendulum) – Modeling, Dynamics, & Energy-Based Swing-Up Control
1. Overview

Day 5 transitions the Quanser QUBE-Servo lab focus from a single-axis DC motor to a multi-variable, nonlinear complex system: the Rotary Inverted Pendulum (Furuta Pendulum).

This session covers rigorous system modeling using the Euler-Lagrange formulation, linearization techniques around the upright equilibrium, state-space vector definitions, and the implementation of an energy-based swing-up control law.

2. System Nomenclature & Coordinate Setup
Rotary Arm ($\theta$): Driven directly by the servo motor at the pivot point. A positive voltage applied to the motor moves the rotary arm in a positive counter-clockwise direction.
Pendulum Angle ($\alpha$): Attached to the free end of the rotary arm. $\alpha$ represents the angular displacement measured relative to the upright vertical position.
$\alpha = 0$: Pendulum is perfectly upright.
$\alpha = \pm\pi$: Pendulum is hanging straight down at rest.
3. Mathematical Modeling: Nonlinear Equations of Motion
A. Simple Pendulum Dynamics Under Pivot Acceleration

Considering a pendulum subject to horizontal pivot acceleration $u(t)$:

$$
J_p \ddot{\alpha}(t)

m_p g l \sin\left(\alpha(t)\right)
m_p l u(t) \cos\left(\alpha(t)\right)
= 0
$$

Where:

$J_p$: Moment of inertia of the pendulum about the pivot axis
$m_p$: Mass of the pendulum
$l$: Length from the pivot to the pendulum's center of mass, $l = \frac{L_p}{2}$
$g$: Acceleration due to gravity
B. Full System Euler-Lagrange Formulation

For multi-degree-of-freedom robotic systems, total kinetic energy ($T$) and potential energy ($V$) are combined into the Lagrangian:

$$
L = T - V
$$

Applying the Euler-Lagrange equation for each generalized coordinate $q_i$:

\frac{\partial L}{\partial q_i}

Q_i
$$

This yields the complete nonlinear equations of motion (EOM):

Rotary Arm Dynamics:

$$
\left(J_r + J_p \sin^2\left(\alpha\right)\right)\ddot{\theta}

m_p l r \cos\left(\alpha\right)\ddot{\alpha}
2J_p \sin\left(\alpha\right)\cos\left(\alpha\right)\dot{\theta}\dot{\alpha}
m_p l r \sin\left(\alpha\right)\dot{\alpha}^2
=
\tau - b_r \dot{\theta}
$$
Pendulum Link Dynamics:

$$
J_p \ddot{\alpha}

m_p l r \cos\left(\alpha\right)\ddot{\theta}
J_p \sin\left(\alpha\right)\cos\left(\alpha\right)\dot{\theta}^2
m_p g l \sin\left(\alpha\right)
=
-b_p \dot{\alpha}
$$

Where viscous damping coefficients ($b_r$, $b_p$) and motor input torque

$$
\tau =
\frac{K_m\left(v_m - K_m\dot{\theta}\right)}{R_m}
$$

are fully integrated.

4. Linearization & State-Space Representation
Linearization: Near the upright operating point ($\alpha \approx 0$), small-angle approximations are applied:

$$
\sin(\alpha) \approx \alpha,
\qquad
\cos(\alpha) \approx 1,
\qquad
\dot{\alpha}^2 \approx 0
$$

State Vector Definition:

$$
x(t) =
\begin{bmatrix}
\theta(t) \
\alpha(t) \
\dot{\theta}(t) \
\dot{\alpha}(t)
\end{bmatrix}
$$

Output Vector Definition:

$$
y(t) =
\begin{bmatrix}
\theta(t) \
\alpha(t)
\end{bmatrix}
$$

Linear State-Space Matrices: Configured in standard form:

$$
\dot{x}(t) = Ax(t) + Bu(t),
\qquad
y(t) = Cx(t) + Du(t)
$$

5. Energy-Based Swing-Up Control Strategy

Because linear state-feedback controllers only stabilize the system locally near $\alpha = 0$, an energy-based controller is deployed to swing the pendulum up from its stable hanging position ($E_r = E_p$).

A. Total Mechanical Energy ($E$)

The total mechanical energy is given by:

$$
E = E_P + E_K
$$

where:

$$
E = m_p g l \left(1 - \cos(\alpha)\right)

\frac{1}{2}J_p\dot{\alpha}^2
$$
B. Energy Error & Control Law

By differentiating energy with respect to time ($\dot{E}$), a nonlinear control law forces the system energy toward the reference energy ($E_r$):

$$
u =
\operatorname{Sat}{u{\max}}
\left[
k_e(E - E_r)
\operatorname{sign}
\left(
\dot{\alpha}\cos(\alpha)
\right)
\right]
$$

C. Actuator Voltage Conversion

The computed linear acceleration control signal $u(t)$ is mapped to physical motor voltage $v_m(t)$ using plant parameters:

\frac{R_m r m_r}{K_t}u(t)
$$
