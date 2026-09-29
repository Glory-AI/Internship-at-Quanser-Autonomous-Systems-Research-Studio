Day 4: State-Space Modeling & Stability Analysis
1. Objective

Transition from classical transfer function methods to modern state-space representation, derive the system matrices from physical parameters, and perform open-loop stability analysis via eigenvalues/poles.

2. State-Space Derivation

Starting from the motor electrical, shaft, and torque equations:

$$
\tau \ddot{\theta}_m(t) + \dot{\theta}_m(t) = K v_m(t)
$$

Using state variables $x_1 = \theta_m(t)$ (position) and $x_2 = \dot{\theta}_m(t)$ (velocity), with input $u = v_m$:

$A$ Matrix:

\begin{bmatrix}
0 & 1 \
0 & -1.3812
\end{bmatrix}
$$

using experimental parameters $K = 0.1451$ and $\tau = 0.7240$.

$B$ Matrix:

\begin{bmatrix}
0 \
0.2004
\end{bmatrix}
$$

$C$ Matrix:

$$
C =
\begin{bmatrix}
1 & 0 \
0 & 1
\end{bmatrix}
$$

$D$ Matrix:

$$
D =
\begin{bmatrix}
0 \
0
\end{bmatrix}
$$

3. Stability Analysis & Pole Location

To evaluate Bounded-Input Bounded-Output (BIBO) stability, we analyze the eigenvalues (poles) of the system's $A$ matrix:

Calculated Poles (Eigenvalues):
$s_1 = 0$ (due to pure integration of motor position to velocity)
$s_2 = -\frac{1}{\tau} = -1.38$
Stability Conclusion:
The pole at $s_2 = -1.38$ lies strictly in the Left-Half Plane (LHP), meaning unforced natural dynamics decay to zero.
The pole at $s_1 = 0$ sits on the imaginary axis (representing the unpowered motor's neutral position holding behavior).
Because there are no poles in the Right-Half Plane (RHP), the open-loop system is marginally stable (standard for a DC motor position plant), laying the foundation for state-feedback stabilization.
