Day 3: Controller Performance & Practical Fixes
1. Objective

Evaluate the performance and limitations of basic proportional (P), proportional-derivative (PD), and PID controllers on the Quanser QUBE-Servo 3, and implement practical fixes for real-world issues like derivative noise and integral windup.

2. Experimental Observations
Proportional Control (P):
Settles with a steady-state error because back-EMF balances the proportional voltage output as the motor slows down near the target, causing the motor to stall before reaching the commanded angle.
Exhibits sharp transient spikes ($3-5$ units) and negative overshoots ($\sim -1$ units) before flattening out.
Practical PID Fixes Implemented:
Derivative Low-Pass Filter: Added a filter $\frac{1}{\tau_f s + 1}$ to the derivative path to eliminate high-frequency encoder jitter.
Anti-Windup: Applied saturation limits on the integrator to prevent integrator windup during large voltage steps.
