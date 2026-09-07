# Methodology

## 1. Environment

The robot moves in a 2D grid world containing static obstacles.

A* provides a collision-free shortest-path baseline.

## 2. Motion model

The state is:

\[
x=[p_x,p_y,v_x,v_y]^T.
\]

The robot follows constant-velocity dynamics with acceleration control.

## 3. State estimation

The EKF predicts the state from control input and corrects it using noisy position observations.

The covariance matrix is propagated alongside the state.

## 4. Uncertainty

Position uncertainty is summarized by:

\[
U=tr(P_{position}).
\]

This is a scalar proxy used to compare candidate trajectories.

## 5. Planning objective

Candidate routes are evaluated with:

\[
J=L+\lambda U_{total}.
\]

The project is intentionally simpler than a belief-space planner. Its purpose is to make the relationship between planning and estimation uncertainty observable and measurable.
