# EKF-Based Uncertainty-Aware Path Planning

A beginner-friendly 2D robotics project that studies how state-estimation uncertainty can influence path planning.

Inspired by:

**Path Planning for Motion Dependent State Estimation on Micro Aerial Vehicles**  
Markus W. Achtelik, Stephan Weiss, Margarita Chli, Roland Siegwart.

The original paper couples path planning with motion-dependent state estimation. This repository implements a deliberately simplified version of that idea using:

- a 2D grid world
- A* as a shortest-path baseline
- a simple robot motion model
- an Extended Kalman Filter (EKF)
- covariance-based uncertainty
- an uncertainty-aware path-selection experiment
- visualizations and unit tests

## Research question

> Can a robot choose a slightly longer path that produces lower state-estimation uncertainty?

This is **not** a reproduction of the paper's full 24-state visual-inertial estimator or RRBT. The purpose is to isolate the core relationship:

```text
Robot motion
     ↓
Noisy measurements
     ↓
EKF state estimation
     ↓
Covariance / uncertainty
     ↓
Path evaluation
```

## Repository structure

```text
ekf-uncertainty-aware-planning/
├── README.md
├── requirements.txt
├── .gitignore
├── src/
│   ├── __init__.py
│   ├── environment.py
│   ├── robot.py
│   ├── ekf.py
│   ├── uncertainty.py
│   ├── planner.py
│   ├── uncertainty_planner.py
│   ├── simulation.py
│   └── visualization.py
├── experiments/
│   ├── run_baseline.py
│   ├── run_ekf.py
│   ├── compare_planners.py
│   └── parameter_sweep.py
├── results/
│   ├── figures/
│   └── data/
├── report/
│   ├── methodology.md
│   ├── experiments.md
│   └── research_notes.md
└── tests/
    ├── test_environment.py
    ├── test_planner.py
    └── test_ekf.py
```

## Installation

```bash
python3 -m venv .venv
source .venv/bin/activate        # macOS/Linux
# .venv\Scripts\activate         # Windows

pip install -r requirements.txt
```

## Run

From the repository root:

```bash
python -m experiments.run_baseline
python -m experiments.run_ekf
python -m experiments.compare_planners
python -m experiments.parameter_sweep
```

Run tests:

```bash
pytest
```

Figures are written to `results/figures/`, and experiment data is written to `results/data/`.

## What the EKF estimates

The state is intentionally small:

\[
x = [p_x,\ p_y,\ v_x,\ v_y]^T
\]

where `p` is position and `v` is velocity.

The motion model is constant velocity:

\[
p_{k+1}=p_k+v_k\Delta t
\]

\[
v_{k+1}=v_k
\]

The EKF maintains:

- the estimated state `x`
- the covariance `P`

The prediction step is:

\[
x^- = F x + G a
\]

\[
P^- = FPF^T + Q
\]

The measurement is position:

\[
z = Hx + noise
\]

and the update is:

\[
y=z-Hx^-
\]

\[
S=HP^-H^T+R
\]

\[
K=P^-H^TS^{-1}
\]

\[
x=x^-+Ky
\]

\[
P=(I-KH)P^-
\]

## Uncertainty

The first uncertainty metric is covariance trace:

\[
U = tr(P_{position})
\]

It summarizes the 2D position uncertainty with one scalar.

This is intentionally simple and interpretable. The original paper discusses several covariance comparison methods before using a distributional/KL-divergence comparison.

## Planner comparison

The baseline A* planner minimizes path length.

The experimental uncertainty-aware selector evaluates candidate paths using:

\[
J = L + \lambda U
\]

where:

- `L` = path length
- `U` = simulated accumulated uncertainty
- `lambda` = tradeoff parameter

The project currently uses a small set of deliberately different candidate routes rather than claiming to reproduce RRBT. This makes the experiment easy to inspect and extend.

## Experiments

### EKF experiment

`run_ekf.py` compares:

- ground-truth trajectory
- noisy position measurements
- EKF estimate
- position uncertainty

### Planner comparison

`compare_planners.py` compares a shortest route with an alternative route under the same environment.

### Parameter sweep

`parameter_sweep.py` varies `lambda` and records:

- selected route
- path length
- average uncertainty
- total objective

## Limitations

This is a teaching/research prototype, not a full MAV implementation. It does not include:

- the paper's full 24-state visual-inertial EKF
- camera feature tracking
- IMU/camera calibration states
- 3D quadrotor dynamics
- RRBT belief trees
- real sensor data
- dynamic obstacles

A natural next step is to replace the simplified estimator with richer visual-inertial measurements and then investigate belief-node pruning or efficient covariance propagation.

## Suggested research extensions

1. Compare trace, determinant, maximum eigenvalue, and KL-divergence-based uncertainty.
2. Add IMU acceleration measurements.
3. Move from 2D to 3D.
4. Add camera observations.
5. Implement belief nodes containing state, covariance, and cost.
6. Build a simplified RRBT.
7. Investigate computationally efficient belief-node pruning.
8. Test with recorded sensor data.

## Author

Aditi Rout  
MS Computer Science, Georgia Institute of Technology
