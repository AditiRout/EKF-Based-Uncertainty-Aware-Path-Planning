# EKF-Based Uncertainty-Aware Path Planning

A 2D simulation study of how state-estimation uncertainty can influence robot path planning.

This project is inspired by the paper:

> **Path Planning for Motion Dependent State Estimation on Micro Aerial Vehicles**
> Markus W. Achtelik, Stephan Weiss, Margarita Chli, Roland Siegwart

The original work combines path planning with motion-dependent state estimation using an Extended Kalman Filter (EKF) and Rapidly-exploring Random Belief Trees (RRBT).

This repository implements a simplified 2D version of the underlying idea:

> **A path that is slightly longer may be preferable if it significantly improves state-estimation quality.**

---

## Research Motivation

Traditional path planners generally optimize geometric objectives such as distance or travel time.

However, for robots that rely on noisy sensors, the robot's motion can affect how well its state can be estimated.

A shortest path is therefore not necessarily the best path.

This project investigates the following question:

> **Can a robot choose a slightly longer path that produces lower state-estimation uncertainty?**

The project combines:

* 2D path planning
* noisy robot motion
* Extended Kalman Filtering
* covariance-based uncertainty estimation
* uncertainty-aware path costs
* controlled simulation experiments

---

## Project Architecture

```text
                 2D Environment
                       │
                       ▼
                ┌────────────┐
                │   Planner  │
                └─────┬──────┘
                      │
             Candidate Path
                      │
                      ▼
                ┌────────────┐
                │   Robot    │
                └─────┬──────┘
                      │
                Noisy Sensors
                      │
                      ▼
                ┌────────────┐
                │    EKF     │
                └─────┬──────┘
                      │
             State + Covariance
                      │
                      ▼
              Uncertainty Cost
                      │
                      ▼
           Path Selection / Comparison
```

---

## State Estimation

The simulated robot uses a simplified state representation:

$$
x = [p_x, p_y, v_x, v_y]^T
$$

where:

* \(p_x,p_y\) are position
* \(v_x,v_y\) are velocity

The EKF maintains both the state estimate and covariance matrix:

$$
P \in \mathbb{R}^{4\times4}
$$

The covariance represents uncertainty in the estimated state.

---

## Extended Kalman Filter

The estimator follows the standard EKF cycle.

### Prediction

$$
x_{k|k-1}=f(x_{k-1},u_k)
$$

$$
P_{k|k-1}=F_kP_{k-1}F_k^T+Q
$$

### Measurement Update

$$
y_k=z_k-h(x_k)
$$

$$
S_k=H_kP_kH_k^T+R
$$

$$
K_k=P_kH_k^TS_k^{-1}
$$

$$
x_k=x_k+K_ky_k
$$

$$
P_k=(I-K_kH_k)P_k
$$

The implementation can therefore be used to study how noisy measurements affect state estimation.

---

## Uncertainty Metric

The first implementation uses covariance trace as a simple scalar uncertainty metric:

$$
U = tr(P)
$$

A lower value indicates lower overall covariance magnitude.

The project can later compare this against:

* determinant
* maximum eigenvalue
* KL divergence

The original paper discusses several covariance comparison approaches and ultimately uses KL divergence for comparing belief distributions.

---

## Path Planning

### Baseline

The baseline planner minimizes path length:

$$
J_{baseline}=L
$$

where \(L\) is the path length.

A* is used as the baseline search algorithm.

### Uncertainty-Aware Planner

The uncertainty-aware planner considers both distance and estimation uncertainty:

$$
J=L+\lambda U
$$

where:

* \(L\) = path length
* \(U\) = accumulated state uncertainty
* \(\lambda\) = uncertainty weighting parameter

When:

$$
\lambda=0
$$

the planner behaves like the distance-only baseline.

Increasing \(\lambda\) places more importance on estimation quality.

---

## Experiments

The project evaluates the following questions.

### 1. Does the EKF improve state estimation?

Compare:

* true state
* noisy measurements
* EKF estimate

Metrics:

* position RMSE
* velocity RMSE
* covariance

### 2. How does sensor noise affect estimation?

Evaluate multiple noise levels and measure:

* estimation error
* final covariance
* average uncertainty

### 3. Does uncertainty-aware planning change the selected path?

Compare:

* shortest path
* uncertainty-aware path

Metrics:

* path length
* average uncertainty
* final uncertainty
* position RMSE

### 4. What is the effect of λ?

Evaluate multiple values of λ and visualize the tradeoff between:

* path efficiency
* state-estimation uncertainty

---

## Results

Results are stored in:

```text
results/
├── figures/
└── data/
```

Example visualizations include:

* planned paths
* EKF state estimates
* covariance/uncertainty ellipses
* uncertainty over time
* path-length vs uncertainty tradeoffs
* parameter-sweep results

---

## Repository Structure

```text
src/
├── environment.py
├── robot.py
├── ekf.py
├── uncertainty.py
├── planner.py
├── uncertainty_planner.py
├── simulation.py
└── visualization.py

experiments/
├── run_baseline.py
├── run_uncertainty_aware.py
├── compare_planners.py
└── parameter_sweep.py

tests/
├── test_ekf.py
├── test_planner.py
└── test_environment.py
```

---

## Installation

Clone the repository:

```bash
git clone <YOUR_REPOSITORY_URL>
cd ekf-uncertainty-aware-planning
```

Create a virtual environment:

```bash
python -m venv .venv
```

Activate it.

macOS/Linux:

```bash
source .venv/bin/activate
```

Windows:

```bash
.venv\Scripts\activate
```

Install dependencies:

```bash
pip install -r requirements.txt
```

---

## Running the Experiments

Run the baseline:

```bash
python experiments/run_baseline.py
```

Run the uncertainty-aware planner:

```bash
python experiments/run_uncertainty_aware.py
```

Compare both planners:

```bash
python experiments/compare_planners.py
```

Run the parameter sweep:

```bash
python experiments/parameter_sweep.py
```

Run tests:

```bash
pytest
```

---

## Limitations

This project is intentionally a simplified research prototype.

It does not attempt to reproduce the complete system presented in the original paper.

In particular, this implementation does not currently include:

* the full 24-state visual-inertial EKF
* camera feature tracking
* IMU/camera calibration states
* quadrotor dynamics
* 3D MAV motion
* Rapidly-exploring Random Belief Trees
* full belief-tree propagation
* real-world sensor data
* dynamic obstacles

The objective is instead to isolate and study the relationship between:

$$
\text{Motion}
\rightarrow
\text{State Estimation}
\rightarrow
\text{Uncertainty}
\rightarrow
\text{Path Planning}
$$

in a controlled environment.

---

## Future Work

Potential extensions include:

1. Replace the simplified EKF with a visual-inertial state estimator.

2. Extend the state representation to 3D.

3. Incorporate IMU measurements.

4. Introduce camera-based observations.

5. Implement belief nodes containing state, covariance, and cost.

6. Implement a simplified RRBT planner.

7. Investigate efficient belief-node pruning.

8. Compare different covariance metrics.

9. Introduce dynamic obstacles.

10. Evaluate the planner using real or recorded sensor data.

---

## Research Connection

The long-term goal is to investigate computationally efficient uncertainty-aware planning.

A particularly interesting direction is reducing the computational cost associated with covariance propagation and belief-node expansion.

This could potentially enable uncertainty-aware planning to operate under more realistic real-time constraints.

---

## Key Takeaway

This project demonstrates a simplified version of a broader robotics problem:

> **Planning should account not only for where a robot can go, but also for how its motion affects what the robot can know about its own state.**

---

## Citation

If you use or extend the concepts from the original paper, please cite:

Achtelik, M. W., Weiss, S., Chli, M., & Siegwart, R.
*Path Planning for Motion Dependent State Estimation on Micro Aerial Vehicles.*

---

## Author

**Aditi Rout**

MS Computer Science
Georgia Institute of Technology

Interests: Robotics, State Estimation, Motion Planning, Computational Perception
