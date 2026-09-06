# Uncertainty-Aware-Path-Planning-in-2D

A small simulation project inspired by the paper **"Path Planning for Motion Dependent State Estimation on Micro Aerial Vehicles."**

## Motivation

Traditional path planning often focuses on finding the shortest collision-free path from a start position to a goal.

However, for robots that rely on noisy sensors for localization, the robot's motion can also influence how accurately its state can be estimated.

This project explores a simplified version of this idea:

> Can a robot accept a small increase in path length to significantly reduce state uncertainty?

## Approach

I compare two planners:

1. **Shortest-path planner** — minimizes path length.
2. **Uncertainty-aware planner** — minimizes a combined objective:

$$
J = L + \lambda U
$$

where:

* \(L\) is path length
* \(U\) is accumulated state uncertainty
* \(\lambda\) controls the trade-off between efficiency and estimation quality.

The robot operates in a 2D grid environment containing obstacles.

## Experiments

The project evaluates:

* Path length
* Final state uncertainty
* Computational cost
* Effect of the uncertainty trade-off parameter
* Effect of sensor noise

## Research Question

How does the trade-off between path efficiency and state uncertainty change as sensor noise increases?

## Limitations

This is a simplified simulation and does not implement the full 24-state EKF, quadrotor dynamics, or RRBT framework from the original paper.

The uncertainty model is intentionally simplified to study the underlying planning trade-off.

## Future Work

Possible extensions include:

* EKF-based uncertainty propagation
* RRT/RRBT-style sampling
* More realistic sensor models
* Dynamic obstacles
* Efficient belief-node pruning
* Extension from 2D to 3D drone motion

## Reference

Achtelik, M. W., Weiss, S., Chli, M., & Siegwart, R.
"Path Planning for Motion Dependent State Estimation on Micro Aerial Vehicles."

## Author

Aditi Rout
MS Computer Science
Georgia Institute of Technology
