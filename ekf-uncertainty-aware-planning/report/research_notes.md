# Research Notes

## Connection to the reference paper

The reference work studies motion-dependent state estimation for micro aerial vehicles and couples estimation uncertainty with path planning.

The paper uses an EKF-based visual-inertial estimator and an RRBT representation in which multiple belief states can correspond to the same physical state.

This project deliberately stops before that level of complexity.

## Why simplify?

A full implementation would require:

- visual-inertial estimation
- a much larger state
- MAV dynamics
- covariance propagation through candidate trajectories
- belief-tree search

The simplified project isolates one research question:

> How does a planning objective change when state-estimation uncertainty becomes part of the cost?

## Possible next research step

A strong next step is to represent each planner node as:

```text
Node = (position, state estimate, covariance, cost)
```

and then study how many candidate belief nodes can be removed without changing the selected solution.

That would move the project toward the computational issues associated with belief-space planning.
