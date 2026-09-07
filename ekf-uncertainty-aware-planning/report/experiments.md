# Experiments

## Experiment 1: A* baseline

Measure:

- path length
- planning result
- collision-free validity

## Experiment 2: EKF

Measure:

- position RMSE
- initial covariance
- final covariance
- covariance trace over time

## Experiment 3: Candidate route comparison

Compare a shortest route against a longer detour.

Measure:

- route length
- average uncertainty
- accumulated uncertainty
- combined objective

## Experiment 4: Lambda sweep

Sweep:

```text
0.00, 0.02, 0.05, 0.08, 0.12, 0.20, 0.50
```

Record which route is selected.

### Interpretation

Do not assume beforehand that the longer route will always win.

The experiment should answer whether the uncertainty term is large enough, under the chosen noise model, to justify additional travel distance.
