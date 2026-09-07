"""Uncertainty-aware route evaluation.

This module intentionally does not claim to implement RRBT.
It evaluates a small set of candidate routes using a combined
distance + uncertainty objective.
"""

from dataclasses import dataclass
import numpy as np

from .uncertainty import position_trace


@dataclass
class RouteScore:
    path_length: float
    average_uncertainty: float
    total_uncertainty: float
    objective: float


def score_route(path, uncertainty_values, lambda_uncertainty):
    """Compute J = L + lambda * accumulated uncertainty."""
    if len(path) < 2:
        length = 0.0
    else:
        length = float(len(path) - 1)

    uncertainty_values = np.asarray(uncertainty_values, dtype=float)
    average = float(np.mean(uncertainty_values))
    total = float(np.sum(uncertainty_values))
    objective = length + lambda_uncertainty * total

    return RouteScore(
        path_length=length,
        average_uncertainty=average,
        total_uncertainty=total,
        objective=objective,
    )


def select_best_route(routes, uncertainty_by_route, lambda_uncertainty):
    """Select the candidate route with minimum combined objective."""
    scores = [
        score_route(
            path,
            uncertainty_by_route[i],
            lambda_uncertainty,
        )
        for i, path in enumerate(routes)
    ]

    best_index = int(np.argmin([score.objective for score in scores]))
    return best_index, scores


def uncertainty_from_covariances(covariances):
    """Convert a sequence of EKF covariance matrices to trace values."""
    return np.array([position_trace(P) for P in covariances])
