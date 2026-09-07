"""Uncertainty metrics and covariance ellipse utilities."""

import numpy as np


def position_trace(P):
    """Scalar uncertainty: trace of 2D position covariance."""
    P_pos = np.asarray(P)[:2, :2]
    return float(np.trace(P_pos))


def position_determinant(P):
    """Area-related uncertainty measure for the position covariance."""
    P_pos = np.asarray(P)[:2, :2]
    return float(np.linalg.det(P_pos))


def max_position_eigenvalue(P):
    """Largest principal variance in the position covariance."""
    P_pos = np.asarray(P)[:2, :2]
    eigenvalues = np.linalg.eigvalsh(P_pos)
    return float(np.max(eigenvalues))


def covariance_ellipse(P, confidence_scale=2.0):
    """Return ellipse radii and orientation from 2D covariance.

    The result is suitable for plotting an approximate uncertainty ellipse.
    """
    P_pos = np.asarray(P)[:2, :2]
    eigenvalues, eigenvectors = np.linalg.eigh(P_pos)

    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]

    eigenvalues = np.maximum(eigenvalues, 0.0)
    radii = confidence_scale * np.sqrt(eigenvalues)

    angle = np.degrees(np.arctan2(
        eigenvectors[1, 0],
        eigenvectors[0, 0],
    ))

    return radii, angle
