"""Matplotlib visualizations."""

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


def ensure_dir(path):
    Path(path).mkdir(parents=True, exist_ok=True)


def plot_world(world, path=None, start=None, goal=None, title="2D Environment",
               save_path=None):
    fig, ax = plt.subplots(figsize=(9, 6))

    obstacle_y, obstacle_x = np.where(world.grid)
    ax.scatter(obstacle_x, obstacle_y, marker="s", s=90, label="Obstacle")

    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, marker="o", label="Path")

    if start:
        ax.scatter([start[0]], [start[1]], s=120, label="Start")
    if goal:
        ax.scatter([goal[0]], [goal[1]], s=120, label="Goal")

    ax.set_xlim(-1, world.width)
    ax.set_ylim(-1, world.height)
    ax.set_aspect("equal")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(title)
    ax.grid(True, alpha=0.25)
    ax.legend()

    if save_path:
        ensure_dir(Path(save_path).parent)
        fig.savefig(save_path, dpi=160, bbox_inches="tight")
    return fig, ax


def plot_ekf(result, save_path=None):
    true_states = result["true_states"]
    measurements = result["measurements"]
    estimates = result["estimates"]

    fig, ax = plt.subplots(figsize=(9, 6))
    ax.plot(
        true_states[:, 0],
        true_states[:, 1],
        label="Ground truth",
        linewidth=2,
    )
    ax.scatter(
        measurements[:, 0],
        measurements[:, 1],
        s=18,
        alpha=0.45,
        label="Noisy measurements",
    )
    ax.plot(
        estimates[:, 0],
        estimates[:, 1],
        "--",
        label="EKF estimate",
        linewidth=2,
    )

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("EKF State Estimation")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend()

    if save_path:
        ensure_dir(Path(save_path).parent)
        fig.savefig(save_path, dpi=160, bbox_inches="tight")
    return fig, ax


def plot_uncertainty(result, save_path=None):
    uncertainties = result["uncertainties"]

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(np.arange(len(uncertainties)), uncertainties, linewidth=2)
    ax.set_xlabel("Time step")
    ax.set_ylabel("Position covariance trace")
    ax.set_title("EKF Position Uncertainty")
    ax.grid(True, alpha=0.25)

    if save_path:
        ensure_dir(Path(save_path).parent)
        fig.savefig(save_path, dpi=160, bbox_inches="tight")
    return fig, ax


def plot_uncertainty_ellipse(result, index=-1, save_path=None):
    """Plot EKF estimate with an ellipse representing position covariance."""
    estimate = result["estimates"][index]
    P = result["covariances"][index]
    P_pos = P[:2, :2]

    eigenvalues, eigenvectors = np.linalg.eigh(P_pos)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]

    angle = np.degrees(np.arctan2(
        eigenvectors[1, 0],
        eigenvectors[0, 0],
    ))

    widths = 2.0 * np.sqrt(np.maximum(eigenvalues, 0.0))

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter([estimate[0]], [estimate[1]], s=80, label="EKF estimate")

    ellipse = Ellipse(
        xy=estimate[:2],
        width=widths[0],
        height=widths[1],
        angle=angle,
        fill=False,
        linewidth=2,
        label="Uncertainty ellipse",
    )
    ax.add_patch(ellipse)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Position Uncertainty Ellipse")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend()

    if save_path:
        ensure_dir(Path(save_path).parent)
        fig.savefig(save_path, dpi=160, bbox_inches="tight")
    return fig, ax


def plot_planner_comparison(routes, scores, save_path=None):
    fig, ax = plt.subplots(figsize=(9, 6))

    for i, route in enumerate(routes):
        xs = [p[0] for p in route]
        ys = [p[1] for p in route]
        ax.plot(
            xs,
            ys,
            marker="o",
            label=(
                f"Route {i + 1}: "
                f"L={scores[i].path_length:.0f}, "
                f"U={scores[i].total_uncertainty:.2f}"
            ),
        )

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Candidate Path Comparison")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend()

    if save_path:
        ensure_dir(Path(save_path).parent)
        fig.savefig(save_path, dpi=160, bbox_inches="tight")
    return fig, ax
