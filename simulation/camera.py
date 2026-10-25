"""Synthetic pinhole camera + actual threshold/centroid detection of its pixels.

One point supplies two pointing constraints, not a unique roll reference.
Roll is therefore held by the simulated attitude controller, not by this point.
"""
from __future__ import annotations
import numpy as np


def rotation_matrix(attitude_deg: np.ndarray) -> np.ndarray:
    r, p, y = np.deg2rad(attitude_deg)
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1., 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1., 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.]])
    return rz @ ry @ rx


class VirtualCamera:
    width, height = 320, 240
    focal = width / (2 * np.tan(np.deg2rad(100.0) / 2))
    cx, cy = (width - 1) / 2, (height - 1) / 2

    def render(self, attitude: np.ndarray, target: np.ndarray, visible: bool,
               rng: np.random.Generator, noise: float = 2.0) -> np.ndarray:
        image = np.clip(rng.normal(12., noise, (self.height, self.width)), 0, 255).astype(np.uint8)
        direction = rotation_matrix(target)[:, 0]
        camera_ray = rotation_matrix(attitude).T @ direction
        if visible and camera_ray[0] > 0.01:
            u = self.cx + self.focal * camera_ray[1] / camera_ray[0]
            v = self.cy - self.focal * camera_ray[2] / camera_ray[0]
            if 4 <= u < self.width - 4 and 4 <= v < self.height - 4:
                yy, xx = np.ogrid[:self.height, :self.width]
                image[(xx - u) ** 2 + (yy - v) ** 2 <= 5.0 ** 2] = 245
        return image

    def detect(self, image: np.ndarray) -> tuple[float, float] | None:
        # Sufficient for the demonstrator's one bright spot, not a real vision algorithm.
        y, x = np.nonzero(image > 190)
        if not 5 <= len(x) <= 600:
            return None
        return float(x.mean()), float(y.mean())

    def visual_error(self, uv: tuple[float, float]) -> tuple[float, float]:
        return (float(np.rad2deg(np.arctan2(uv[0] - self.cx, self.focal))),
                float(np.rad2deg(np.arctan2(uv[1] - self.cy, self.focal))))

    def pointing_reference(self, uv: tuple[float, float], attitude: np.ndarray,
                           roll_reference: float = 0.) -> np.ndarray:
        ray = np.array([1., (uv[0] - self.cx) / self.focal, -(uv[1] - self.cy) / self.focal])
        world = rotation_matrix(attitude) @ ray
        return np.array([roll_reference,
                         np.rad2deg(np.arctan2(-world[2], np.hypot(world[0], world[1]))),
                         np.rad2deg(np.arctan2(world[1], world[0]))])
