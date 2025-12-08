#!/usr/bin/env python3

import time
import numpy as np
import pandas as pd
from math import tan, radians
from scipy.spatial.transform import Rotation as R
from scipy.spatial import ConvexHull


# Realsense D456 intrinsics
CAMERA_MIN_OPERATING_DEPTH_M = 0.6
CAMERA_MAX_OPERATING_DEPTH_M = 6.0
CAMERA_FOV_H_DEG = 86.0  # horizontal
CAMERA_FOV_V_DEG = 57.0  # vertical


class PathAnalyzer:
    """
    End-to-end analysis of a moving camera trajectory.

    Generates geometric, kinematic, and sensor-coverage metrics from a recorded
    tip path (nav_msgs/Path recorded into a CSV) with orientation awareness.
    """

    def __init__( self, csv_file="tip_path.csv", voxel_size=0.05, rays_per_pose=200):
        # Known intrinsics
        self._depth_min = CAMERA_MIN_OPERATING_DEPTH_M
        self._depth_max = CAMERA_MAX_OPERATING_DEPTH_M
        self._fov_h = radians(CAMERA_FOV_H_DEG)
        self._fov_v = radians(CAMERA_FOV_V_DEG)

        # Params
        self._csv_filepath = csv_file
        self._voxel_size = voxel_size
        self._rays_per_pose = rays_per_pose

        # Internal properties
        self._positions = None
        self._rotations = None
        self._voxels = set()

        # Output metrics
        self._total_path_length = 0.0
        self._workspace_bounds = None
        self._hull_volume = 0.0
        self._scanned_volume = 0.0
        self._scan_redundancy = 0.0

    def run(self):
        """
        Runs the full analysis pipeline.
        """
        t_start = time.perf_counter()

        self._load_csv()
        self._compute_path_metrics()
        self._compute_bounding_box()
        self._compute_convex_hull()
        self._estimate_scan_volume()

        
        print("================================")
        print(f"Trajectory length: {self._total_path_length:.3f} m (w/ {len(self._positions)} samples)")
        print(f"Workspace envelope: X={self._workspace_bounds[0]:.3f} m, Y={self._workspace_bounds[1]:.3f} m, Z={self._workspace_bounds[2]:.3f} m")
        print(f"Volume traversed by camera: {self._hull_volume:.3f} m^3")
        print(f"Estimated scanned volume: {self._scanned_volume:.3f} m³")
        print(f"Scan inefficiency (redundancy factor): {self._scan_redundancy:.2f}")

        print("================================")
        t_end = time.perf_counter()
        print(f"Script ran in {t_end - t_start:.3f} s")

    def _load_csv(self):
        print("Loading nav_msgs/Path data from CSV...")

        try:
            # Extract positions and orientations
            df = pd.read_csv(self._csv_filepath)
            self._positions = df[['x', 'y', 'z']].to_numpy()
            quats = df[['qx', 'qy', 'qz', 'qw']].to_numpy()

            # Convert quaternions to scipy Rotations
            self._rotations = R.from_quat(quats)
        except FileNotFoundError:
            print("Loading nav_msgs/Path data from CSV... ERROR")
            print(f"  File not found: {self._csv_filepath}")
            exit(1)
        except Exception as e:
            print("Loading nav_msgs/Path data from CSV... ERROR")
            print(f"  Unexpected error: {e}")
            exit(1)

        print("Loading nav_msgs/Path data from CSV... SUCCESS")

    def _compute_path_metrics(self):
        print("Computing path length and motion statistics...")

        # Compute total path length by summing segment lengths
        diffs = np.diff(self._positions, axis=0)
        segment_lengths = np.linalg.norm(diffs, axis=1)
        self._total_path_length = np.sum(segment_lengths)

        print("Computing path length and motion statistics... SUCCESS")

    def _compute_bounding_box(self):
        """
        Computes the axis-aligned bounding box of the Path.
        """
        print("Computing axis-aligned workspace bounds...")

        # Batch-compute min and max along each axis
        min_xyz = self._positions.min(axis=0)
        max_xyz = self._positions.max(axis=0)
        self._workspace_bounds = max_xyz - min_xyz

        print("Computing axis-aligned workspace bounds... SUCCESS")

    def _compute_convex_hull(self):
        """
        Computes the convex hull created by the Path.
        """
        print("Computing convex hull of trajectory points...")

        # Computing convex hull requires 4 or more non-coplanar points
        if len(self._positions) >= 4:
            hull = ConvexHull(self._positions)
            self._hull_volume = hull.volume
        else:
            print("Computing convex hull of trajectory points... ERROR")
            print(f"  Not enough points for convex hull: {len(self._positions)} (need >= 4)")

        print("Computing convex hull of trajectory points... SUCCESS")

    def _estimate_scan_volume(self):
        """
        Estimates the volume scanned by the camera using orientation-aware
        frustum ray sampling and voxelization.
        """
        print("Estimating orientation-aware scanned volume...")

        # For each pose, sample rays within camera frustum
        for pos, rot in zip(self._positions, self._rotations):
            rays_cam = self._sample_frustum_rays(self._rays_per_pose)
            rays_world = rot.apply(rays_cam) + pos  # transform to world frame

            # Voxelize sampled points
            for p in rays_world:
                self._voxels.add(self._voxel_index(p))

        num_voxels = len(self._voxels)
        self._scanned_volume = num_voxels * (self._voxel_size ** 3)  # in m^3
        total_samples = len(self._positions) * self._rays_per_pose
        self._scan_redundancy = total_samples / max(num_voxels, 1)  # avoid dividing by zero

        print("Estimating orientation-aware scanned volume... SUCCESS")

    def _sample_frustum_rays(self, n):
        """
        Utility function.
        Monte-Carlo samples points within the camera depth frustum
        (i.e., camera optical frame).
        """
        rays = []
        for _ in range(n):
            # Sample depth and normalized image plane coords
            d = np.random.uniform(self._depth_min, self._depth_max)
            u = np.random.uniform(-1.0, 1.0)
            v = np.random.uniform(-1.0, 1.0)

            # Get 3D point in camera frame
            x = d
            y = d * tan(self._fov_h / 2) * u
            z = d * tan(self._fov_v / 2) * v

            rays.append([x, y, z])

        return np.array(rays)

    def _voxel_index(self, point):
        """
        Utility function.
        Converts a 3D point to an integer voxel index.
        """
        return tuple((point / self._voxel_size).astype(int))


if __name__ == "__main__":
    analyzer = PathAnalyzer()
    analyzer.run()
