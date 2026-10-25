#!/usr/bin/env python3
"""
frontier_exploration.py — Real Frontier Exploration for Occupancy Grid Maps.

Finds boundaries between explored free space and unexplored unknown space,
clusters them, inflates obstacles for safe robot clearance, and computes
optimal navigation goal coordinates for SLAM mapping.
"""

import math
import numpy as np
import cv2
from typing import List, Tuple, Optional, Dict


class FrontierExploration:
    def __init__(
        self,
        min_cluster_size: int = 4,
        robot_radius_m: float = 0.35,
        max_goal_distance_m: float = 10.0,
    ):
        self.min_cluster_size = min_cluster_size
        self.robot_radius_m = robot_radius_m
        self.max_goal_distance_m = max_goal_distance_m

    def extract_frontiers(
        self,
        grid_data: np.ndarray,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        robot_x: float = 0.0,
        robot_y: float = 0.0,
    ) -> Tuple[List[Dict], np.ndarray, Dict]:
        """
        Analyze 2D occupancy grid and return list of ranked frontier goals.

        grid_data: 1D or 2D array of int8 (-1=unknown, 0=free, 100=occupied)
        Returns:
            frontier_goals: List of dicts [{'x': float, 'y': float, 'area': int, 'distance': float}]
            safe_frontier_mask: 2D uint8 mask of valid frontier cells
            stats: Dict with exploration summary metrics
        """
        if grid_data.ndim == 1:
            grid = grid_data.reshape((height, width))
        else:
            grid = grid_data

        # 1. Segment occupancy grid
        free_mask = (grid == 0).astype(np.uint8)
        unknown_mask = (grid == -1).astype(np.uint8)
        occupied_mask = (grid > 50).astype(np.uint8)

        total_free_cells = int(np.sum(free_mask))
        total_unknown_cells = int(np.sum(unknown_mask))
        explored_area_m2 = round(total_free_cells * resolution * resolution, 2)

        # If no free space or no unknown space, no frontiers exist
        if total_free_cells == 0 or total_unknown_cells == 0:
            return [], np.zeros((height, width), dtype=np.uint8), {
                'total_frontiers': 0,
                'explored_area_m2': explored_area_m2,
                'is_fully_mapped': total_free_cells > 200 and total_unknown_cells == 0,
            }

        # 2. Frontier detection: Free cells adjacent to unknown cells
        # Dilate unknown cells by 3x3 kernel
        kernel_3 = np.ones((3, 3), np.uint8)
        dilated_unknown = cv2.dilate(unknown_mask, kernel_3)
        raw_frontier = cv2.bitwise_and(free_mask, dilated_unknown)

        # 3. Obstacle clearance: Inflate obstacles by robot safety radius
        obs_radius_cells = max(3, int(math.ceil(self.robot_radius_m / resolution)))
        kernel_size = 2 * obs_radius_cells + 1
        obs_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        dilated_obs = cv2.dilate(occupied_mask, obs_kernel)

        # Filter out frontiers too close to obstacles
        safe_frontier = cv2.bitwise_and(raw_frontier, cv2.bitwise_not(dilated_obs))

        # 4. Connected components clustering
        num_labels, labels, cc_stats, centroids = cv2.connectedComponentsWithStats(
            safe_frontier, connectivity=8
        )

        frontier_goals = []

        # Iterate over all detected clusters (label 0 is background)
        for label in range(1, num_labels):
            area = cc_stats[label, cv2.CC_STAT_AREA]
            if area < self.min_cluster_size:
                continue

            cx, cy = centroids[label]
            px, py = int(round(cx)), int(round(cy))

            # Ensure the goal point itself is within safe free space
            if (
                0 <= px < width
                and 0 <= py < height
                and free_mask[py, px] == 1
                and dilated_obs[py, px] == 0
            ):
                goal_gx, goal_gy = px, py
            else:
                # Find the cell in this cluster that is free and furthest from obstacles
                cluster_pts = np.argwhere(labels == label)  # rows (y), cols (x)
                if len(cluster_pts) == 0:
                    continue
                # Pick the point closest to centroid that is safe
                dists_to_c = (cluster_pts[:, 1] - cx) ** 2 + (cluster_pts[:, 0] - cy) ** 2
                best_idx = np.argmin(dists_to_c)
                goal_gy, goal_gx = cluster_pts[best_idx]

            # Convert to world coordinates (meters)
            world_x = origin_x + (goal_gx + 0.5) * resolution
            world_y = origin_y + (goal_gy + 0.5) * resolution

            dist = math.hypot(world_x - robot_x, world_y - robot_y)
            if dist > self.max_goal_distance_m:
                continue

            # Heading towards frontier
            yaw_deg = math.degrees(math.atan2(world_y - robot_y, world_x - robot_x))

            # Score: combination of cluster size and proximity
            # Larger cluster = more unknown area to map; closer = faster to reach
            score = area / (1.0 + 0.8 * dist)

            frontier_goals.append({
                'x': round(world_x, 3),
                'y': round(world_y, 3),
                'yaw_deg': round(yaw_deg, 1),
                'area': int(area),
                'distance': round(dist, 2),
                'score': round(score, 2),
            })

        # Sort by score descending (best frontier first)
        frontier_goals.sort(key=lambda f: f['score'], reverse=True)

        is_fully_mapped = (len(frontier_goals) == 0 and total_free_cells > 200)

        stats = {
            'total_frontiers': len(frontier_goals),
            'explored_area_m2': explored_area_m2,
            'is_fully_mapped': is_fully_mapped,
            'total_free_cells': total_free_cells,
            'total_unknown_cells': total_unknown_cells,
        }

        return frontier_goals, safe_frontier, stats
