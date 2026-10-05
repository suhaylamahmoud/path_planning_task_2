from __future__ import annotations

import math
from typing import List, Tuple

from src.models import CarPose, Cone, Path2D


class PathPlanning:

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        
        DEFAULT_HALF_WIDTH = 1.25  # track half-width / full width = 2.5 m
        MIN_PATH_LEN = 6.5 #output path length between 6.5 m & 8.5 m
        MAX_PATH_LEN = 8.5
        STEP = 0.25  

    #car pose extraction
        cx, cy, cyaw = self.car_pose.x, self.car_pose.y, self.car_pose.yaw
    #precomputation of the forward heading unit vector components for planar projections
        head_x, head_y = math.cos(cyaw), math.sin(cyaw)

    #separating cones based on colour
        blue_cones = [c for c in self.cones if c.color == 1]
        yellow_cones = [c for c in self.cones if c.color == 0]

    # part 2 w kda : Polyline offset for 1, 2, 3 or more cones on the same side
        def get_single_side_midpoints(cones_list: List[Cone], is_left_blue: bool) -> List[Tuple[float, float]]:
            if not cones_list:
                return []

            # 1 cone: offset relative to vehicle heading
            if len(cones_list) == 1:
                c = cones_list[0]
                if is_left_blue:
                    # blue (left) -> shift right (-90 deg from heading)
                    return [(c.x + head_y * DEFAULT_HALF_WIDTH, c.y - head_x * DEFAULT_HALF_WIDTH)]
                else:
                    # yellow (right) -> shift left (+90 deg from heading)
                    return [(c.x - head_y * DEFAULT_HALF_WIDTH, c.y + head_x * DEFAULT_HALF_WIDTH)]

            # 2 or 3+ cones: sort along forward progression
            sorted_cones = sorted(cones_list, key=lambda c: (c.x - cx) * head_x + (c.y - cy) * head_y)
            pts = [(c.x, c.y) for c in sorted_cones]
            result = []

            for i in range(len(pts)):
                # determine local segment tangent
                if i < len(pts) - 1:
                    dx = pts[i + 1][0] - pts[i][0]
                    dy = pts[i + 1][1] - pts[i][1]
                else:
                    dx = pts[i][0] - pts[i - 1][0]
                    dy = pts[i][1] - pts[i - 1][1]

                seg_len = math.hypot(dx, dy)
                if seg_len > 1e-4:
                    tx, ty = dx / seg_len, dy / seg_len
                else:
                    tx, ty = head_x, head_y

                # make sure tangent flows away from the vehicle progression
                if (pts[i][0] - cx) * tx + (pts[i][1] - cy) * ty < -0.5:
                    tx, ty = -tx, -ty

                # left normal (+90 deg): (-ty, tx); right normal (-90 deg): (ty, -tx)
                if is_left_blue:
                    # blue cone is on LEFT -> path is to its RIGHT !!
                    nx, ny = ty, -tx
                else:
                    # yellow cone is on RIGHT -> path is to its LEFT !!
                    nx, ny = -ty, tx

                result.append((pts[i][0] + nx * DEFAULT_HALF_WIDTH, pts[i][1] + ny * DEFAULT_HALF_WIDTH))

            return result

        # 1- gate pairing & virtual centerline generation

        midpoints: List[Tuple[float, float]] = []

        if blue_cones and yellow_cones:
            candidate_pairs = []
            for bi, b in enumerate(blue_cones):
                for yj, y in enumerate(yellow_cones):
                    d = math.hypot(b.x - y.x, b.y - y.y)
                    if d <= 5.5:
                        candidate_pairs.append((d, bi, yj))

            candidate_pairs.sort(key=lambda item: item[0])
            matched_b, matched_y = set(), set()
            paired_gates = []
            for _, bi, yj in candidate_pairs:
                if bi not in matched_b and yj not in matched_y:
                    matched_b.add(bi)
                    matched_y.add(yj)
                    paired_gates.append((blue_cones[bi], yellow_cones[yj]))

            # add actual gate midpoints
            for b, y in paired_gates:
                midpoints.append(((b.x + y.x) * 0.5, (b.y + y.y) * 0.5))

            # determining track direction
            if len(midpoints) >= 2:
                track_dir = (midpoints[-1][0] - midpoints[0][0], midpoints[-1][1] - midpoints[0][1])
                t_norm = math.hypot(track_dir[0], track_dir[1])
                track_unit = (track_dir[0] / t_norm, track_dir[1] / t_norm) if t_norm > 1e-4 else (head_x, head_y)
            else:
                track_unit = (head_x, head_y)

            # left normal (+90 deg) and right normal (-90 deg)
            left_nx, left_ny = -track_unit[1], track_unit[0]
            right_nx, right_ny = track_unit[1], -track_unit[0]

            # unpaired cones
            for bi, b in enumerate(blue_cones):
                if bi not in matched_b:
                    midpoints.append((b.x + right_nx * DEFAULT_HALF_WIDTH, b.y + right_ny * DEFAULT_HALF_WIDTH))
            for yj, y in enumerate(yellow_cones):
                if yj not in matched_y:
                    midpoints.append((y.x + left_nx * DEFAULT_HALF_WIDTH, y.y + left_ny * DEFAULT_HALF_WIDTH))

        elif yellow_cones:
            # YELLOW CONES BASSSS(1, 2, or 3+ cones)
            midpoints.extend(get_single_side_midpoints(yellow_cones, is_left_blue=False))

        elif blue_cones:
            # BLUE CONES BASSSS(1, 2, or 3+ cones)
            midpoints.extend(get_single_side_midpoints(blue_cones, is_left_blue=True))


        # 2. sequential ordering of centerline waypoints

        def forward_rank(pt: Tuple[float, float]) -> float:
            dx, dy = pt[0] - cx, pt[1] - cy
            longitudinal = dx * head_x + dy * head_y
            return longitudinal + 0.4 * math.hypot(dx, dy)

        midpoints.sort(key=forward_rank)

        # remove duplicate or clustered points
        cleaned_targets: List[Tuple[float, float]] = []
        for p in midpoints:
            if not cleaned_targets or math.hypot(p[0] - cleaned_targets[-1][0], p[1] - cleaned_targets[-1][1]) > 0.4:
                cleaned_targets.append(p)


        # 3. path generation: initial turn + centerline passage

        dense_path: List[Tuple[float, float]] = [(cx, cy)]

        if cleaned_targets:
            first_target = cleaned_targets[0]
            dx_first = first_target[0] - cx
            dy_first = first_target[1] - cy
            dist_first = math.hypot(dx_first, dy_first)

            if dist_first > 0.3:
                if len(cleaned_targets) > 1:
                    t_dx = cleaned_targets[1][0] - first_target[0]
                    t_dy = cleaned_targets[1][1] - first_target[1]
                    t_len = math.hypot(t_dx, t_dy)
                    tx_end, ty_end = (t_dx / t_len, t_dy / t_len) if t_len > 1e-4 else (head_x, head_y)
                else:
                    tx_end, ty_end = head_x, head_y

                scale = dist_first * 0.7
                v0 = (head_x * scale, head_y * scale)
                v1 = (tx_end * scale, ty_end * scale)

                steps_arc = max(4, int(dist_first / STEP))
                for s in range(1, steps_arc):
                    t = s / steps_arc
                    h00 = 2 * t**3 - 3 * t**2 + 1
                    h10 = t**3 - 2 * t**2 + t
                    h01 = -2 * t**3 + 3 * t**2
                    h11 = t**3 - t**2
                    px = h00 * cx + h10 * v0[0] + h01 * first_target[0] + h11 * v1[0]
                    py = h00 * cy + h10 * v0[1] + h01 * first_target[1] + h11 * v1[1]
                    dense_path.append((px, py))

            dense_path.extend(cleaned_targets)
        else:
            dense_path.append((cx + head_x * 2.0, cy + head_y * 2.0))

        # 4. extrapolate forward to required length (6.5m - 8.5m)

        while True:
            cur_len = sum(
                math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                for p1, p2 in zip(dense_path, dense_path[1:])
            )
            if cur_len >= MIN_PATH_LEN or len(dense_path) < 2:
                break
            dx = dense_path[-1][0] - dense_path[-2][0]
            dy = dense_path[-1][1] - dense_path[-2][1]
            mag = math.hypot(dx, dy)
            if mag < 1e-6:
                dx, dy, mag = head_x, head_y, 1.0
            dense_path.append((dense_path[-1][0] + (dx / mag) * 1.5, dense_path[-1][1] + (dy / mag) * 1.5))


        # 5. arc-length resampling

        cum_dist = [0.0]
        for p1, p2 in zip(dense_path, dense_path[1:]):
            cum_dist.append(cum_dist[-1] + math.hypot(p2[0] - p1[0], p2[1] - p1[1]))

        total_len = min(cum_dist[-1], MAX_PATH_LEN)
        num_output_points = max(2, int(total_len / STEP))

        path: Path2D = []
        seg = 0
        for i in range(1, num_output_points + 1):
            s = (i / num_output_points) * total_len
            while seg < len(cum_dist) - 2 and cum_dist[seg + 1] < s:
                seg += 1
            span = cum_dist[seg + 1] - cum_dist[seg]
            t = 0.0 if span < 1e-6 else (s - cum_dist[seg]) / span
            p1, p2 = dense_path[seg], dense_path[seg + 1]
            path.append((p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1])))

        return path