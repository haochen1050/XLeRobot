#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ruff: noqa: N802, N803, N806  (T_a_b / K / R follow standard transform notation)

"""Table pose from a single AprilTag (36h11) on the table, or from a cube with a tag on each face.

Tag frame (OpenCV ArUco convention): origin at the tag centre, x towards the printed right edge, y towards
the printed top edge, z out of the printed face (up, for a tag lying on the table).
"""

import math
import time
from collections.abc import Callable
from dataclasses import dataclass

import cv2
import numpy as np

from .camera_calibration import CameraIntrinsics
from .geometry import from_rvec_tvec, invert, pose_xyz_yaw
from .table_pose import TablePoseError, TablePoseEstimator, TableTarget, table_pose_error

# Returns (BGR image, capture time on time.monotonic()) or None when no frame is available.
FrameSource = Callable[[], tuple[np.ndarray, float] | None]


TAG_MOUNTS = ("flat", "vertical", "cube")
CUBE_FACES = ("top", "front", "right", "back", "left")
# Rotation about the vertical that takes the front face onto each side face (cube frame, see TagPlacement).
_CUBE_SIDE_YAW_DEG = {"front": 0.0, "right": 90.0, "back": 180.0, "left": -90.0}

# Reference orientation of each mount in the table frame (columns: tag x, y, z axes), before yaw_deg.
_MOUNT_ROTATION = {
    # Lying on the tabletop, face up; tag x into the table, printed top (tag y) to the robot's left.
    "flat": np.eye(3),
    # Hanging on the front edge, face towards the robot; printed top up, tag x to the robot's right.
    "vertical": np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
}


@dataclass(frozen=True)
class TagPlacement:
    """Where the tag sits in the table frame (see table_pose.py)."""

    # Tag centre from the front edge, into the table (0 for a tag hung on the edge itself).
    x_m: float
    # Tag centre along the edge relative to the work-region centre (+ = left as seen by the robot).
    y_m: float = 0.0
    # Rotation of the printed tag about its own face normal, from the mount's reference orientation.
    # Flat: -90 = printed top points into the table. Vertical: 0 = printed top points up.
    yaw_deg: float = -90.0
    size_m: float = 0.10
    # None accepts exactly one visible tag.
    tag_id: int | None = None
    mount: str = "flat"
    # Tag centre height relative to the tabletop (negative = below the top surface). Vertical mount only.
    z_m: float = 0.0
    # Cube mount only. The cube stands on the table, or on a holder (its height does not matter); x_m / y_m are
    # its centre and yaw_deg its rotation about the vertical (0 = the front face square to the robot). The cube
    # frame, used in place of the tag frame, has its origin at the centre of the cube's base, x from the front
    # face into the cube, y left, z up.
    cube_size_m: float = 0.13
    # (face, tag ID) or (face, tag ID, turn_deg) for every face to use; tags with other IDs are ignored. With no
    # turn, side tags have their printed top up and the top tag's printed top points to the right as seen from
    # the front. turn_deg (a multiple of 90) turns the tag from there about the face's outward normal,
    # counter-clockwise as seen from outside the cube: 90 = a side tag's printed top points to the viewer's left.
    # The default is the lab's cube, each face checked on the camera (tag 4 is on the bottom).
    cube_faces: tuple[tuple, ...] = (("top", 5), ("front", 2), ("left", 3), ("right", 1), ("back", 0))

    def __post_init__(self) -> None:
        if self.mount not in TAG_MOUNTS:
            raise ValueError(f"mount must be one of {TAG_MOUNTS}, got {self.mount!r}")
        if self.mount == "cube":
            faces, ids = [f[0] for f in self.cube_faces], [f[1] for f in self.cube_faces]
            if not faces or set(faces) - set(CUBE_FACES) or len(set(faces)) < len(faces) or len(set(ids)) < len(ids):
                raise ValueError(f"cube_faces needs distinct faces from {CUBE_FACES} with distinct IDs: {self.cube_faces}")
            if any(len(f) > 3 or (len(f) == 3 and f[2] % 90 != 0) for f in self.cube_faces):
                raise ValueError(f"a cube face's turn must be a multiple of 90 deg: {self.cube_faces}")
            if self.size_m > self.cube_size_m:
                raise ValueError(f"tag size {self.size_m} m is larger than the cube ({self.cube_size_m} m)")

    def cube_face_poses(self) -> dict[int, np.ndarray]:
        """{tag ID: pose of that face's tag in the cube frame}."""
        s = self.cube_size_m
        poses = {}
        for face, tag_id, *turn in self.cube_faces:
            if face == "top":
                T = pose_xyz_yaw(0.0, 0.0, s, 180.0)
            else:
                T = pose_xyz_yaw(0.0, 0.0, 0.0, _CUBE_SIDE_YAW_DEG[face])
                T[:3, :3] = T[:3, :3] @ _MOUNT_ROTATION["vertical"]
                T[:3, 3] = T[:3, :3] @ np.array([0.0, 0.0, s / 2]) + np.array([0.0, 0.0, s / 2])
            if turn:
                T[:3, :3] = T[:3, :3] @ pose_xyz_yaw(0.0, 0.0, 0.0, turn[0])[:3, :3]
            poses[tag_id] = T
        return poses

    def T_table_tag(self) -> np.ndarray:
        if self.mount == "cube":
            return pose_xyz_yaw(self.x_m, self.y_m, 0.0, self.yaw_deg)
        z = self.z_m if self.mount == "vertical" else 0.0
        T = pose_xyz_yaw(self.x_m, self.y_m, z, 0.0)
        T[:3, :3] = _MOUNT_ROTATION[self.mount] @ pose_xyz_yaw(0, 0, 0, self.yaw_deg)[:3, :3]
        return T

    def up_in_tag(self) -> np.ndarray:
        """World up (table z) expressed in the tag frame."""
        return self.T_table_tag()[2, :3]


@dataclass(frozen=True)
class TagObservation:
    tag_id: int
    T_cam_tag: np.ndarray
    reprojection_px: float


def tag_object_points(size_m: float) -> np.ndarray:
    h = size_m / 2
    # Same order as ArUco corners: top-left, top-right, bottom-right, bottom-left.
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


class AprilTagDetector:
    def __init__(self, intrinsics: CameraIntrinsics, tag_size_m: float):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params
        )
        self.intrinsics = intrinsics
        self._object_points = tag_object_points(tag_size_m)

    def detect(self, image: np.ndarray) -> list[tuple[int, np.ndarray]]:
        """[(tag_id, 4x2 corners)]"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None:
            return []
        return [(int(i), c.reshape(4, 2)) for i, c in zip(ids.ravel(), corners, strict=True)]

    def solve(
        self,
        tag_id: int,
        corners: np.ndarray,
        up_in_cam: np.ndarray | None = None,
        up_in_tag: np.ndarray = np.array([0.0, 0.0, 1.0]),
    ) -> TagObservation:
        """Tag pose in the camera frame.

        A planar square has two near-equivalent PnP solutions. With ``up_in_cam`` (the world up direction in
        camera coordinates) the one that best maps ``up_in_tag`` (the tag-frame direction that should point
        up: the face normal for a flat tag, the printed top for a vertical one) onto up is chosen; otherwise
        the lowest reprojection error.
        """
        _, rvecs, tvecs, errors = cv2.solvePnPGeneric(
            self._object_points,
            corners.astype(np.float32),
            self.intrinsics.camera_matrix,
            self.intrinsics.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        candidates = [
            TagObservation(tag_id, from_rvec_tvec(r, t), float(np.ravel(e)[0]))
            for r, t, e in zip(rvecs, tvecs, errors, strict=True)
        ]
        if up_in_cam is not None:
            return max(candidates, key=lambda o: float((o.T_cam_tag[:3, :3] @ up_in_tag) @ up_in_cam))
        return min(candidates, key=lambda o: o.reprojection_px)


def solve_cube(
    detector: AprilTagDetector,
    matches: list[tuple[int, np.ndarray]],
    face_poses: dict[int, np.ndarray],
    up_in_cam: np.ndarray,
) -> TagObservation:
    """Cube pose in the camera frame from one or more of its face tags.

    The largest face in the image gives a first pose (its planar ambiguity settled by which way is up); with
    more faces, all their corners refine it together, which removes the single-tag wobble.
    """
    tag_id, corners = max(matches, key=lambda m: cv2.contourArea(m[1].astype(np.float32)))
    T_cube_face = face_poses[tag_id]
    first = detector.solve(tag_id, corners, up_in_cam=up_in_cam, up_in_tag=T_cube_face[2, :3])
    T_cam_cube = first.T_cam_tag @ invert(T_cube_face)
    if len(matches) == 1:
        return TagObservation(tag_id, T_cam_cube, first.reprojection_px)

    face_points = detector._object_points.astype(np.float64)
    object_points = np.concatenate(
        [(face_poses[i][:3, :3] @ face_points.T).T + face_poses[i][:3, 3] for i, _ in matches]
    )
    image_points = np.concatenate([c for _, c in matches]).astype(np.float64)
    K, dist = detector.intrinsics.camera_matrix, detector.intrinsics.dist_coeffs
    rvec, _ = cv2.Rodrigues(T_cam_cube[:3, :3])
    _, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        K,
        dist,
        rvec,
        T_cam_cube[:3, 3].reshape(3, 1).copy(),
        useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1))))
    return TagObservation(tag_id, from_rvec_tvec(rvec, tvec), rms)


class AprilTagTableEstimator(TablePoseEstimator):
    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        T_base_cam: np.ndarray,
        placement: TagPlacement,
        target: TableTarget,
        frame_source: FrameSource | None = None,
        *,
        max_age_s: float = 0.3,
        max_reprojection_px: float = 2.0,
        max_tilt_deg: float = 15.0,
        max_heading_deg: float = 45.0,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.detector = AprilTagDetector(intrinsics, placement.size_m)
        self.intrinsics = intrinsics
        self.T_base_cam = T_base_cam
        self.placement = placement
        self.target = target
        self.frame_source = frame_source
        self.max_age_s = max_age_s
        self.max_reprojection_px = max_reprojection_px
        self.min_up_cos = math.cos(math.radians(max_tilt_deg))
        self.max_heading_deg = max_heading_deg
        self.clock = clock
        self._T_tag_table = invert(placement.T_table_tag())
        self._up_in_cam = T_base_cam[:3, :3].T @ np.array([0.0, 0.0, 1.0])
        self._up_in_tag = placement.up_in_tag()
        # Last accepted tag / table poses in the base frame, for logging and target teaching.
        self.last_T_base_tag: np.ndarray | None = None
        self.last_T_base_table: np.ndarray | None = None
        # Horizontal bearing of the tag from the camera in the base frame (+ = to the robot's left), for
        # the last processed frame; None when the tag was not detected. Set even when the frame is rejected
        # by a later check, so a search can turn towards a tag it can see but not yet use.
        self.last_tag_bearing_deg: float | None = None
        # Table pose from the last frame whose tag passed the fit/orientation checks, even if the heading
        # limit then rejected it. Lets the orbit approach work from beyond max_heading_deg.
        self.last_raw_T_base_table: np.ndarray | None = None
        # Smallest distance (px) from the detected tag's corners to the image border in the last processed
        # frame; None when the tag was not detected.
        self.last_tag_margin_px: float | None = None
        # Tag pose in the base frame from the most recent frame whose tag fitted; kept across frames so a
        # planned motion can be checked against it (see predict_tag_margin).
        self.last_fit_T_base_tag: np.ndarray | None = None
        # Corners (tag frame; cube frame for a cube) whose predicted image positions decide whether a planned
        # motion keeps the tag in view. For a cube: the faces of the last fit, the top face until then.
        self._tag_corners = tag_object_points(placement.size_m).astype(np.float64)
        self._cube_faces = placement.cube_face_poses() if placement.mount == "cube" else None
        if self._cube_faces is not None:
            self._tag_corners = self._cube_corners([next(iter(self._cube_faces))])

    def get_table_pose_error(self) -> TablePoseError:
        frame = self.frame_source() if self.frame_source is not None else None
        if frame is None:
            return TablePoseError.invalid(self.clock(), "no camera frame")
        return self.estimate(*frame)

    def _cube_corners(self, tag_ids: list[int]) -> np.ndarray:
        face_points = tag_object_points(self.placement.size_m).astype(np.float64)
        return np.concatenate(
            [(self._cube_faces[i][:3, :3] @ face_points.T).T + self._cube_faces[i][:3, 3] for i in tag_ids]
        )

    def _bearing_deg(self, corners: np.ndarray) -> float:
        """Bearing of the tag centre's viewing ray, projected onto the floor plane of the base frame."""
        centre = corners.reshape(-1, 1, 2).mean(axis=0, keepdims=True).astype(np.float64)
        x, y = cv2.undistortPoints(centre, self.intrinsics.camera_matrix, self.intrinsics.dist_coeffs).ravel()
        ray = self.T_base_cam[:3, :3] @ np.array([x, y, 1.0])
        return math.degrees(math.atan2(ray[1], ray[0]))

    def _margin_px(self, pixels: np.ndarray) -> float:
        w, h = self.intrinsics.image_size
        u, v = pixels.reshape(-1, 2).T
        return float(np.min([u, w - 1 - u, v, h - 1 - v]))

    def predict_tag_margin(self, dx_m: float, dy_m: float, dyaw_deg: float) -> float | None:
        """Predicted smallest distance (px) from the tag's corners to the image border after the base moves by
        (dx, dy, dyaw) in its current frame, from the last fitted tag pose. Negative = (partly) out of view,
        -inf = behind the camera. None if no tag has been fitted yet."""
        if self.last_fit_T_base_tag is None:
            return None
        T_cam_tag = (
            invert(self.T_base_cam)
            @ invert(pose_xyz_yaw(dx_m, dy_m, 0.0, dyaw_deg))
            @ self.last_fit_T_base_tag
        )
        pts = (T_cam_tag[:3, :3] @ self._tag_corners.T).T + T_cam_tag[:3, 3]
        if np.any(pts[:, 2] <= 1e-3):
            return float("-inf")
        K = self.intrinsics.camera_matrix
        pinhole = (K[:2, :2] @ (pts[:, :2] / pts[:, 2:]).T).T + K[:2, 2]
        distorted, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), K, self.intrinsics.dist_coeffs)
        # The distortion polynomial can fold points far outside the view back into the image; the pinhole
        # projection catches those.
        return min(self._margin_px(pinhole), self._margin_px(distorted))

    def estimate(self, image: np.ndarray, capture_time_s: float) -> TablePoseError:
        self.last_tag_bearing_deg = None
        self.last_raw_T_base_table = None
        self.last_tag_margin_px = None
        age = self.clock() - capture_time_s
        if age > self.max_age_s:
            return TablePoseError.invalid(capture_time_s, f"stale frame ({age:.2f}s old)")
        h, w = image.shape[:2]
        if (w, h) != tuple(self.intrinsics.image_size):
            return TablePoseError.invalid(
                capture_time_s, f"image {w}x{h} does not match calibration {self.intrinsics.image_size}"
            )

        detections = self.detector.detect(image)
        seen = [d[0] for d in detections]
        if self._cube_faces is not None:
            matches = [d for d in detections if d[0] in self._cube_faces]
            # The same ID twice means a second cube or a false detection: which is which is unknown.
            usable = bool(matches) and len({d[0] for d in matches}) == len(matches)
            expected = f"cube tags {sorted(self._cube_faces)}"
        else:
            wanted = self.placement.tag_id
            matches = [d for d in detections if wanted is None or d[0] == wanted]
            usable = len(matches) == 1
            expected = "one tag" if wanted is None else f"tag {wanted}"
        if not usable:
            return TablePoseError.invalid(capture_time_s, f"expected {expected}, saw {seen}")
        pixels = np.concatenate([corners for _, corners in matches])
        self.last_tag_bearing_deg = self._bearing_deg(pixels)
        self.last_tag_margin_px = self._margin_px(pixels)

        if self._cube_faces is not None:
            obs = solve_cube(self.detector, matches, self._cube_faces, self._up_in_cam)
        else:
            obs = self.detector.solve(*matches[0], up_in_cam=self._up_in_cam, up_in_tag=self._up_in_tag)
        if obs.reprojection_px > self.max_reprojection_px:
            return TablePoseError.invalid(
                capture_time_s, f"reprojection {obs.reprojection_px:.2f}px too high"
            )

        T_base_tag = self.T_base_cam @ obs.T_cam_tag
        T_base_table = T_base_tag @ self._T_tag_table
        # The table's up axis, as implied by the tag and its configured mount, must point up.
        if T_base_table[2, 2] < self.min_up_cos:
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, T_base_table[2, 2]))))
            return TablePoseError.invalid(
                capture_time_s,
                f"tag tilted {tilt:.0f} deg from its expected '{self.placement.mount}' orientation "
                "(check --tag-mount / --tag-yaw-deg)",
            )

        self.last_raw_T_base_table = T_base_table
        self.last_fit_T_base_tag = T_base_tag
        if self._cube_faces is not None:
            self._tag_corners = self._cube_corners([d[0] for d in matches])
        heading = math.degrees(math.atan2(T_base_table[1, 0], T_base_table[0, 0]))
        if abs(heading) > self.max_heading_deg:
            quarter = 90 * round(heading / 90)
            if quarter != 0 and abs(heading - quarter) < 20:
                # Close to a multiple of 90: almost certainly the physical tag is turned relative to
                # TagPlacement.yaw_deg. Acting on it would send the robot to the wrong side of the tag.
                hint = f"tag probably rotated ~{quarter:+d} deg relative to its configured yaw"
            else:
                hint = "robot is off to the side of the table's approach direction"
            return TablePoseError.invalid(
                capture_time_s,
                f"table heading {heading:+.0f} deg exceeds +-{self.max_heading_deg:g}; {hint}",
            )

        self.last_T_base_tag = T_base_tag
        self.last_T_base_table = T_base_table
        return table_pose_error(T_base_table, self.target, capture_time_s)
