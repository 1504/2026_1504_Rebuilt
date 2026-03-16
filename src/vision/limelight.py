"""
Team 1504 - LimelightVision
Feeds AprilTag pose estimates into the drive subsystem's pose estimator.
"""

import math
import commands2
import wpilib
import wpimath.geometry
import ntcore
from wpilib import SmartDashboard

from src.constants import VisionConstants
from src.subsystems.drive import DriveSubsystem


class LimelightVision(commands2.Subsystem):
    def __init__(self, drive: DriveSubsystem) -> None:
        super().__init__()
        self._drive = drive
        self._nt    = ntcore.NetworkTableInstance.getDefault()
        self._table = self._nt.getTable(VisionConstants.kLimelightName)

        # ── NT subscribers ─────────────────────────────────────────
        self._botpose_sub   = self._table.getDoubleArrayTopic("botpose_wpiblue").subscribe([])
        self._tv_sub        = self._table.getIntegerTopic("tv").subscribe(0)
        self._tid_sub       = self._table.getIntegerTopic("tid").subscribe(-1)
        self._tag_count_sub = self._table.getIntegerTopic("botpose_tagcount").subscribe(0)
        self._ta_sub        = self._table.getDoubleTopic("ta").subscribe(0.0)
        self._tx_sub        = self._table.getDoubleTopic("tx").subscribe(0.0)
        self._ty_sub        = self._table.getDoubleTopic("ty").subscribe(0.0)
        self._pipeline_pub  = self._table.getIntegerTopic("pipeline").publish()
        self._megatag2_sub  = self._table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe([])

        # ── State tracking ─────────────────────────────────────────
        self._last_accepted_ts: float = 0.0
        self._accepted_count: int = 0
        self._rejected_count: int = 0

    # ─────────────────────────────────────────────────────────────
    # PERIODIC
    # ─────────────────────────────────────────────────────────────
    def periodic(self) -> None:
        # ── Gate 1: valid target ───────────────────────────────────
        # FIXED: "No Target" is normal — not a rejection worth counting.
        # Counting it inflated _rejected_count by 50 Hz during any moment
        # without visible tags (most of the match).  Only gates that
        # receive actual pose data and then filter it out are real rejections.
        if self._tv_sub.get() == 0:
            SmartDashboard.putString("Vision/Status", "No Target")
            SmartDashboard.putBoolean("Vision/PoseAccepted", False)
            SmartDashboard.putNumber("Vision/LatencyMs", 0.0)
            return

        # ── Gate 2: prefer MegaTag2 if available ──────────────────
        megatag2 = self._megatag2_sub.get()
        botpose  = self._botpose_sub.get()

        use_megatag2 = len(megatag2) >= 7
        pose_data    = megatag2 if use_megatag2 else botpose

        if len(pose_data) < 7:
            self._publish_rejection("Bad Pose Data", latency=0.0)
            return

        x, y, _z, _roll, _pitch, yaw_deg, latency_ms = pose_data[:7]

        # ── Gate 3: field bounds check ─────────────────────────────
        if not (VisionConstants.kFieldMinX < x < VisionConstants.kFieldMaxX) or \
           not (VisionConstants.kFieldMinY < y < VisionConstants.kFieldMaxY):
            self._publish_rejection("Out of Bounds", latency=latency_ms)
            return

        # ── Gate 4: heading sanity check (classic botpose only) ────
        if not use_megatag2:
            gyro_deg  = self._drive.get_heading_degrees()
            yaw_error = abs(_angle_diff(yaw_deg, gyro_deg))
            if yaw_error > VisionConstants.kMaxYawErrorDeg:
                self._publish_rejection(
                    f"Yaw Mismatch ({yaw_error:.1f}°)", latency=latency_ms
                )
                return

        # ── Gate 5: velocity gating ────────────────────────────────
        chassis   = self._drive.get_chassis_speeds()
        speed_mps = math.hypot(chassis.vx, chassis.vy)
        if speed_mps > VisionConstants.kMaxVisionSpeedMps:
            self._publish_rejection(
                f"Too Fast ({speed_mps:.1f} m/s)", latency=latency_ms
            )
            return

        # ── Compute std devs and inject pose ──────────────────────
        tag_count = int(self._tag_count_sub.get())
        std_devs  = self._compute_std_devs(x, y, tag_count, use_megatag2)

        pose = wpimath.geometry.Pose2d(
            wpimath.geometry.Translation2d(x, y),
            wpimath.geometry.Rotation2d.fromDegrees(yaw_deg),
        )
        timestamp = wpilib.Timer.getFPGATimestamp() - (latency_ms / 1000.0)
        self._drive.add_vision_measurement(pose, timestamp, std_devs)

        self._last_accepted_ts = wpilib.Timer.getFPGATimestamp()
        self._accepted_count  += 1

        status = f"{'MegaTag2' if use_megatag2 else 'Classic'} | {tag_count} tag(s)"
        SmartDashboard.putString("Vision/Status",        status)
        SmartDashboard.putBoolean("Vision/PoseAccepted", True)
        SmartDashboard.putNumber("Vision/LatencyMs",     latency_ms)
        SmartDashboard.putNumber("Vision/AcceptedCount", self._accepted_count)
        SmartDashboard.putNumber("Vision/RejectedCount", self._rejected_count)
        SmartDashboard.putNumber("Vision/TX",            self._tx_sub.get())
        SmartDashboard.putNumber("Vision/TY",            self._ty_sub.get())
        SmartDashboard.putNumber("Vision/TargetArea",    self._ta_sub.get())
        SmartDashboard.putNumber("Vision/TagID",         self._tid_sub.get())

    # ─────────────────────────────────────────────────────────────
    # HELPERS
    # ─────────────────────────────────────────────────────────────
    def _publish_rejection(self, status: str, latency: float) -> None:
        """Publish telemetry for a pose that was seen but filtered out.
        Only called when actual pose data was received and then rejected —
        not for the routine no-target case.
        """
        self._rejected_count += 1
        SmartDashboard.putString("Vision/Status",        status)
        SmartDashboard.putBoolean("Vision/PoseAccepted", False)
        SmartDashboard.putNumber("Vision/LatencyMs",     latency)
        SmartDashboard.putNumber("Vision/AcceptedCount", self._accepted_count)
        SmartDashboard.putNumber("Vision/RejectedCount", self._rejected_count)
        SmartDashboard.putNumber("Vision/TX",            self._tx_sub.get())
        SmartDashboard.putNumber("Vision/TY",            self._ty_sub.get())
        SmartDashboard.putNumber("Vision/TargetArea",    self._ta_sub.get())
        SmartDashboard.putNumber("Vision/TagID",         self._tid_sub.get())

    def _compute_std_devs(
        self,
        x: float,
        y: float,
        tag_count: int,
        use_megatag2: bool,
    ) -> tuple[float, float, float]:
        robot_pose = self._drive.get_pose()
        dist       = math.hypot(x - robot_pose.X(), y - robot_pose.Y())
        dist_scale = 1.0 + VisionConstants.kDistanceScaleFactor * dist * dist

        if tag_count >= 2:
            xy_std  = VisionConstants.kMultiTagStdDevs[0]  * dist_scale
            rot_std = VisionConstants.kMultiTagStdDevs[2]
        else:
            xy_std  = VisionConstants.kSingleTagStdDevs[0] * dist_scale
            rot_std = VisionConstants.kSingleTagStdDevs[2]

        # MegaTag2 fuses gyro into heading — never let vision override rotation
        if use_megatag2:
            rot_std = 9999.0

        return (xy_std, xy_std, rot_std)

    # ─────────────────────────────────────────────────────────────
    # PUBLIC HELPERS
    # ─────────────────────────────────────────────────────────────
    def has_target(self) -> bool:
        return self._tv_sub.get() == 1

    def get_target_id(self) -> int:
        return int(self._tid_sub.get())

    def get_tx(self) -> float:
        return self._tx_sub.get()

    def get_ty(self) -> float:
        return self._ty_sub.get()

    def get_target_area(self) -> float:
        return self._ta_sub.get()

    def get_distance_to_target(self) -> float | None:
        if not self.has_target():
            return None
        angle_to_target_deg = VisionConstants.kCameraPitch + self._ty_sub.get()
        if angle_to_target_deg <= 0:
            return None
        return (
            (VisionConstants.kTagHeightMeters - VisionConstants.kCameraHeightMeters)
            / math.tan(math.radians(angle_to_target_deg))
        )

    def set_pipeline(self, index: int) -> None:
        self._pipeline_pub.set(index)

    def time_since_last_accepted(self) -> float:
        return wpilib.Timer.getFPGATimestamp() - self._last_accepted_ts


def _angle_diff(a_deg: float, b_deg: float) -> float:
    """Shortest signed difference between two angles (degrees). Result in (-180, 180]."""
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0