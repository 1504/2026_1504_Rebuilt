"""
Team 1504 - LimelightVision
MegaTag2 pose injection + pose-based distance to the Hub for shooting.

Distance method: robot pose → known Hub tag position from WPILib field layout.
We never use TY-based geometry here because the camera pitch is 0° (flat),
making the tangent formula produce garbage at shallow angles.

Bump correction: vision is accepted at higher speed over the bump, but std devs
are multiplied so the pose correction is a gentle drift rather than a hard jump.

⚠️ Likely mistake — AprilTagFields enum name:
   The exact name for 2026 Rebuilt depends on your robotpy-apriltag version.
   If the layout fails to load on boot, run this in a test to find the right name:
       from wpilib.apriltag import AprilTagFields
       print([x for x in dir(AprilTagFields) if '2026' in x])
   Then update the loadField() call below.
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

        # Load the WPILib 2026 field layout so we can look up Hub tag positions.
        # ⚠️ If this throws on boot, see module docstring for how to find the right name.
        self._tag_layout = None
        try:
            from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
            self._tag_layout = AprilTagFieldLayout.loadField(
                AprilTagField.k2026RebuiltWelded
            )
        except Exception as e:
            wpilib.reportWarning(
                f"[Vision] AprilTag layout load failed: {e} — hub distance unavailable",
                printTrace=False,
            )

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

        self._last_accepted_ts: float = 0.0
        self._accepted_count: int = 0
        self._rejected_count: int = 0

    # ─────────────────────────────────────────────────────────────
    # PERIODIC
    # ─────────────────────────────────────────────────────────────
    def periodic(self) -> None:
        # No target is normal — don't count it as a rejection.
        if self._tv_sub.get() == 0:
            SmartDashboard.putString("Vision/Status", "No Target")
            SmartDashboard.putBoolean("Vision/PoseAccepted", False)
            SmartDashboard.putNumber("Vision/LatencyMs", 0.0)
            return

        # Prefer MegaTag2 (gyro-fused, more stable) over classic botpose.
        megatag2     = self._megatag2_sub.get()
        botpose      = self._botpose_sub.get()
        use_megatag2 = len(megatag2) >= 7
        pose_data    = megatag2 if use_megatag2 else botpose

        if len(pose_data) < 7:
            self._publish_rejection("Bad Pose Data", latency=0.0)
            return

        x, y, _z, _roll, _pitch, yaw_deg, latency_ms = pose_data[:7]

        # Gate: field bounds
        if not (VisionConstants.kFieldMinX < x < VisionConstants.kFieldMaxX) or \
           not (VisionConstants.kFieldMinY < y < VisionConstants.kFieldMaxY):
            self._publish_rejection("Out of Bounds", latency=latency_ms)
            return

        # Gate: heading sanity (classic botpose only — MegaTag2 already fuses gyro)
        if not use_megatag2:
            yaw_error = abs(_angle_diff(yaw_deg, self._drive.get_heading_degrees()))
            if yaw_error > VisionConstants.kMaxYawErrorDeg:
                self._publish_rejection(f"Yaw {yaw_error:.1f}°", latency=latency_ms)
                return

        # Gate: speed — relaxed over the bump, inflated std devs instead of rejection
        chassis   = self._drive.get_chassis_speeds()
        speed_mps = math.hypot(chassis.vx, chassis.vy)

        if speed_mps > VisionConstants.kBumpCorrectionSpeedMps:
            self._publish_rejection(f"Too Fast {speed_mps:.1f}m/s", latency=latency_ms)
            return

        # Above normal gate but below bump gate → accept with inflated uncertainty
        speed_mult = (
            VisionConstants.kBumpStdDevMultiplier
            if speed_mps > VisionConstants.kMaxVisionSpeedMps
            else 1.0
        )

        tag_count = int(self._tag_count_sub.get())
        std_devs  = self._compute_std_devs(x, y, tag_count, use_megatag2, speed_mult)

        pose = wpimath.geometry.Pose2d(
            wpimath.geometry.Translation2d(x, y),
            wpimath.geometry.Rotation2d.fromDegrees(yaw_deg),
        )
        timestamp = wpilib.Timer.getFPGATimestamp() - (latency_ms / 1000.0)
        self._drive.add_vision_measurement(pose, timestamp, std_devs)

        self._last_accepted_ts = wpilib.Timer.getFPGATimestamp()
        self._accepted_count  += 1

        dist = self.get_distance_to_hub()
        SmartDashboard.putString("Vision/Status",
            f"{'MegaTag2' if use_megatag2 else 'Classic'} | {tag_count} tag(s)")
        SmartDashboard.putBoolean("Vision/PoseAccepted", True)
        SmartDashboard.putNumber("Vision/LatencyMs",     latency_ms)
        SmartDashboard.putNumber("Vision/AcceptedCount", self._accepted_count)
        SmartDashboard.putNumber("Vision/RejectedCount", self._rejected_count)
        SmartDashboard.putNumber("Vision/TX",            self._tx_sub.get())
        SmartDashboard.putNumber("Vision/TY",            self._ty_sub.get())
        SmartDashboard.putNumber("Vision/TargetArea",    self._ta_sub.get())
        SmartDashboard.putNumber("Vision/TagID",         self._tid_sub.get())
        SmartDashboard.putNumber("Vision/HubDistM",      dist if dist is not None else -1.0)

    # ─────────────────────────────────────────────────────────────
    # DISTANCE — pose-based, not TY-based
    # ─────────────────────────────────────────────────────────────
    def get_distance_to_hub(self) -> float | None:
        """
        Returns robot-to-Hub distance (meters) using robot pose + WPILib field layout.
        Filters to alliance-appropriate Hub tag IDs — returns None if the currently
        visible tag isn't a Hub tag for our alliance, or if layout failed to load.

        ⚠️ Returns None until the field layout loads successfully on boot.
           If this stays None all match, check the AprilTagFields enum name.
        """
        if self._tag_layout is None:
            return None

        tid = int(self._tid_sub.get())
        if tid < 0:
            return None

        alliance = wpilib.DriverStation.getAlliance()
        hub_ids  = (
            VisionConstants.kRedHubTagIds
            if alliance == wpilib.DriverStation.Alliance.kRed
            else VisionConstants.kBlueHubTagIds
        )
        if tid not in hub_ids:
            return None

        tag_pose = self._tag_layout.getTagPose(tid)
        if tag_pose is None:
            return None

        robot = self._drive.get_pose()
        return math.hypot(robot.X() - tag_pose.x, robot.Y() - tag_pose.y)

    # Alias so DriveToShootCommand works with either name
    def get_distance_to_target(self) -> float | None:
        return self.get_distance_to_hub()

    # ─────────────────────────────────────────────────────────────
    # HELPERS
    # ─────────────────────────────────────────────────────────────
    def _publish_rejection(self, status: str, latency: float) -> None:
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
        speed_multiplier: float = 1.0,
    ) -> tuple[float, float, float]:
        robot = self._drive.get_pose()
        dist  = math.hypot(x - robot.X(), y - robot.Y())
        # Scale grows with distance² — farther tags are noisier
        scale = (1.0 + VisionConstants.kDistanceScaleFactor * dist * dist) * speed_multiplier

        if tag_count >= 2:
            xy_std  = VisionConstants.kMultiTagStdDevs[0]  * scale
            rot_std = VisionConstants.kMultiTagStdDevs[2]
        else:
            xy_std  = VisionConstants.kSingleTagStdDevs[0] * scale
            rot_std = VisionConstants.kSingleTagStdDevs[2]

        if use_megatag2:
            rot_std = 9999.0  # MegaTag2 fuses gyro — never override rotation

        return (xy_std, xy_std, rot_std)

    # ─────────────────────────────────────────────────────────────
    # PUBLIC ACCESSORS
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

    def set_pipeline(self, index: int) -> None:
        self._pipeline_pub.set(index)

    def time_since_last_accepted(self) -> float:
        return wpilib.Timer.getFPGATimestamp() - self._last_accepted_ts


def _angle_diff(a_deg: float, b_deg: float) -> float:
    """Shortest signed difference between two angles in degrees. Result in (-180, 180]."""
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0