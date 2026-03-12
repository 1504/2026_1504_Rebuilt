"""
Team 1504 - LimelightVision
Feeds AprilTag pose estimates into the drive subsystem's pose estimator.

Inspired by 6328's VirtualSubsystem pattern:
  - No command requirements (never interferes with drive commands)
  - Runs every loop via periodic()
  - Trusts multi-tag results more than single-tag

Usage:
  In RobotContainer, create LimelightVision(drive).
  It self-registers its periodic via Subsystem.
"""

import commands2
import wpilib
import wpimath.geometry
import ntcore

from src.constants import VisionConstants


class LimelightVision(commands2.Subsystem):
    def __init__(self, drive) -> None:
        super().__init__()
        self._drive = drive
        self._nt = ntcore.NetworkTableInstance.getDefault()
        self._table = self._nt.getTable(VisionConstants.kLimelightName)

        # Cached NT subscribers for efficiency
        self._botpose_sub  = self._table.getDoubleArrayTopic("botpose_wpiblue").subscribe([])
        self._tv_sub       = self._table.getIntegerTopic("tv").subscribe(0)
        self._tid_sub      = self._table.getIntegerTopic("tid").subscribe(-1)
        # FIXED: was getDoubleTopic — botpose_tagcount is published as an integer
        self._tag_count_sub = self._table.getIntegerTopic("botpose_tagcount").subscribe(0)

    def periodic(self) -> None:
        # No valid target → skip
        if self._tv_sub.get() == 0:
            return

        botpose = self._botpose_sub.get()
        if len(botpose) < 7:
            return

        x, y, _z, _roll, _pitch, yaw_deg, latency_ms = botpose[:7]

        # Ignore obviously invalid estimates (outside field)
        if not (0.0 < x < 16.5) or not (0.0 < y < 8.1):
            return

        pose = wpimath.geometry.Pose2d(
            wpimath.geometry.Translation2d(x, y),
            wpimath.geometry.Rotation2d.fromDegrees(yaw_deg),
        )

        timestamp = wpilib.Timer.getFPGATimestamp() - (latency_ms / 1000.0)

        tag_count = self._tag_count_sub.get()
        std_devs = (
            VisionConstants.kMultiTagStdDevs
            if tag_count >= 2
            else VisionConstants.kSingleTagStdDevs
        )

        self._drive.add_vision_measurement(pose, timestamp, std_devs)

    def has_target(self) -> bool:
        return self._tv_sub.get() == 1

    def get_target_id(self) -> int:
        return int(self._tid_sub.get())

    def get_tx(self) -> float:
        """Horizontal angle offset to primary target (degrees)."""
        return self._table.getEntry("tx").getDouble(0.0)

    def get_ty(self) -> float:
        """Vertical angle offset to primary target (degrees)."""
        return self._table.getEntry("ty").getDouble(0.0)