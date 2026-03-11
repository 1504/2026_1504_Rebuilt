"""
Team 1504 - DriveSubsystem
MAXSwerve with NavX gyro + PathPlanner auto integration.

Key improvements over old code:
- Pose estimator (fuses vision + odometry) instead of plain odometry
- PathPlanner auto registration
- Field2d widget for SmartDashboard visualization
- Clean periodic telemetry logging
- Per-axis slew rate limiting (replaces brittle polar slew logic)
"""

import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
import wpimath.estimator
import wpimath.filter
import wpimath.units
import navx
import commands2
from wpilib import Field2d, SmartDashboard

from src.constants import DriveConstants
import src.swerve.swerve_module as swerve_module


class DriveSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # ── Swerve modules ────────────────────────────────────────
        self.front_left  = swerve_module.SwerveModule(
            DriveConstants.kFrontLeftDriveId,
            DriveConstants.kFrontLeftTurnId,
            DriveConstants.kFrontLeftChassisOffset,
            DriveConstants.kFrontLeftEncoderOffset,
        )
        self.front_right = swerve_module.SwerveModule(
            DriveConstants.kFrontRightDriveId,
            DriveConstants.kFrontRightTurnId,
            DriveConstants.kFrontRightChassisOffset,
            DriveConstants.kFrontRightEncoderOffset,
        )
        self.rear_left   = swerve_module.SwerveModule(
            DriveConstants.kRearLeftDriveId,
            DriveConstants.kRearLeftTurnId,
            DriveConstants.kRearLeftChassisOffset,
            DriveConstants.kRearLeftEncoderOffset,
        )
        self.rear_right  = swerve_module.SwerveModule(
            DriveConstants.kRearRightDriveId,
            DriveConstants.kRearRightTurnId,
            DriveConstants.kRearRightChassisOffset,
            DriveConstants.kRearRightEncoderOffset,
        )

        # ── Kinematics ────────────────────────────────────────────
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            wpimath.geometry.Translation2d( DriveConstants.kWheelBase / 2,  DriveConstants.kTrackWidth / 2),
            wpimath.geometry.Translation2d( DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
            wpimath.geometry.Translation2d(-DriveConstants.kWheelBase / 2,  DriveConstants.kTrackWidth / 2),
            wpimath.geometry.Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
        )

        # ── Gyro ──────────────────────────────────────────────────
        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

        # ── Pose estimator (replaces plain SwerveDrive4Odometry) ──
        # Fuses wheel odometry + AprilTag vision measurements.
        self.pose_estimator = wpimath.estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            self._get_module_positions(),
            wpimath.geometry.Pose2d(),
        )

        # ── Per-axis slew rate limiters ───────────────────────────
        # Slewing X and Y independently avoids all quadrant-change
        # dead zones that the old polar magnitude/direction approach had.
        self._x_limiter   = wpimath.filter.SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self._y_limiter   = wpimath.filter.SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self._rot_limiter = wpimath.filter.SlewRateLimiter(DriveConstants.kRotationalSlewRate)

        # ── Dashboard visualization ───────────────────────────────
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    # ─────────────────────────────────────────────────────────────
    # PERIODIC
    # ─────────────────────────────────────────────────────────────
    def periodic(self) -> None:
        self.pose_estimator.update(
            self.gyro.getRotation2d(),
            self._get_module_positions(),
        )
        pose = self.pose_estimator.getEstimatedPosition()
        self.field.setRobotPose(pose)

        SmartDashboard.putNumber("Drive/HeadingDeg", self.gyro.getAngle())
        SmartDashboard.putNumber("Drive/PoseX", pose.X())
        SmartDashboard.putNumber("Drive/PoseY", pose.Y())

    # ─────────────────────────────────────────────────────────────
    # DRIVING
    # ─────────────────────────────────────────────────────────────
    def drive(
        self,
        x_speed: float,
        y_speed: float,
        rot: float,
        field_relative: bool,
        rate_limit: bool = True,
    ) -> None:
        """
        Drive the robot.

        :param x_speed:       Forward/back (-1 to 1, fraction of max)
        :param y_speed:       Left/right (-1 to 1, fraction of max)
        :param rot:           Rotation (-1 to 1, fraction of max)
        :param field_relative: True = field-centric driving
        :param rate_limit:    True = apply slew rate limiting
        """
        if rate_limit:
            x_speed_commanded, y_speed_commanded = self._apply_rate_limit(x_speed, y_speed)
            rot = self._rot_limiter.calculate(rot)
        else:
            x_speed_commanded = x_speed
            y_speed_commanded = y_speed

        x_mps   = x_speed_commanded * DriveConstants.kMaxSpeedMps
        y_mps   = y_speed_commanded * DriveConstants.kMaxSpeedMps
        rot_rps = rot * DriveConstants.kMaxAngularSpeedRps

        chassis_speeds = (
            wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                x_mps, y_mps, rot_rps, self.gyro.getRotation2d()
            )
            if field_relative
            else wpimath.kinematics.ChassisSpeeds(x_mps, y_mps, rot_rps)
        )

        fl, fr, rl, rr = self.kinematics.toSwerveModuleStates(chassis_speeds)
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            (fl, fr, rl, rr), DriveConstants.kMaxSpeedMps
        )
        self.front_left.set_desired_state(fl)
        self.front_right.set_desired_state(fr)
        self.rear_left.set_desired_state(rl)
        self.rear_right.set_desired_state(rr)

    def set_x(self) -> None:
        """Lock wheels in X pattern to resist being pushed."""
        self.front_left.set_desired_state(
            wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(math.pi / 4))
        )
        self.front_right.set_desired_state(
            wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(-math.pi / 4))
        )
        self.rear_left.set_desired_state(
            wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(-math.pi / 4))
        )
        self.rear_right.set_desired_state(
            wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(math.pi / 4))
        )

    def stop(self) -> None:
        self.drive(0.0, 0.0, 0.0, False, False)

    def reset_slew(self) -> None:
        """
        Reinitialize all slew rate limiters to a clean zero state.
        Call at the start of teleop and whenever the drive command is
        first scheduled — prevents stale values from auto/disabled
        causing a jerk or drift on enable.
        """
        self._x_limiter   = wpimath.filter.SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self._y_limiter   = wpimath.filter.SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self._rot_limiter = wpimath.filter.SlewRateLimiter(DriveConstants.kRotationalSlewRate)

    # ─────────────────────────────────────────────────────────────
    # POSE / ODOMETRY
    # ─────────────────────────────────────────────────────────────
    def get_pose(self) -> wpimath.geometry.Pose2d:
        return self.pose_estimator.getEstimatedPosition()

    def reset_pose(self, pose: wpimath.geometry.Pose2d) -> None:
        self.pose_estimator.resetPosition(
            self.gyro.getRotation2d(),
            self._get_module_positions(),
            pose,
        )

    def add_vision_measurement(
        self,
        pose: wpimath.geometry.Pose2d,
        timestamp: float,
        std_devs: tuple[float, float, float] | None = None,
    ) -> None:
        """Feed a vision pose estimate into the pose estimator."""
        if std_devs:
            self.pose_estimator.addVisionMeasurement(
                pose, timestamp,
                (std_devs[0], std_devs[1], std_devs[2])
            )
        else:
            self.pose_estimator.addVisionMeasurement(pose, timestamp)

    def zero_heading(self) -> None:
        self.gyro.reset()

    def get_heading_degrees(self) -> float:
        return self.gyro.getAngle()

    def get_rotation2d(self) -> wpimath.geometry.Rotation2d:
        return self.gyro.getRotation2d()

    # ─────────────────────────────────────────────────────────────
    # HELPERS
    # ─────────────────────────────────────────────────────────────
    def _get_module_positions(self):
        return (
            self.front_left.get_position(),
            self.front_right.get_position(),
            self.rear_left.get_position(),
            self.rear_right.get_position(),
        )

    def _apply_rate_limit(self, x_speed: float, y_speed: float) -> tuple[float, float]:
        """
        Per-axis slew rate limiting.
        Slewing X and Y independently means quadrant changes are handled
        naturally — each axis just chases its target value. No angle
        binning, no branches that zero out your input mid-motion.
        """
        return (
            self._x_limiter.calculate(x_speed),
            self._y_limiter.calculate(y_speed),
        )