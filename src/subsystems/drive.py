"""
Team 1504 - DriveSubsystem
MAXSwerve with NavX gyro + PathPlanner auto integration.

Key improvements over old code:
- Pose estimator (fuses vision + odometry) instead of plain odometry
- PathPlanner auto registration
- Field2d widget for SmartDashboard visualization
- Clean periodic telemetry logging
"""

import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
from wpimath.kinematics import ChassisSpeeds
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
            initialPose=wpimath.geometry.Pose2d(DriveConstants.k_start_x, DriveConstants.k_start_y,
                                    self.gyro.getRotation2d())
        )

        # ── Slew rate limiters ────────────────────────────────────
        self._mag_limiter = wpimath.filter.SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self._rot_limiter = wpimath.filter.SlewRateLimiter(DriveConstants.kRotationalSlewRate)
        self._current_translation_dir = 0.0
        self._current_translation_mag = 0.0
        self._current_rotation = 0.0
        self._prev_time = wpilib.Timer.getFPGATimestamp()

        # ── Dashboard visualization ───────────────────────────────
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    # ─────────────────────────────────────────────────────────────
    # PERIODIC
    # ─────────────────────────────────────────────────────────────
    def periodic(self) -> None:
        # Update pose estimator with current wheel positions & gyro
        self.pose_estimator.update(
            self.gyro.getRotation2d(),
            self._get_module_positions(),
        )
        pose = self.pose_estimator.getEstimatedPosition()
        self.field.setRobotPose(pose)

        # Telemetry
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
        Zero all slew rate limiter state.
        Call this at the start of teleop and whenever the drive command
        is first scheduled — prevents stale values from a previous mode
        causing the robot to drift or lurch on enable.
        """
        self._mag_limiter = wpimath.filter.SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self._rot_limiter = wpimath.filter.SlewRateLimiter(DriveConstants.kRotationalSlewRate)
        self._current_translation_dir = 0.0
        self._current_translation_mag = 0.0
        self._current_rotation = 0.0
        self._prev_time = wpilib.Timer.getFPGATimestamp()

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
    def get_chassisSpeeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.front_left.get_state,self.front_right.get_state,self.rear_left.get_state,self.rear_right.get_state)
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
        """Apply slew rate limiting to translation inputs."""
        import math as _math
        current_time = wpilib.Timer.getFPGATimestamp()
        elapsed = current_time - self._prev_time
        self._prev_time = current_time

        input_translation_dir = _math.atan2(y_speed, x_speed)
        input_translation_mag = _math.hypot(x_speed, y_speed)

        slew_rate = (
            DriveConstants.kDirectionSlewRate
            if self._current_translation_mag != 0.0
            else 500.0  # very fast when stopping
        )

        angle_diff = abs(input_translation_dir - self._current_translation_dir)
        if angle_diff < _math.pi * 0.45 or angle_diff > _math.pi * 1.55:
            self._current_translation_dir = _slerp_angle(
                self._current_translation_dir, input_translation_dir, slew_rate * elapsed
            )
            self._current_translation_mag = self._mag_limiter.calculate(input_translation_mag)
        elif angle_diff > _math.pi * 0.85 and angle_diff < _math.pi * 1.15:
            self._current_translation_dir = _slerp_angle(
                self._current_translation_dir, input_translation_dir, slew_rate * elapsed
            )
            self._current_translation_mag = self._mag_limiter.calculate(0.0)
        else:
            self._current_translation_mag = self._mag_limiter.calculate(0.0)

        return (
            self._current_translation_mag * _math.cos(self._current_translation_dir),
            self._current_translation_mag * _math.sin(self._current_translation_dir),
        )


def _slerp_angle(current: float, target: float, max_step: float) -> float:
    """Step toward target angle by at most max_step radians."""
    import math
    diff = (target - current + math.pi) % (2 * math.pi) - math.pi
    return current + max(min(diff, max_step), -max_step)