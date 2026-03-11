"""
Team 1504 - DriveSubsystem
MAXSwerve with NavX gyro + PathPlanner auto integration.
"""

import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
import wpimath.estimator
import wpimath.filter
import wpimath.units
import wpimath.system.plant
import navx
import commands2
from wpilib import Field2d, SmartDashboard

# PathPlanner
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants, ModuleConfig
from pathplannerlib.controller import PPHolonomicDriveController

from src.constants import DriveConstants, AutoConstants
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

        # ── Pose estimator ────────────────────────────────────────
        self.pose_estimator = wpimath.estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            self._get_module_positions(),
            wpimath.geometry.Pose2d(),
        )

        # ── PathPlanner AutoBuilder ───────────────────────────────
        # Try to load robot config from the PathPlanner GUI settings file.
        # If it doesn't exist yet (first run / no GUI file deployed), fall
        # back to building the config manually from constants.
        try:
            config = RobotConfig.fromGUISettings()
        except Exception:
            config = RobotConfig(
                massKG=AutoConstants.kRobotMassKg,
                MOI=AutoConstants.kRobotMOI,
                moduleConfig=ModuleConfig(
                    wheelRadiusMeters=DriveConstants.kWheelDiameterMeters / 2,
                    maxDriveVelocityMPS=DriveConstants.kMaxSpeedMps,
                    wheelCOF=AutoConstants.kWheelCOF,
                    # DCMotor.NEO() is the correct way to specify motor type
                    driveMotor=wpimath.system.plant.DCMotor.NEO(1),
                    driveCurrentLimit=DriveConstants.kDriveCurrentLimit,
                    numMotors=1,
                ),
                moduleOffsets=[
                    wpimath.geometry.Translation2d( DriveConstants.kWheelBase / 2,  DriveConstants.kTrackWidth / 2),
                    wpimath.geometry.Translation2d( DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
                    wpimath.geometry.Translation2d(-DriveConstants.kWheelBase / 2,  DriveConstants.kTrackWidth / 2),
                    wpimath.geometry.Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
                ],
            )

        AutoBuilder.configure(
            self.get_pose,
            self.reset_pose,
            self._get_chassis_speeds,
            # output must accept (ChassisSpeeds, DriveFeedforwards) —
            # we ignore feedforwards since we're not using them yet
            lambda speeds, feedforwards: self._drive_chassis_speeds(speeds),
            PPHolonomicDriveController(
                PIDConstants(AutoConstants.kPxController, 0.0, 0.0),
                PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0),
            ),
            config,
            self._should_flip_path,
            self,
        )

        # ── Per-axis slew rate limiters ───────────────────────────
        self._x_limiter   = wpimath.filter.SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self._y_limiter   = wpimath.filter.SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self._rot_limiter = wpimath.filter.SlewRateLimiter(DriveConstants.kRotationalSlewRate)

        # ── Dashboard ─────────────────────────────────────────────
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
    # PATHPLANNER HELPERS
    # ─────────────────────────────────────────────────────────────
    def _get_chassis_speeds(self) -> wpimath.kinematics.ChassisSpeeds:
        """Current robot-relative chassis speeds from measured module states."""
        return self.kinematics.toChassisSpeeds(
            (
                self.front_left.get_state(),
                self.front_right.get_state(),
                self.rear_left.get_state(),
                self.rear_right.get_state(),
            )
        )

    def _drive_chassis_speeds(self, speeds: wpimath.kinematics.ChassisSpeeds) -> None:
        """Command chassis speeds directly — called by PathPlanner during auto."""
        fl, fr, rl, rr = self.kinematics.toSwerveModuleStates(speeds)
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            (fl, fr, rl, rr), DriveConstants.kMaxSpeedMps
        )
        self.front_left.set_desired_state(fl)
        self.front_right.set_desired_state(fr)
        self.rear_left.set_desired_state(rl)
        self.rear_right.set_desired_state(rr)

    def _should_flip_path(self) -> bool:
        """Mirror paths to the red side of the field when on red alliance."""
        alliance = wpilib.DriverStation.getAlliance()
        return alliance == wpilib.DriverStation.Alliance.kRed

    # ─────────────────────────────────────────────────────────────
    # INTERNAL HELPERS
    # ─────────────────────────────────────────────────────────────
    def _get_module_positions(self):
        return (
            self.front_left.get_position(),
            self.front_right.get_position(),
            self.rear_left.get_position(),
            self.rear_right.get_position(),
        )

    def _apply_rate_limit(self, x_speed: float, y_speed: float) -> tuple[float, float]:
        return (
            self._x_limiter.calculate(x_speed),
            self._y_limiter.calculate(y_speed),
        )