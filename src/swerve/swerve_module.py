"""
Team 1504 - SwerveModule
MAXSwerve with REV SparkMax. Cleaned up from original code.
"""

import math
import wpimath.kinematics
import wpimath.geometry
import rev
from rev import SparkMax, SparkMaxConfig, SparkBase

from src.constants import DriveConstants


class SwerveModule:
    def __init__(
        self,
        drive_can_id: int,
        turn_can_id: int,
        chassis_angular_offset: float,
        absolute_encoder_offset: float,
    ) -> None:

        self._drive = SparkMax(drive_can_id, SparkMax.MotorType.kBrushless)
        self._turn  = SparkMax(turn_can_id,  SparkMax.MotorType.kBrushless)

        drive_cfg = SparkMaxConfig()
        turn_cfg  = SparkMaxConfig()

        # Drive encoder: convert to meters and m/s
        drive_cfg.encoder.positionConversionFactor(DriveConstants.kDriveEncoderPositionFactor)
        drive_cfg.encoder.velocityConversionFactor(DriveConstants.kDriveEncoderVelocityFactor)

        # Turn encoder: convert to radians
        turn_cfg.absoluteEncoder.positionConversionFactor(DriveConstants.kTurnEncoderPositionFactor)
        turn_cfg.absoluteEncoder.velocityConversionFactor(DriveConstants.kTurnEncoderVelocityFactor)
        turn_cfg.absoluteEncoder.inverted(DriveConstants.kTurnEncoderInverted)
        turn_cfg.absoluteEncoder.zeroOffset(absolute_encoder_offset)

        # Wrap turn PID (0 to 2π)
        turn_cfg.closedLoop.positionWrappingEnabled(True)
        turn_cfg.closedLoop.positionWrappingInputRange(0, DriveConstants.kTurnEncoderPositionFactor)
        turn_cfg.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kAbsoluteEncoder)

        # Drive PID
        drive_cfg.closedLoop.P(DriveConstants.kDriveP)
        drive_cfg.closedLoop.I(DriveConstants.kDriveI)
        drive_cfg.closedLoop.D(DriveConstants.kDriveD)
        drive_cfg.closedLoop.velocityFF(DriveConstants.kDriveFF)
        drive_cfg.closedLoop.outputRange(DriveConstants.kDriveMinOutput, DriveConstants.kDriveMaxOutput)

        # Turn PID
        turn_cfg.closedLoop.P(DriveConstants.kTurnP)
        turn_cfg.closedLoop.I(DriveConstants.kTurnI)
        turn_cfg.closedLoop.D(DriveConstants.kTurnD)
        turn_cfg.closedLoop.velocityFF(DriveConstants.kTurnFF)
        turn_cfg.closedLoop.outputRange(DriveConstants.kTurnMinOutput, DriveConstants.kTurnMaxOutput)

        # Motor settings
        drive_cfg.setIdleMode(DriveConstants.kDriveIdleMode)
        turn_cfg.setIdleMode(DriveConstants.kTurnIdleMode)
        drive_cfg.smartCurrentLimit(DriveConstants.kDriveCurrentLimit)
        turn_cfg.smartCurrentLimit(DriveConstants.kTurnCurrentLimit)

        self._drive.configure(drive_cfg, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self._turn.configure(turn_cfg,   rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self._drive_encoder   = self._drive.getEncoder()
        self._turn_encoder    = self._turn.getAbsoluteEncoder()
        self._drive_pid       = self._drive.getClosedLoopController()
        self._turn_pid        = self._turn.getClosedLoopController()

        self._chassis_offset  = chassis_angular_offset
        self._desired_state   = wpimath.kinematics.SwerveModuleState(
            0.0, wpimath.geometry.Rotation2d(self._turn_encoder.getPosition())
        )
        self._drive_encoder.setPosition(0)

    def get_state(self) -> wpimath.kinematics.SwerveModuleState:
        return wpimath.kinematics.SwerveModuleState(
            self._drive_encoder.getVelocity(),
            wpimath.geometry.Rotation2d(self._turn_encoder.getPosition() - self._chassis_offset),
        )

    def get_position(self) -> wpimath.kinematics.SwerveModulePosition:
        return wpimath.kinematics.SwerveModulePosition(
            self._drive_encoder.getPosition(),
            wpimath.geometry.Rotation2d(self._turn_encoder.getPosition() - self._chassis_offset),
        )

    def set_desired_state(self, desired_state: wpimath.kinematics.SwerveModuleState) -> None:
        # .angle is typed Optional in RobotPy stubs — guard before arithmetic
        input_angle = desired_state.angle if desired_state.angle is not None else wpimath.geometry.Rotation2d()
        corrected_angle = input_angle + wpimath.geometry.Rotation2d(self._chassis_offset)
        corrected = wpimath.kinematics.SwerveModuleState(desired_state.speed, corrected_angle)

        # optimize() is typed Optional[SwerveModuleState] in RobotPy stubs — fall back to corrected
        optimized: wpimath.kinematics.SwerveModuleState = (
            wpimath.kinematics.SwerveModuleState.optimize(
                corrected, wpimath.geometry.Rotation2d(self._turn_encoder.getPosition())
            ) or corrected
        )

        turn_rads: float = optimized.angle.radians() if optimized.angle is not None else corrected_angle.radians()
        drive_speed: float = optimized.speed if optimized.speed is not None else 0.0

        self._drive_pid.setReference(drive_speed, SparkBase.ControlType.kVelocity)
        self._turn_pid.setReference(turn_rads, SparkBase.ControlType.kPosition)
        self._desired_state = optimized

    def reset_encoders(self) -> None:
        self._drive_encoder.setPosition(0)