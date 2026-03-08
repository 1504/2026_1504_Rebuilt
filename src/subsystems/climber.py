"""
Team 1504 - ClimberSubsystem
Update motor type/ID in ClimberConstants to match hardware.
"""

import commands2
import wpilib
from wpilib import SmartDashboard

# Using SparkMax here — swap for TalonFX if using Kraken
import rev
from rev import SparkMax, SparkMaxConfig

from src.constants import ClimberConstants


class ClimberSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._motor = SparkMax(ClimberConstants.kClimberMotorId, SparkMax.MotorType.kBrushless)
        cfg = SparkMaxConfig()
        cfg.smartCurrentLimit(ClimberConstants.kCurrentLimit)
        cfg.setIdleMode(SparkMaxConfig.IdleMode.kBrake)  # Hold position when stopped
        self._motor.configure(cfg, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self._encoder = self._motor.getEncoder()

    def periodic(self) -> None:
        SmartDashboard.putNumber("Climber/Position", self._encoder.getPosition())
        SmartDashboard.putNumber("Climber/Current", self._motor.getOutputCurrent())

    def climb_up(self) -> None:
        self._motor.set(ClimberConstants.kClimbSpeed)

    def climb_down(self) -> None:
        self._motor.set(ClimberConstants.kDescendSpeed)

    def stop(self) -> None:
        self._motor.set(0.0)

    def get_position(self) -> float:
        return self._encoder.getPosition()
