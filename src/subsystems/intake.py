"""
Team 1504 - IntakeSubsystem
SparkFlex + NEO Vortex intake roller.
"""

import commands2
import rev
import wpilib
from wpilib import SmartDashboard
from rev import SparkFlex, SparkFlexConfig

from src.constants import IntakeConstants


class IntakeSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # NEO Vortex requires SparkFlex, NOT SparkMax
        self._motor = SparkFlex(IntakeConstants.kMotorId, SparkFlex.MotorType.kBrushless)

        cfg = SparkFlexConfig()
        cfg.smartCurrentLimit(IntakeConstants.kCurrentLimit)
        cfg.inverted(False)  # Flip to True if intake runs backwards

        self._motor.configure(
            cfg,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self._encoder = self._motor.getEncoder()

    def periodic(self) -> None:
        SmartDashboard.putNumber("Intake/Current", self._motor.getOutputCurrent())
        SmartDashboard.putNumber("Intake/Velocity", self._encoder.getVelocity())

    def intake_run(self, speed: float | None = None) -> None:
        spd = speed if speed is not None else IntakeConstants.kIntakeSpeed
        self._motor.set(spd)

    def reverse(self, speed: float | None = None) -> None:
        spd = speed if speed is not None else IntakeConstants.kReverseSpeed
        self._motor.set(spd)

    def stop(self) -> None:
        self._motor.set(0.0)