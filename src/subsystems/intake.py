"""
Team 1504 - IntakeSubsystem
Two SparkFlex + NEO Vortex motors on the same axle.
Motor 2 follows Motor 1 in the opposite direction (mirrored mounting).
Both motors use coast idle mode so the intake spins freely when stopped.
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

        # ── Motor 1 (leader) ──────────────────────────────────────
        self._motor = SparkFlex(IntakeConstants.kMotorId, SparkFlex.MotorType.kBrushless)

        cfg = SparkFlexConfig()
        cfg.smartCurrentLimit(IntakeConstants.kCurrentLimit)
        cfg.setIdleMode(SparkFlexConfig.IdleMode.kCoast)
        cfg.inverted(False)  # Flip if intake runs backwards

        self._motor.configure(
            cfg,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # ── Motor 2 (follower, opposite direction) ────────────────
        self._motor2 = SparkFlex(IntakeConstants.kMotorId2, SparkFlex.MotorType.kBrushless)

        cfg2 = SparkFlexConfig()
        cfg2.smartCurrentLimit(IntakeConstants.kCurrentLimit)
        cfg2.setIdleMode(SparkFlexConfig.IdleMode.kCoast)
        # Inverted relative to leader because motors face opposite directions
        # on the same axle. Flip this bool if the intake runs backwards.
        cfg2.inverted(True)

        self._motor2.configure(
            cfg2,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self._encoder = self._motor.getEncoder()

    def periodic(self) -> None:
        SmartDashboard.putNumber("Intake/Current1", self._motor.getOutputCurrent())
        SmartDashboard.putNumber("Intake/Current2", self._motor2.getOutputCurrent())
        SmartDashboard.putNumber("Intake/Velocity", self._encoder.getVelocity())

    def intake_run(self, speed: float | None = None) -> None:
        spd = speed if speed is not None else IntakeConstants.kIntakeSpeed
        self._motor.set(spd)
        self._motor2.set(spd)

    def reverse(self, speed: float | None = None) -> None:
        spd = speed if speed is not None else IntakeConstants.kReverseSpeed
        self._motor.set(spd)
        self._motor2.set(spd)

    def stop(self) -> None:
        self._motor.set(0.0)
        self._motor2.set(0.0)