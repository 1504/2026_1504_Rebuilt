"""
Team 1504 - ShooterSubsystem (IO-layer version)
All hardware access goes through ShooterIO.
"""

import commands2
from wpilib import SmartDashboard

from src.constants import ShooterConstants
from src.io.shooter_io import ShooterIO, ShooterInputs


class ShooterSubsystem(commands2.Subsystem):
    def __init__(self, io: ShooterIO) -> None:
        super().__init__()
        self._io = io
        self._inputs = ShooterInputs()
        self._target_rps: float = 0.0

    def periodic(self) -> None:
        self._io.update_inputs(self._inputs)

        SmartDashboard.putNumber("Shooter/Motor1 RPS",   self._inputs.motor1_rps)
        SmartDashboard.putNumber("Shooter/Motor2 RPS",   self._inputs.motor2_rps)
        SmartDashboard.putNumber("Shooter/Target RPS",   self._target_rps)
        SmartDashboard.putBoolean("Shooter/AtSpeed",     self.is_at_speed())
        SmartDashboard.putNumber("Shooter/FeederCurrent", self._inputs.feeder_current)

    # ── Flywheel ──────────────────────────────────────────────────────────────
    def set_velocity_rps(self, rps: float) -> None:
        self._target_rps = rps
        self._io.set_velocity_rps(rps)

    def set_velocity_from_distance(self, distance_meters: float) -> None:
        self.set_velocity_rps(self._interpolate_rps(distance_meters))

    def stop_shooter(self) -> None:
        self._target_rps = 0.0
        self._io.set_velocity_rps(0.0)

    def is_at_speed(self) -> bool:
        if self._target_rps == 0.0:
            return False
        tol = ShooterConstants.kVelocityToleranceRps
        return (
            abs(self._inputs.motor1_rps - self._target_rps) < tol
            and abs(self._inputs.motor2_rps - self._target_rps) < tol
        )

    # ── Feeder ────────────────────────────────────────────────────────────────
    def run_feeder(self, speed: float | None = None) -> None:
        self._io.set_feeder(speed if speed is not None else ShooterConstants.kFeederSpeed)

    def stop_feeder(self) -> None:
        self._io.set_feeder(0.0)

    def stop_all(self) -> None:
        self._io.stop()
        self._target_rps = 0.0

    # ── Interpolation ─────────────────────────────────────────────────────────
    def _interpolate_rps(self, distance_meters: float) -> float:
        table = ShooterConstants.kShooterTable
        if not table:
            return 40.0
        if distance_meters <= table[0][0]:
            return table[0][1]
        if distance_meters >= table[-1][0]:
            return table[-1][1]
        for i in range(len(table) - 1):
            d0, v0 = table[i]
            d1, v1 = table[i + 1]
            if d0 <= distance_meters <= d1:
                t = (distance_meters - d0) / (d1 - d0)
                return v0 + t * (v1 - v0)
        return 40.0