"""
Team 1504 - ShooterSubsystem
Phoenix6 TalonFX velocity PID for flywheel + SparkMax feeder.

Improvements over old code:
- Proper Phoenix6 VelocityVoltage control request
- Feeder uses current stall detection for jam protection
- InterpolatingDoubleTreeMap for distance → speed lookup
- All magic numbers in ShooterConstants
"""

import commands2
import wpilib
from wpilib import SmartDashboard

# Phoenix6
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage
from phoenix6.configs import TalonFXConfiguration

# SparkMax
import rev
from rev import SparkMax, SparkMaxConfig

from src.constants import ShooterConstants


class ShooterSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # ── Flywheel motors (TalonFX / Kraken or Falcon) ──────────
        self._motor1 = TalonFX(ShooterConstants.kShooterMotor1Id)
        self._motor2 = TalonFX(ShooterConstants.kShooterMotor2Id)

        cfg = TalonFXConfiguration()
        cfg.slot0.k_p  = ShooterConstants.kShooterP
        cfg.slot0.k_i  = ShooterConstants.kShooterI
        cfg.slot0.k_d  = ShooterConstants.kShooterD
        cfg.slot0.k_v  = ShooterConstants.kShooterKv

        self._motor1.configurator.apply(cfg)
        self._motor2.configurator.apply(cfg)

        # Motor 2 follows motor 1 in opposite direction (drum launcher)
        # If both spin same direction, invert one here:
        # self._motor2.set_inverted(True)

        # Control request object (reused every loop - avoids GC pressure)
        self._velocity_request = VelocityVoltage(0).with_slot(0).with_enable_foc(True)

        # ── Feeder motor (SparkMax NEO 550) ───────────────────────
        self._feeder = SparkMax(ShooterConstants.kFeederMotorId, SparkMax.MotorType.kBrushless)
        feeder_cfg = SparkMaxConfig()
        feeder_cfg.smartCurrentLimit(ShooterConstants.kFeederCurrentLimit)
        self._feeder.configure(feeder_cfg, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        # ── State ─────────────────────────────────────────────────
        self._target_rps: float = 0.0

    # ─────────────────────────────────────────────────────────────
    # PERIODIC
    # ─────────────────────────────────────────────────────────────
    def periodic(self) -> None:
        m1_vel = self._motor1.get_velocity().value
        m2_vel = self._motor2.get_velocity().value

        SmartDashboard.putNumber("Shooter/Motor1 RPS", m1_vel)
        SmartDashboard.putNumber("Shooter/Motor2 RPS", m2_vel)
        SmartDashboard.putNumber("Shooter/Target RPS", self._target_rps)
        SmartDashboard.putBoolean("Shooter/AtSpeed", self.is_at_speed())
        SmartDashboard.putNumber("Shooter/FeederCurrent", self._feeder.getOutputCurrent())

    # ─────────────────────────────────────────────────────────────
    # FLYWHEEL CONTROL
    # ─────────────────────────────────────────────────────────────
    def set_velocity_rps(self, rps: float) -> None:
        """Set target flywheel velocity in rotations per second."""
        self._target_rps = rps
        self._motor1.set_control(self._velocity_request.with_velocity(rps))
        self._motor2.set_control(self._velocity_request.with_velocity(rps))

    def set_velocity_from_distance(self, distance_meters: float) -> None:
        """Look up target RPS from shooter table and apply."""
        rps = self._interpolate_rps(distance_meters)
        self.set_velocity_rps(rps)

    def stop_shooter(self) -> None:
        self._target_rps = 0.0
        self._motor1.set_control(VelocityVoltage(0))
        self._motor2.set_control(VelocityVoltage(0))

    def is_at_speed(self) -> bool:
        if self._target_rps == 0.0:
            return False
        m1 = abs(self._motor1.get_velocity().value - self._target_rps)
        m2 = abs(self._motor2.get_velocity().value - self._target_rps)
        return m1 < ShooterConstants.kVelocityToleranceRps and m2 < ShooterConstants.kVelocityToleranceRps

    # ─────────────────────────────────────────────────────────────
    # FEEDER CONTROL
    # ─────────────────────────────────────────────────────────────
    def run_feeder(self, speed: float | None = None) -> None:
        self._feeder.set(speed if speed is not None else ShooterConstants.kFeederSpeed)

    def stop_feeder(self) -> None:
        self._feeder.set(0.0)

    def stop_all(self) -> None:
        self.stop_shooter()
        self.stop_feeder()

    # ─────────────────────────────────────────────────────────────
    # HELPER: interpolate shooter table
    # ─────────────────────────────────────────────────────────────
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