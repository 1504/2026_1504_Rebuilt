"""
Team 1504 - ShooterSubsystem
Phoenix6 TalonFX velocity PID for flywheel + SparkMax feeder.
"""

import commands2
import wpilib
from wpilib import SmartDashboard

# Phoenix6
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage, NeutralOut
from phoenix6.configs import TalonFXConfiguration

# SparkMax
import rev
from rev import SparkMax, SparkMaxConfig

from src.constants import ShooterConstants
from phoenix6.signals import InvertedValue


class ShooterSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # ── Flywheel motors (TalonFX / Kraken or Falcon) ──────────
        self._motor1 = TalonFX(ShooterConstants.kShooterMotor1Id)
        self._motor2 = TalonFX(ShooterConstants.kShooterMotor2Id)
        
        # Set globals to read this once per loop
        self._motor1CurrentVelocity = 0.0
        self._motor2CurrentVelocity = 0.0

        cfg1 = TalonFXConfiguration()
        cfg1.slot0.k_p = ShooterConstants.kShooterP
        cfg1.slot0.k_i = ShooterConstants.kShooterI
        cfg1.slot0.k_d = ShooterConstants.kShooterD
        cfg1.slot0.k_v = ShooterConstants.kShooterKv
        cfg1.current_limits.supply_current_limit_enable  = True
        cfg1.current_limits.supply_current_limit         = ShooterConstants.kFlywheelCurrentLimit
        # Sustain limit: after 0.5 s of high draw, drop to 40 A to protect the
        # motor and avoid browning out the rail while the flywheel recovers.
        cfg1.current_limits.supply_current_lower_limit   = 40
        cfg1.current_limits.supply_current_lower_time    = 0.5
        cfg1.current_limits.stator_current_limit_enable  = True
        cfg1.current_limits.stator_current_limit         = ShooterConstants.kFlywheelStatorCurrentLimit
        self._motor1.configurator.apply(cfg1)

        cfg2 = TalonFXConfiguration()
        cfg2.slot0.k_p = ShooterConstants.kShooterP
        cfg2.slot0.k_i = ShooterConstants.kShooterI
        cfg2.slot0.k_d = ShooterConstants.kShooterD
        cfg2.slot0.k_v = ShooterConstants.kShooterKv
        cfg2.current_limits.supply_current_limit_enable  = True
        cfg2.current_limits.supply_current_limit         = ShooterConstants.kFlywheelCurrentLimit
        cfg2.current_limits.supply_current_lower_limit   = 40
        cfg2.current_limits.supply_current_lower_time    = 0.5
        cfg2.current_limits.stator_current_limit_enable  = True
        cfg2.current_limits.stator_current_limit         = ShooterConstants.kFlywheelStatorCurrentLimit
        cfg2.motor_output.inverted                        = InvertedValue.CLOCKWISE_POSITIVE
        self._motor2.configurator.apply(cfg2)

        # Reused every loop to avoid GC pressure
        self._velocity_request = VelocityVoltage(0).with_slot(0).with_enable_foc(True)
        # NeutralOut releases the motor so it coasts freely when stopped.
        # VelocityVoltage(0) would keep the PID active and resist being turned
        # by hand (e.g. during inspection), so we use NeutralOut instead.
        self._neutral_request  = NeutralOut()

        # ── Feeder motor (SparkMax NEO 550) ───────────────────────
        # NEO 550 safe continuous limit is ~20 A. The original 60 A limit was
        # causing large current spikes on ball intake that browned out the
        # flywheel motors sharing the same power rail.
        self._feeder   = SparkMax(ShooterConstants.kFeederMotorId,   SparkMax.MotorType.kBrushless)

        feeder_cfg = SparkMaxConfig()
        feeder_cfg.smartCurrentLimit(ShooterConstants.kFeederCurrentLimit)
        feeder_cfg.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        self._feeder.configure(
            feeder_cfg,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # ── State ─────────────────────────────────────────────────
        self._target_rps: float = 0.0

    # ─────────────────────────────────────────────────────────────
    # PERIODIC
    # ─────────────────────────────────────────────────────────────
    def periodic(self) -> None:
        self._motor1CurrentVelocity = self._motor1.get_velocity().value
        self._motor2CurrentVelocity = self._motor2.get_velocity().value
        
        # Read in values set from smart dashboard
        self._targetVelocity = SmartDashboard.getNumber("Shooter/TargetVelocity", ShooterConstants.kDefaultShooterRps)
        self._feederPercent = SmartDashboard.getNumber("Shooter/TargetFeeder%", ShooterConstants.kFeederSpeed)
        
        SmartDashboard.putNumber("Shooter/Velocities/Motor1 RPS",      self._motor1CurrentVelocity)
        SmartDashboard.putNumber("Shooter/Velocities/Motor2 RPS",      self._motor2CurrentVelocity)
            
        SmartDashboard.putNumber("Shooter/Commanded RPS",   self._target_rps)
        SmartDashboard.putBoolean("Shooter/AtSpeed",        self.is_at_speed())
        
        SmartDashboard.putNumber("Shooter/Currents/FeederCurrent",   self._feeder.getOutputCurrent())
        SmartDashboard.putNumber("Shooter/Currents/Motor1Current",   self._motor1.get_supply_current().value)
        SmartDashboard.putNumber("Shooter/Currents/Motor2Current",   self._motor2.get_supply_current().value)

    # ─────────────────────────────────────────────────────────────
    # FLYWHEEL CONTROL
    # ─────────────────────────────────────────────────────────────
    def set_velocity_rps(self, rps: float) -> None:
        self._target_rps = rps
        self._motor1.set_control(self._velocity_request.with_velocity(rps))
        self._motor2.set_control(self._velocity_request.with_velocity(rps))

    def set_velocity_from_distance(self, distance_meters: float) -> None:
        rps = self._interpolate_rps(distance_meters)
        self.set_velocity_rps(rps)

    def stop_shooter(self) -> None:
        self._target_rps = 0.0
        self._motor1.set_control(self._neutral_request)
        self._motor2.set_control(self._neutral_request)

    def is_at_speed(self) -> bool:
        if self._target_rps == 0.0:
            return False
        m1_err = abs(self._motor1CurrentVelocity - self._target_rps)
        m2_err = abs(self._motor2CurrentVelocity - self._target_rps)
        return (
            m1_err < ShooterConstants.kVelocityToleranceRps
            and m2_err < ShooterConstants.kVelocityToleranceRps
        )

    # Increase shooter RPS
    def increase_target_velocity(self) -> None:
        self._targetVelocity = min(self._targetVelocity + ShooterConstants.kShooterRpsStep, ShooterConstants.kShooterMaxRps)
        SmartDashboard.putNumber("Shooter/TargetVelocity", self._targetVelocity)
    
    # Decreases shooter RPS
    def decrease_target_velocity(self) -> None:
        self._targetVelocity = max(self._targetVelocity - ShooterConstants.kShooterRpsStep, ShooterConstants.kShooterMinRps)
        SmartDashboard.putNumber("Shooter/TargetVelocity", self._targetVelocity)
        
    # Reset shooter RPS to default
    def reset_target_velocity(self) -> None:
        self._targetVelocity = ShooterConstants.kDefaultShooterRps
        SmartDashboard.putNumber("Shooter/TargetVelocity", self._targetVelocity)

    # ─────────────────────────────────────────────────────────────
    # FEEDER CONTROL
    # ─────────────────────────────────────────────────────────────
    def run_feeder(self, speed: float | None = None) -> None:
        speed = speed if speed is not None else ShooterConstants.kFeederSpeed
        SmartDashboard.putNumber("Shooter/Commanded Feeder Speed", speed)
        self._feeder.set(speed)

    def stop_feeder(self) -> None:
        self._feeder.set(0.0)

    def stop_all(self) -> None:
        self.stop_shooter()
        self.stop_feeder()
        
    def shoot_at_sd_velocity(self) -> None:
        self.set_velocity_rps(self.get_sd_rps())

    # return the value that we're reading from SmartDashboard
    def get_sd_rps(self) -> float:
        return self._targetVelocity
    
    def get_sd_feeder(self) -> float:
        return self._feederPercent

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