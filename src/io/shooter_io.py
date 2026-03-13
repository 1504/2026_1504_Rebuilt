"""
Team 1504 - ShooterIO
IO layer for the shooter subsystem.

Real  → TalonFX velocity PID + SparkMax feeder (what runs on the actual robot)
Sim   → WPILib FlywheelSim with physics model (what runs in simulation)

Usage in robot_container.py:
    from src.io.shooter_io import ShooterIOReal, ShooterIOSim
    import wpilib
    io = ShooterIOReal() if wpilib.RobotBase.isReal() else ShooterIOSim()
    self.shooter = ShooterSubsystem(io)
"""

from __future__ import annotations
import math
from dataclasses import dataclass, field
import wpilib
import wpimath.system.plant
import wpimath.controller
import wpimath.units
from wpilib.simulation import FlywheelSim
from wpilib import simulation
from wpimath.system.plant import LinearSystemId, DCMotor


# ─────────────────────────────────────────────────────────────────────────────
# INPUTS  (logged data from the hardware layer each loop)
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class ShooterInputs:
    motor1_rps: float = 0.0
    motor2_rps: float = 0.0
    motor1_voltage: float = 0.0
    motor2_voltage: float = 0.0
    feeder_speed: float = 0.0
    feeder_current: float = 0.0
    connected: bool = True


# ─────────────────────────────────────────────────────────────────────────────
# INTERFACE
# ─────────────────────────────────────────────────────────────────────────────
class ShooterIO:
    def update_inputs(self, inputs: ShooterInputs) -> None:
        """Refresh inputs from hardware/sim. Called every loop."""
        pass

    def set_velocity_rps(self, rps: float) -> None:
        pass

    def set_feeder(self, speed: float) -> None:
        pass

    def stop(self) -> None:
        self.set_velocity_rps(0.0)
        self.set_feeder(0.0)


# ─────────────────────────────────────────────────────────────────────────────
# REAL HARDWARE
# ─────────────────────────────────────────────────────────────────────────────
class ShooterIOReal(ShooterIO):
    def __init__(self) -> None:
        from phoenix6.hardware import TalonFX
        from phoenix6.controls import VelocityVoltage
        from phoenix6.configs import TalonFXConfiguration
        import rev
        from rev import SparkMax, SparkMaxConfig
        from src.constants import ShooterConstants

        self._motor1 = TalonFX(ShooterConstants.kShooterMotor1Id)
        self._motor2 = TalonFX(ShooterConstants.kShooterMotor2Id)

        cfg = TalonFXConfiguration()
        cfg.slot0.k_p = ShooterConstants.kShooterP
        cfg.slot0.k_i = ShooterConstants.kShooterI
        cfg.slot0.k_d = ShooterConstants.kShooterD
        cfg.slot0.k_v = ShooterConstants.kShooterKv
        self._motor1.configurator.apply(cfg)
        self._motor2.configurator.apply(cfg)

        self._vel_req = VelocityVoltage(0).with_slot(0).with_enable_foc(True)

        self._feeder = SparkMax(ShooterConstants.kFeederMotorId, SparkMax.MotorType.kBrushless)
        feeder_cfg = SparkMaxConfig()
        feeder_cfg.smartCurrentLimit(ShooterConstants.kFeederCurrentLimit)
        self._feeder.configure(
            feeder_cfg,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    def update_inputs(self, inputs: ShooterInputs) -> None:
        inputs.motor1_rps = self._motor1.get_velocity().value
        inputs.motor2_rps = self._motor2.get_velocity().value
        inputs.motor1_voltage = self._motor1.get_motor_voltage().value
        inputs.motor2_voltage = self._motor2.get_motor_voltage().value
        inputs.feeder_current = self._feeder.getOutputCurrent()
        inputs.connected = True

    def set_velocity_rps(self, rps: float) -> None:
        from phoenix6.controls import VelocityVoltage
        self._motor1.set_control(self._vel_req.with_velocity(rps))
        self._motor2.set_control(self._vel_req.with_velocity(rps))

    def set_feeder(self, speed: float) -> None:
        self._feeder.set(speed)


# ─────────────────────────────────────────────────────────────────────────────
# SIMULATION
# ─────────────────────────────────────────────────────────────────────────────
class ShooterIOSim(ShooterIO):
    """
    Two Kraken X60 flywheels modeled as a single combined flywheel,
    plus a trivial feeder pass-through.

    Physics: J = 0.003 kg·m² (two ~150g wheels, r≈0.05 m each)
    Uses a simple PID + feedforward to mimic the TalonFX closed-loop.
    """

    # Match ShooterConstants defaults; import at runtime to avoid circular deps
    _KP = 0.5
    _KV = 0.12   # V·s/rot  (feedforward gain)
    _DT = 0.02   # seconds per loop

    def __init__(self) -> None:
        # Two NEO-equivalent motors, J = 0.003 kg·m²
        motor = wpimath.system.plant.DCMotor.krakenX60(2)
        self._flywheel_sim = FlywheelSim(
        plant=LinearSystemId.flywheelSystem(motor, 0.003, 1.0),  # motor, moi, gearing
        gearbox=motor,
        )
        self._pid = wpimath.controller.PIDController(self._KP, 0.0, 0.0)
        self._target_rps: float = 0.0
        self._feeder_speed: float = 0.0
        self._voltage_applied: float = 0.0

    def update_inputs(self, inputs: ShooterInputs) -> None:
        # Run the physics sim one timestep
        self._flywheel_sim.setInputVoltage(self._voltage_applied)
        self._flywheel_sim.update(self._DT)

        # FlywheelSim gives angular velocity in rad/s; convert to RPS
        rps = self._flywheel_sim.getAngularVelocity() / (2 * math.pi)

        inputs.motor1_rps = rps
        inputs.motor2_rps = rps
        inputs.motor1_voltage = self._voltage_applied
        inputs.motor2_voltage = self._voltage_applied
        inputs.feeder_speed = self._feeder_speed
        inputs.feeder_current = abs(self._feeder_speed) * 15.0  # rough estimate
        inputs.connected = True

    def set_velocity_rps(self, rps: float) -> None:
        self._target_rps = rps
        current_rps = self._flywheel_sim.getAngularVelocity() / (2 * math.pi)
        ff = self._KV * rps * 2 * math.pi  # volts from feedforward
        fb = self._pid.calculate(current_rps, rps)
        self._voltage_applied = max(-12.0, min(12.0, ff + fb))

    def set_feeder(self, speed: float) -> None:
        self._feeder_speed = speed

    def stop(self) -> None:
        self._target_rps = 0.0
        self._voltage_applied = 0.0
        self._feeder_speed = 0.0
        self._pid.reset()
