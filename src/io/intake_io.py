"""
Team 1504 - IntakeIO
IO layer for the intake subsystem.

Real  → Dual SparkMax + DIO beam-break sensor
Sim   → Simple velocity model + virtual sensor (auto-trips after a delay)
"""

from __future__ import annotations
from dataclasses import dataclass
import wpilib
import wpimath.system.plant


@dataclass
class IntakeInputs:
    left_speed: float = 0.0
    right_speed: float = 0.0
    left_current: float = 0.0
    right_current: float = 0.0
    has_fuel: bool = False
    connected: bool = True


class IntakeIO:
    def update_inputs(self, inputs: IntakeInputs) -> None:
        pass

    def set_speed(self, speed: float) -> None:
        pass

    def stop(self) -> None:
        self.set_speed(0.0)


# ─────────────────────────────────────────────────────────────────────────────
# REAL HARDWARE
# ─────────────────────────────────────────────────────────────────────────────
class IntakeIOReal(IntakeIO):
    def __init__(self) -> None:
        import rev
        from rev import SparkMax, SparkMaxConfig
        import wpilib
        from src.constants import IntakeConstants

        self._left  = SparkMax(IntakeConstants.kLeftMotorId,  SparkMax.MotorType.kBrushless)
        self._right = SparkMax(IntakeConstants.kRightMotorId, SparkMax.MotorType.kBrushless)

        left_cfg  = SparkMaxConfig()
        right_cfg = SparkMaxConfig()
        left_cfg.smartCurrentLimit(IntakeConstants.kCurrentLimit)
        right_cfg.smartCurrentLimit(IntakeConstants.kCurrentLimit)
        right_cfg.inverted(True)

        self._left.configure(left_cfg,   rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self._right.configure(right_cfg, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self._sensor = wpilib.DigitalInput(IntakeConstants.kFuelSensorChannel)

    def update_inputs(self, inputs: IntakeInputs) -> None:
        inputs.left_speed    = self._left.get()
        inputs.right_speed   = self._right.get()
        inputs.left_current  = self._left.getOutputCurrent()
        inputs.right_current = self._right.getOutputCurrent()
        inputs.has_fuel      = not self._sensor.get()  # normally-high, inverted when blocked
        inputs.connected     = True

    def set_speed(self, speed: float) -> None:
        self._left.set(speed)
        self._right.set(speed)


# ─────────────────────────────────────────────────────────────────────────────
# SIMULATION
# ─────────────────────────────────────────────────────────────────────────────
class IntakeIOSim(IntakeIO):
    """
    Simulates intake rollers plus a virtual fuel sensor.

    To test 'has_fuel' in simulation:
      - Call sim_trigger_fuel_sensor() from a test or the Robot.simulationPeriodic()
      - Or press a button bound to intake.sim_trigger_fuel_sensor() in robot_container

    The sim_running flag tracks whether rollers are spinning so the sensor
    auto-resets when the intake is stopped (piece ejected / fed).
    """

    def __init__(self) -> None:
        self._speed: float = 0.0
        self._has_fuel_sim: bool = False

    def update_inputs(self, inputs: IntakeInputs) -> None:
        inputs.left_speed  = self._speed
        inputs.right_speed = self._speed
        # Rough current estimate: stall current * duty cycle
        inputs.left_current  = abs(self._speed) * 30.0
        inputs.right_current = abs(self._speed) * 30.0
        inputs.has_fuel  = self._has_fuel_sim
        inputs.connected = True

    def set_speed(self, speed: float) -> None:
        self._speed = speed
        # Clear fuel sensor when intake is reversed (piece ejected)
        if speed < 0:
            self._has_fuel_sim = False

    # Call this from simulationPeriodic or a sim-only button to "inject" a game piece
    def sim_trigger_fuel_sensor(self) -> None:
        self._has_fuel_sim = True

    def sim_clear_fuel_sensor(self) -> None:
        self._has_fuel_sim = False
