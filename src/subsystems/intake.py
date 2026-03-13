"""
Team 1504 - IntakeSubsystem (IO-layer version)
"""

import commands2
from wpilib import SmartDashboard

from src.constants import IntakeConstants
from src.io.intake_io import IntakeIO, IntakeInputs


class IntakeSubsystem(commands2.Subsystem):
    def __init__(self, io: IntakeIO) -> None:
        super().__init__()
        self._io = io
        self._inputs = IntakeInputs()

    def periodic(self) -> None:
        self._io.update_inputs(self._inputs)
        SmartDashboard.putBoolean("Intake/HasFuel",     self._inputs.has_fuel)
        SmartDashboard.putNumber("Intake/LeftCurrent",  self._inputs.left_current)

    def intake_run(self, speed: float | None = None) -> None:
        self._io.set_speed(speed if speed is not None else IntakeConstants.kIntakeSpeed)

    def reverse(self, speed: float | None = None) -> None:
        self._io.set_speed(speed if speed is not None else IntakeConstants.kReverseSpeed)

    def stop(self) -> None:
        self._io.stop()

    def has_fuel(self) -> bool:
        return self._inputs.has_fuel

    # Sim-only helper — safely ignored on real hardware (IntakeIOReal has no such method)
    def sim_trigger_fuel(self) -> None:
        if hasattr(self._io, "sim_trigger_fuel_sensor"):
            self._io.sim_trigger_fuel_sensor()  # type: ignore[attr-defined]