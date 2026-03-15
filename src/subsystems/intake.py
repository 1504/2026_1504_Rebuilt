"""
Team 1504 - IntakeSubsystem
Dual SparkMax intake rollers + DIO fuel sensor.
"""

import commands2
import rev
import wpilib
from wpilib import DigitalInput, SmartDashboard
from rev import SparkMax, SparkMaxConfig

from src.constants import IntakeConstants


class IntakeSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        #self._left_motor  = SparkMax(IntakeConstants.kLeftMotorId,  SparkMax.MotorType.kBrushless)
        self._right_motor = SparkMax(IntakeConstants.kRightMotorId, SparkMax.MotorType.kBrushless)

        left_cfg  = SparkMaxConfig()
        right_cfg = SparkMaxConfig()
        left_cfg.smartCurrentLimit(IntakeConstants.kCurrentLimit)
        right_cfg.smartCurrentLimit(IntakeConstants.kCurrentLimit)
        right_cfg.inverted(False)  # Right motor faces opposite direction

        #self._left_motor.configure(left_cfg,  rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self._right_motor.configure(right_cfg, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        #self._fuel_sensor = DigitalInput(IntakeConstants.kFuelSensorChannel)

    # def periodic(self) -> None:
    #     SmartDashboard.putBoolean("Intake/HasFuel", self.has_fuel())
    #     #SmartDashboard.putNumber("Intake/LeftCurrent", self._left_motor.getOutputCurrent())

    def intake_run(self, speed: float | None = None) -> None:
        spd = speed if speed is not None else IntakeConstants.kIntakeSpeed
        #self._left_motor.set(spd)
        self._right_motor.set(spd)

    def reverse(self, speed: float | None = None) -> None:
        spd = speed if speed is not None else IntakeConstants.kReverseSpeed
        #self._left_motor.set(spd)
        self._right_motor.set(spd)

    def stop(self) -> None:
        #self._left_motor.set(0.0)
        self._right_motor.set(0.0)

    # def has_fuel(self) -> bool:
    #     """True when fuel sensor beam is broken (piece detected)."""
    #     #return not self._fuel_sensor.get()  # DIO is normally-high; inverted when blocked
        