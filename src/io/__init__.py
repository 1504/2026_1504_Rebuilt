# src/io/__init__.py
"""
IO layer — hardware abstraction for simulation and testing.

Each module exposes:
  - An Inputs dataclass  (logged data read from hardware each loop)
  - A base IO class      (interface / no-op default)
  - A Real implementation (talks to actual motors/sensors)
  - A Sim implementation  (WPILib physics models)

Selection happens once in RobotContainer:
    is_real = wpilib.RobotBase.isReal()
    io = FooIOReal() if is_real else FooIOSim()
    self.foo = FooSubsystem(io)
"""

from src.io.shooter_io import ShooterIO, ShooterIOReal, ShooterIOSim, ShooterInputs
from src.io.intake_io  import IntakeIO,  IntakeIOReal,  IntakeIOSim,  IntakeInputs
from src.io.gyro_io    import GyroIO,    GyroIONavX,    GyroIOSim,    GyroInputs

__all__ = [
    "ShooterIO", "ShooterIOReal", "ShooterIOSim", "ShooterInputs",
    "IntakeIO",  "IntakeIOReal",  "IntakeIOSim",  "IntakeInputs",
    "GyroIO",    "GyroIONavX",    "GyroIOSim",    "GyroInputs",
]
