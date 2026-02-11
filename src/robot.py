import wpilib
import wpimath
import navx
import src.subsystems.drivesubsystem as drivesubsystem
import commands2
import src.subsystems.climb as climb
import src.subsystems.intake as intake
import src.constants as robotcontainer
import wpilib.drive
import wpimath.filter
import wpimath.controller

from wpilib import Timer

# To see messages from networktables, you must setup logging
import logging

logging.basicConfig(level=logging.DEBUG)

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        self.driver_controller = commands2.button.CommandXboxController(0)
        self.gadget_controller = commands2.button.CommandXboxController(1)