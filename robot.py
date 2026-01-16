import wpilib
import wpimath
import navx
import drivesubsystem
import commands2
import climb
import intake
import constants
import wpilib.drive
import wpimath.filter
import wpimath.controller

from wpilib import Timer

# To see messages from networktables, you must setup logging
import logging

logging.basicConfig(level=logging.DEBUG)

class MyRobot(commands2.TimedCommandRobot):