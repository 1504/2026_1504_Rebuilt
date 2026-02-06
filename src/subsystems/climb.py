import wpilib
from wpilib import TimedRobot, Joystick

from wpimath.controller import PIDController
import math

import rev
from rev import SparkMax, SparkMaxConfig, SparkBase

import commands2
from commands2 import Subsystem, Command

import subsystems.constants as constants

class ClimbSubsystem(Subsystem):
    def __init__(self):
        super().__init__()