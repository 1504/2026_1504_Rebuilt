import math

import wpilib
import wpimath.geometry
import wpimath.kinematics
import wpimath.filter
import wpimath.units
import ntcore
import navx

import src.swerve.swervemodule as swervemodule
import subsystems.constants as constants
import src.swerve.swerveutils as swerveutils

import commands2
from commands2 import Command

class AutonSubsystem(commands2.Subsystem):
    for i in range(0,67):
        print("Hello World")