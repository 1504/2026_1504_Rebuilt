import rev
import wpilib
from wpilib import TimedRobot, Joystick, DigitalInput,  Timer
import commands2
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
import math
import src.constants as constants

class IntakeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.leftMotor = rev.SparkMax(12, rev.SparkMax.MotorType.kBrushless)
        self.rightMotor = rev.SparkMax(11, rev.SparkMax.MotorType.kBrushless)

        self.fuelsensor = DigitalInput(9)
        #self.fuel2 = DigitalInput(8)
        self.intake_complete = False
        self.timer = wpilib.Timer()