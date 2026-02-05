import rev
import wpilib
from wpilib import TimedRobot, Joystick, DigitalInput,  Timer
import commands2
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
import math
import constants
class ShootSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        #0 & 1 are placeholder numbers
        self.leftMotor = rev.SparkMax(12, rev.SparkMax.MotorType.kBrushless)
        self.rightMotor = rev.SparkMax(11, rev.SparkMax.MotorType.kBrushless)
        #self.placeholderNumber = 0 #Replace with actual number
    
        self.shoot_complete = False
        self.timer = wpilib.Timer()

        #conveyorDetector.whileActiveContinuous(new RunMotor());
        #self.motors = wpilib.MotorControllerGroup(self.leftMotor, self.rightMotor)
def shoot(self)
    #button press and go
        self.leftMotor.set(-0.2)
        self.rightMotor.set(0.2)
