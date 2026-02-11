import wpilib
from wpilib import TimedRobot, Joystick

from wpimath.controller import PIDController
import math

import rev
from rev import SparkMax, SparkMaxConfig, SparkBase

import commands2
from commands2 import Subsystem, Command

import src.constants as constants

import src.constants

import time

import src.constants as constants

class Climbing(Subsystem):
    def __init__(self):
        super().__init__()
        self.climbingOffset=1
        self.climbingMotor1: SparkMax = SparkMax(16, SparkMax.MotorType.kBrushless)
        self.climbingMotor2: SparkMax = SparkMax(17, SparkMax.MotorType.kBrushless)


        self.climbingEncoder1 = self.climbingMotor1.getEncoder()
        self.climbingEncoder2 = self.climbingMotor2.getEncoder()

        self.climbingEncoder1.setPosition(0)
        self.climbingEncoder2.setPosition(0)
        self.pidController1 = PIDController(0.0,0.0,0.0)
        self.pidController2 = PIDController(0.0,0.0,0.0)
    def stop(self):
        self.climbingMotor1.set(0.0)
        self.climbingMotor2.set(0.0)

    def printHeight(self):
        print(self.climbingEncoder1.getPosition())

    def pullup(self):
         self.climbingMotor1.set(0.1)
         self.climbingMotor2.set(0.1)

    def pushDown(self):
        self.climbingMotor1.set(-0.03)
        self.climbingMotor2.set(-0.03)

    def PullUpManual(self):
        self.climbingMotor1.set(-0.10)
        self.climbingMotor2.set(-0.10)

    def defaultPos(self):
        #while self.elevatorEncoder2.getPosition() > constants.kDefaultPosRotation:
           # self.elevatorMotor1.set(-constants.kDefaultPosSpeed)
           # self.elevatorMotor2.set
        self.climbingEncoder1.setPosition(0)
        self.climbingEncoder2.setPosition(0)

        

    def l1(self):
        self.climbingMotor1.set((-1*self.pidController1.calculate(10+self.climbingOffset, self.climbingEncoder1.getPosition()))*0.5)
    def l2(self):
        self.climbingMotor1.set((-1*self.pidController1.calculate(20+self.climbingOffset, self.climbingEncoder1.getPosition()))*0.5)
    def l3(self):
        self.climbingMotor1.set((-1*self.pidController1.calculate(30+self.climbingOffset, self.climbingEncoder1.getPosition()))*0.5)
class ClimbingPullUpManualCommand(Command):
    def __init__(self, climbing_subsystem):
        super().__init__()

        self.climbing_subsystem = climbing_subsystem
        
    #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.climbing_subsystem.PushDown() 

    def end(self, interrupted):
        self.climbing_subsystem.stop()

class ClimbingPullUpCommand(Command):
    def __init__(self, climbing_subsystem):
        super().__init__()

        self.climbing_subsystem = climbing_subsystem
        
    #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.climbing_subsystem.pullup() 

    def end(self, interrupted):
        self.climbing_subsystem.stop()

class printHeightCommand(Command):
    def __init__(self, climbing_subsystem):
        super().__init__()

        self.climbing_subsystem = climbing_subsystem

        
     #stopped here
    def initialize(self):
        pass

    def execute(self):
        self.climbing_subsystem.printHeight()

    def end(self, interrupted):
        self.climbing_subsystem.stop()

class pullUpClimbCommand(Command):
    def __init__(self, climb_subsystem):
        super().__init__()

        self.climb_subsystem = climb_subsystem

    def initialize(self):
        self.climb_subsystem.pullup()
        self.start_time = time.time()
        self.inTime = time.time() + 0.20

    def execute(self):
        pass

    def isFinished(self):
        return time.time() > self.inTime

    def end(self, interrupted):
        self.climb_subsystem.stop()

class pushDownClimbCommand(Command):
    def __init__(self, climb_subsystem):
        super().__init__()

        self.climb_subsystem = climb_subsystem

    def initialize(self):
        self.climb_subsystem.pushdown()
        self.start_time = time.time()
        self.inTime = time.time() + 0.20

    def execute(self):
        pass

    def isFinished(self):
        return time.time() > self.inTime

    def end(self, interrupted):
        self.climb_subsystem.stop()