import rev
import wpilib
from wpilib import TimedRobot, Joystick, DigitalInput,  Timer
import commands2
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
import math
from src import constants
from wpimath.controller import PIDController
import navx
# import wpimath.geometry
# import wpimath.kinematics
# import wpimath.trajectory
# import wpimath.filter

class IntakeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.groundIntake: SparkMax = SparkMax(13, SparkMax.MotorType.kBrushless) # change ID later
        self.jointMotor: SparkMax = SparkMax(14, SparkMax.MotorType.kBrushless)
        self.pidIntake = PIDController(0.01, 0, 0) # PID values to be tuned later
        #Increase kP until the system responds quickly and oscillates then cut it in half, (first value)
        #increare kI until offset is corrected in a reasonable time (second value)
        #Increase kD until overshoot is eliminated. Reduce oscilation (third value) 
        self.intakeEncoder1 = self.groundIntake.getEncoder()
        self.intakeEncoder2 = self.jointMotor.getEncoder()
        self.intakeEncoder1.setPosition(0)
        self.intakeEncoder2.setPosition(0)

    def printHeight(self):
        print(self.intakeEncoder1.getPosition())

    def downRaw(self):
        self.jointMotor.set(0.2)
    
    def upRaw(self):
        self.jointMotor.set(-0.15)

    def intakeFast(self):
        self.groundIntake.set(-0.3)
    
    def intakeSpit(self):
        self.groundIntake.set(0.3)

    def stop(self):
        self.groundIntake.set(0.0)
        self.jointMotor.set(0.0)

class downRawCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
    def initialize(self):
        pass
    def execute(self):
        self.intake_subsystem.downRaw()
    def end(self, interrupted):
        self.intake_subsystem.stop()
        
class upRawCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
    def initialize(self):
        pass
    def execute(self):
        self.intake_subsystem.upRaw()
    def end(self, interrupted):
        self.intake_subsystem.stop()

class intakeFastCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
    def initialize(self):
        pass  
    def execute(self):
        self.intake_subsystem.intakeFast() 
    def end(self, interrupted): 
        self.intake_subsystem.stop()

class intakeSpitCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
    def initialize(self):
        pass
    def execute(self):
        self.intake_subsystem.intakeSpit()
    def end(self, interrupted):
        self.intake_subsystem.stop()

class PrintHeightCommand(Command):
    def __init__(self, intake_subsystem):
        super().__init__()
        self.intake_subsystem = intake_subsystem
    def initialize(self):
        pass
    def execute(self):
        self.intake_subsystem.printHeight()
    def end(self, interrupted):
        pass

#stop command???
# class xCommand(Command):
#     def __init__(self, intake_subsystem):
#         super().__init__()
#         self.intake_subsystem = intake_subsystem   
#     def initialize(self):
#         pass    
#     def execute(self):
#         self.intake_subsystem.stop()
#     def end(self, interrupted):
#         self.intake_subsystem.stop()