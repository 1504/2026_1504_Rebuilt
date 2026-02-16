import commands2
import wpimath.controller
import wpimath.trajectory
import rev
import wpilib
from wpimath.units import inchesToMeters
import math

import constants
from constants import ElevatorConstants

class Elevator(commands2.TrapezoidProfileSubsystem):
    def __init__(self):
        super().__init__(
            constraints=wpimath.trajectory.TrapezoidProfile.Constraints(
                ElevatorConstants.k_max_velocity_meter_per_second,
                ElevatorConstants.k_max_acceleration_meter_per_sec_squared
            ),
            initial_position=ElevatorConstants.k_min_height,
            period=0.02,
        )
        self.feedforward = wpimath.controller.ElevatorFeedforward(
            kS=ElevatorConstants.k_kS_volts,
            kG=ElevatorConstants.k_kG_volts,
            kV=ElevatorConstants.k_kV_volt_second_per_radian,
            kA=ElevatorConstants.k_kA_volt_second_squared_per_meter,
            dt=0.02)
        

        self.counter = ElevatorConstants.k_counter_offset
        self.tolerance = 0.03  # meters - then we will be "at goal"
        self.goal = ElevatorConstants.k_min_height
        self.at_goal = True

        self.climbingMotor1 = rev.SparkMax((9, rev.SparkMax.MotorType.kBrushless))
        self.follower = rev.SparkMax(10, rev.SparkMax.MotorType.kBrushless)


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