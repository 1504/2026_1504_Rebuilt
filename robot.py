import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller

import commands2
import src.subsystems.climb as climb
import src.constants as constants
import src.subsystems.intake as intake
from wpilib import Timer
import ntcore
from wpimath.units import inchesToMeters, lbsToKilograms
# To see messages from networktables:
import logging

logging.basicConfig(level=logging.DEBUG)

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        self.counter = 0
        #self.driver_controller = commands2.button.CommandXboxController(0)
        self.gadget_controller = commands2.button.CommandXboxController(1)
     
        self.climb_subsystem = climb.Elevator()
        self.intake_subsystem = intake.IntakeSubsystem()
        
        self.x_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.y_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.rot_limiter = wpimath.filter.SlewRateLimiter(3)

        # climb bindings
        self.gadget_controller.a().onTrue(climb.MoveElevator(self.climb_subsystem,mode='specified',height=inchesToMeters(7),use_dash=False,offset=0,wait_to_finish=True))
        self.gadget_controller.b().onTrue(climb.MoveElevator(self.climb_subsystem,mode='specified',height= inchesToMeters(5),use_dash=False,offset=0,wait_to_finish=True))
        self.gadget_controller.x().whileTrue(climb.printHeightCommand(self.climb_subsystem))
        self.gadget_controller.y().onTrue(climb.MoveElevator(self.climb_subsystem,mode='specified',height= inchesToMeters(3),use_dash=False,offset=0,wait_to_finish=True))
        
        # self.gadget_controller.x().whileTrue(climb.ClimbL2Command(self.climb_subsystem))
        # self.gadget_controller.b().whileTrue(climb.ClimbL1Command(self.climb_subsystem))
        # commands2.button.Trigger(lambda: self.gadget_controller.getLeftY() < -0.5).whileTrue(climb.ClimbUpCommand(self.climb_subsystem))
        # commands2.button.Trigger(lambda: self.gadget_controller.getLeftY() > 0.5).whileTrue(climb.ClimbDownCommand(self.climb_subsystem))

        # intake Bindings
        # self.gadget_controller.leftBumper().whileTrue(intake.PrimeCoralCommand(self.intake_subsystem))
        # self.gadget_controller.leftTrigger().whileTrue(intake.BackCoralCommand(self.intake_subsystem))
        # self.gadget_controller.rightBumper().whileTrue(intake.slowForwardCoralCommand(self.intake_subsystem))#slow corel
        # self.gadget_controller.rightTrigger().whileTrue(intake.fastForwardCoralCommand(self.intake_subsystem))#fast coral




        self.timer = Timer()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    
    def autonomousInit(self) -> None:
        pass


    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass
    

    def teleopPeriodic(self) -> None:
        pass
        # Teleop periodic logic
        # if self.driver_controller.getLeftTriggerAxis() > 0.1: 
        #     self.slowdwj(False)
        # elif self.driver_controller.getRightTriggerAxis() > 0.1:
        #     self.slowdwj(False)
        # elif self.driver_controller.rightBumper(True):
        #     self.slowdwj(False)
        # elif self.driver_controller.leftBumper(True):
        #     self.slowdwj(False)
        # else:
        #     self.driveWithJoystick(True)
        
    
    def testPeriodic(self) -> None:
        pass

   

if __name__ == "__main__":
    wpilib.run(MyRobot)