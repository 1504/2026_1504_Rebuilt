import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import navx
import src.subsystems.drivesubsystem as drivesubsystem
import commands2
import src.subsystems.climb as climb
import src.constants as constants
import src.subsystems.intake as intake
from wpilib import Timer
import ntcore
from src.subsystems.LimelightCamera import LimelightCamera
import math

# --- VISION SUBSYSTEM DEFINITION ---
class VisionSubsystem:
    def __init__(self):
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("limelight")
        self.tid_sub = self.table.getDoubleTopic("tid").subscribe(-1)
        self.ty_sub = self.table.getDoubleTopic("ty").subscribe(0.0)
        
        # Physical Camera Constants
        self.CAMERA_HEIGHT = 34.0  # Inches off floor
        self.CAMERA_PITCH = -22.0  # Degrees (downward)

    def get_tag_height(self, tag_id):
        """Returns height in inches based on 2026 specs."""
        # HUB AprilTags
        if tag_id in [2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27]:
            return 44.25
        # TOWER WALL & OUTPOST AprilTags
        elif tag_id in [15, 16, 31, 32, 13, 14, 29, 30]:
            return 21.75
        # TRENCH AprilTags
        elif tag_id in [1, 6, 7, 12, 17, 22, 23, 28]:
            return 35.0
        return 0.0

    def get_distance_to_tag(self) -> float:
        tag_id = int(self.tid_sub.get())
        ty = self.ty_sub.get()
        target_height = self.get_tag_height(tag_id)

        if target_height == 0 or tag_id == -1:
            return -1.0 

        angle_to_goal_rad = math.radians(self.CAMERA_PITCH + ty)
        
        # Protect against tan(0)
        denom = math.tan(angle_to_goal_rad)
        if abs(denom) < 1e-6:
            return -1.0
            
        # d = (h2 - h1) / tan(a1 + a2)
        distance = (target_height - self.CAMERA_HEIGHT) / denom
        return abs(distance)

# --- ORIGINAL ROBOT CODE (UNMODIFIED) ---

# To see messages from networktables:
import logging

logging.basicConfig(level=logging.DEBUG)

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        self.driver_controller = commands2.button.CommandXboxController(0)
        self.gadget_controller = commands2.button.CommandXboxController(1)
        self.swerve = drivesubsystem.DriveSubsystem()
        self.climb_subsystem = climb.ClimbSubsystem()
        self.intake_subsystem = intake.IntakeSubsystem()

        # --- VISION CALLBACK INITIALIZATION ---
        self.vision = VisionSubsystem()
        self.vision_log_timer = wpilib.Timer()
        self.vision_log_timer.start()
        
        self.camera = LimelightCamera("limelight")  # name of your camera goes in parentheses
        
        self.x_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.y_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.rot_limiter = wpimath.filter.SlewRateLimiter(3)

        # climb bindings
        #self.gadget_controller.a().whileTrue(climb.ClimbDownCommand(self.climb_subsystem))
        # self.gadget_controller.y().whileTrue(climb.ClimbL3Command(self.climb_subsystem))
        # self.gadget_controller.x().whileTrue(climb.ClimbL2Command(self.climb_subsystem))
        # self.gadget_controller.b().whileTrue(climb.ClimbL1Command(self.climb_subsystem))
        # commands2.button.Trigger(lambda: self.gadget_controller.getLeftY() < -0.5).whileTrue(climb.ClimbUpCommand(self.climb_subsystem))
        # commands2.button.Trigger(lambda: self.gadget_controller.getLeftY() > 0.5).whileTrue(climb.ClimbDownCommand(self.climb_subsystem))

        # intake Bindings
        # self.gadget_controller.leftBumper().whileTrue(intake.PrimeCoralCommand(self.intake_subsystem))
        # self.gadget_controller.leftTrigger().whileTrue(intake.BackCoralCommand(self.intake_subsystem))
        # self.gadget_controller.rightBumper().whileTrue(intake.slowForwardCoralCommand(self.intake_subsystem))#slow corel
        # self.gadget_controller.rightTrigger().whileTrue(intake.fastForwardCoralCommand(self.intake_subsystem))#fast coral

        ## Core Functions

        # NetworkTable initialization

        self.coreTableInstance = ntcore.NetworkTableInstance.getDefault()


        # Limelight subscribing and publishing setup

        # self.limelightTable = self.coreTableInstance.getTable("limelight") # IP's not work here



        # self.txLimelightSub = self.limelightTable.getDoubleTopic("tx").subscribe(0.0) # via limelight to limelightTable, for direct on robot retrieval

        # self.tyLimelightSub = self.limelightTable.getDoubleTopic("ty").subscribe(0.0)

        # self.taLimelightSub = self.limelightTable.getDoubleTopic("ta").subscribe(0.0)

        # self.tvLimelightSub = self.limelightTable.getDoubleTopic("tv").subscribe(0)

        # self.tidLimelightSub = self.limelightTable.getDoubleTopic("tid").subscribe(0.0)




        # # dataArray from 3D AprilTag

        # self.dataArrayLimelightSub = self.limelightTable.getDoubleArrayTopic("camerapose_targetspace").subscribe([6])



        #  Changing settings on Limelight

        # self.pipelineLimelightPub = self.limelightTable.getDoubleTopic("pipeline").publish() # via limelightTable to limelight

        # self.pipelineLimelightPub.set(limelight1_DefaultPipeline) # Default pipeline

        # self.streamLimelightPub = self.limelightTable.getDoubleTopic("stream").publish()

        #  self.coreTable = self.coreTableInstance.getTable("datatable")


        # # General subscribing and publishing setup

        # # Generic part 1 code for declaring a telemetry publishing ( this declaration is placed in robotInit)

        # # self.xxxPub = self.coreTable.getDoubleTopic("xxx").publish()

        # # self.xxxPub = self.coreTable.getBooleanTopic("xxx").publish()

        # # Drive system telemetry

        self.oldX=0
        self.timer = Timer()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

        # --- VISION TERMINAL CALLBACK ---
        if self.vision_log_timer.hasElapsed(0.5):
            dist = self.vision.get_distance_to_tag()
            if dist > 0:
                print(f"[RIO Log] AprilTag Distance: {dist:.2f} inches")
            else:
                print("[RIO Log] Searching for Tags...")
            self.vision_log_timer.reset()

    
    def autonomousInit(self) -> None:
        pass


    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        self.timer.reset()
        self.timer.start()
        pass
    

    def teleopPeriodic(self) -> None:
        # Teleop periodic logic
        #turn_to_object(self)
        self.turn_to_object()
        x = self.camera.getX()
        print(f"x={x}")
        if(self.timer.get() % 2 == 0):
            self.oldX=self.camera.getX()
    
    def testPeriodic(self) -> None:
        pass

    def driveWithJoystick(self, field_relative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        x_speed = (
            -self.x_speed_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getLeftY(), 0.08)
            )
            # * drivesubsystem.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        y_speed = (
            -self.y_speed_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getLeftX(), 0.08)
            )
            # * drivesubsystem.kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            -self.rot_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getRightX(),0.08)
            )
            # * drivesubsystem.kMaxSpeed
        )


        self.swerve.drive(x_speed, y_speed, rot, field_relative, rate_limit=True)

    def turn_to_object(self) -> None:
     #   self.swerve.drive(0,0,self.camera.getX() * 0.05, False,rate_limit=True)
      #  self.swerve.drive(.1,0,0, True,rate_limit=True)
        if(self.oldX-self.camera.getX()>5):
            self.swerve.drive(0,0,self.camera.getX() * 0.05, False, True)
        #elif(self.timer.get()%3==0):
        else:
            self.swerve.drive(.1,0,0, False ,True)



    def slowdwj(self, field_relative: bool) -> None:
        x_speed = (
            -self.x_speed_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getLeftY(), 0.08)
            )
             * 0.2
        )

        y_speed = (
            -self.y_speed_limiter.calculate(
                wpimath.applyDeadband(self.driver_controller.getLeftX(), 0.08)
            )
             * 0.2
        )

        rot = (
            -self.rot_limiter.calculate(
                wpimath.applyDeadband(self.camera.getX()* -0.5, 0.08)
            )
             * 0.2
        )


        self.swerve.drive(x_speed, y_speed, rot,field_relative, rate_limit=True)

if __name__ == "__main__":
    wpilib.run(MyRobot)
