import rev
import wpilib
from wpilib import TimedRobot, Joystick, DigitalInput,  Timer
import commands2
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase, SparkFlex
import math
from src import constants
from wpimath.controller import PIDController
import navx
from commands2 import SequentialCommandGroup
# import wpimath.geometry
# import wpimath.kinematics
# import wpimath.trajectory
# import wpimath.filter

class IntakeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        # --- Slide Motors (NEO on SparkMax) ---
        self.slideMotor1 = rev.SparkMax(constants.kLeftIntakeSliderRevCANId, rev.SparkMax.MotorType.kBrushless)
        self.slideMotor2 = rev.SparkMax(constants.kRightIntakeSliderRevCanId, rev.SparkMax.MotorType.kBrushless)

        # --- Intake Motors (SparkFlex) ---
        self.intakeMotor1 = rev.SparkFlex(constants.kIntakeNeoFrontCanId, rev.SparkFlex.MotorType.kBrushless)
        self.intakeMotor2 = rev.SparkFlex(constants.kIntakeNeoRearCanID, rev.SparkFlex.MotorType.kBrushless)

        # Slide positions (rotations) — tune these to your actual slide travel
        self.SLIDE_IN  = 0.0
        self.SLIDE_OUT = 30.0

        # --- Configure Leader Slide Motor ---
        leaderConfig = rev.SparkMaxConfig()
        leaderConfig.inverted(False)
        leaderConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        leaderConfig.smartCurrentLimit(40)
        leaderConfig.softLimit.forwardSoftLimit(self.SLIDE_OUT)
        leaderConfig.softLimit.forwardSoftLimitEnabled(True)
        leaderConfig.softLimit.reverseSoftLimit(self.SLIDE_IN)
        leaderConfig.softLimit.reverseSoftLimitEnabled(True)
        leaderConfig.closedLoop \
            .setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder) \
            .pid(0.1, 0.0, 0.01) \
            .outputRange(-1.0, 1.0)
        leaderConfig.closedLoop.maxMotion \
            .maxVelocity(5000) \
            .maxAcceleration(3000) \
            .allowedClosedLoopError(0.5)

        self.slideMotor1.configure(
            leaderConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters
        )

        # --- Configure Follower Slide Motor ---
        followerConfig = rev.SparkMaxConfig()
        followerConfig.follow(constants.kLeftIntakeSliderRevCANId, invert=True)  # follow slideMotor1 (CAN ID 1), inverted
        followerConfig.smartCurrentLimit(40)

        self.slideMotor2.configure(
            followerConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters
        )

        # --- Configure Intake Leader (SparkFlex) ---
        intakeLeaderConfig = rev.SparkFlexConfig()
        intakeLeaderConfig.inverted(False)
        intakeLeaderConfig.setIdleMode(rev.SparkFlexConfig.IdleMode.kCoast)
        intakeLeaderConfig.smartCurrentLimit(40)

        self.intakeMotor1.configure(
            intakeLeaderConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters
        )

        # --- Configure Intake Follower (SparkFlex) ---
        intakeFollowerConfig = rev.SparkFlexConfig()
        intakeFollowerConfig.follow(constants.kIntakeNeoFrontCanId, invert=True)  # follow intakeMotor1 (CAN ID 3)
        intakeFollowerConfig.smartCurrentLimit(40)

        self.intakeMotor2.configure(
            intakeFollowerConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters
        )

        # Get encoder and closed loop controller from leader
        self.slideEncoder = self.slideMotor1.getEncoder()
        self.slideController = self.slideMotor1.getClosedLoopController()

        # Zero the encoder at startup
        self.slideEncoder.setPosition(0)

    # --- Slide Control ---
    def slideOut(self):
        self.slideController.setReference(
            self.SLIDE_OUT,
            rev.SparkMax.ControlType.kMAXMotionPositionControl
        )

    def slideIn(self):
        self.slideController.setReference(
            self.SLIDE_IN,
            rev.SparkMax.ControlType.kMAXMotionPositionControl
        )

    def getSlidePosition(self):
        return self.slideEncoder.getPosition()

    def isSlideOut(self, tolerance=1.0):
        return abs(self.getSlidePosition() - self.SLIDE_OUT) < tolerance

    def isSlideIn(self, tolerance=1.0):
        return abs(self.getSlidePosition() - self.SLIDE_IN) < tolerance

    # --- Intake Roller Control ---
    def intakeIn(self, speed=0.8):
        self.intakeMotor1.set(speed)

    def intakeOut(self, speed=0.8):
        self.intakeMotor1.set(-speed)

    def intakeStop(self):
        self.intakeMotor1.set(0)

    def periodic(self):
        wpilib.SmartDashboard.putNumber("Slide Position", self.getSlidePosition())
        wpilib.SmartDashboard.putBoolean("Slide Out", self.isSlideOut())
        wpilib.SmartDashboard.putBoolean("Slide In", self.isSlideIn())

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

class SlideOutCommand(Command):
    def __init__(self, intake: IntakeSubsystem):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)
    def initialize(self):
        self.intake.slideOut()
    def execute(self):
        pass
    def isFinished(self):
        return self.intake.isSlideOut()
    def end(self, interrupted):
        pass

class SlideInCommand(Command):
    def __init__(self, intake: IntakeSubsystem):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)
    def initialize(self):
        self.intake.slideIn()
    def execute(self):
        pass
    def isFinished(self):
        return self.intake.isSlideIn()
    def end(self, interrupted):
        pass

class IntakeInCommand(Command):
    def __init__(self, intake: IntakeSubsystem, speed: float = 0.8):
        super().__init__()
        self.intake = intake
        self.speed = speed
        self.addRequirements(intake)
    def initialize(self):
        self.intake.intakeIn(self.speed)
    def execute(self):
        pass
    def isFinished(self):
        return False
    def end(self, interrupted):
        self.intake.intakeStop()

class IntakeOutCommand(Command):
    def __init__(self, intake: IntakeSubsystem, speed: float = 0.8):
        super().__init__()
        self.intake = intake
        self.speed = speed
        self.addRequirements(intake)
    def initialize(self):
        self.intake.intakeOut(self.speed)
    def execute(self):
        pass
    def isFinished(self):
        return False
    def end(self, interrupted):
        self.intake.intakeStop()

class DeployAndIntakeCommand(SequentialCommandGroup):
    def __init__(self, intake: IntakeSubsystem):
        super().__init__(
            SlideOutCommand(intake),
            IntakeInCommand(intake)
        )

class RetractAndStopCommand(SequentialCommandGroup):
    def __init__(self, intake: IntakeSubsystem):
        super().__init__(
            IntakeOutCommand(intake),
            SlideInCommand(intake)
        )