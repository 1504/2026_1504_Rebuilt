import commands2
import wpimath.controller
import wpimath.trajectory
import rev
import wpilib
from wpimath.units import inchesToMeters
import math
import time


from src.constants import IntakeDrawerConstants


class drawerSubsystem(commands2.TrapezoidProfileSubsystem):
    def __init__(self):
        super().__init__(
            constraints=wpimath.trajectory.TrapezoidProfile.Constraints(
                IntakeDrawerConstants.k_max_velocity_meter_per_second,
                IntakeDrawerConstants.k_max_acceleration_meter_per_sec_squared
            ),
            initial_position=IntakeDrawerConstants.kDrawerStartPosition,
            period=0.02,
        )
        self.feedforward = wpimath.controller.ElevatorFeedforward(
            kS=IntakeDrawerConstants.k_kS_volts,
            kG=IntakeDrawerConstants.k_kG_volts,
            kV=IntakeDrawerConstants.k_kV_volt_second_per_radian,
            kA=IntakeDrawerConstants.k_kA_volt_second_squared_per_meter,
            dt=0.02)
        
        self.setName(IntakeDrawerConstants.k_name)
        
        self.tolerance = 0.01  # meters - then we will be "at goal"
        self.goal = IntakeDrawerConstants.kDrawerStartPosition
        self.at_goal = True

        self.leftDrawerMotor = rev.SparkMax(IntakeDrawerConstants.kLeftDrawerMotorId, rev.SparkMax.MotorType.kBrushless)
        self.rightDrawerMotor = rev.SparkMax(IntakeDrawerConstants.kRightDrawerMotorId, rev.SparkMax.MotorType.kBrushless)
        self.sparks = [self.leftDrawerMotor, self.rightDrawerMotor]

        
        IntakeDrawerConstants.k_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        IntakeDrawerConstants.k_follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        
        rev_resets = rev.ResetMode.kResetSafeParameters    
        rev_persists = rev.PersistMode.kPersistParameters

        self.leftDrawerMotor.configure(IntakeDrawerConstants.k_config, rev_resets, rev_persists)
        self.rightDrawerMotor.configure(IntakeDrawerConstants.k_follower_config, rev_resets, rev_persists)       

        self.controller = self.leftDrawerMotor.getClosedLoopController()

        self.encoder = self.leftDrawerMotor.getEncoder()
        self.encoder.setPosition(self.goal)
        self.enable()

    def useState(self, setpoint: wpimath.trajectory.TrapezoidProfile.State) -> None:
        # Calculate the feedforward from the setpoint
        # print("SETPOINT POSITION: " + str(math.degrees(setpoint.position)))
        feedforward = self.feedforward.calculate(setpoint.velocity)  # the 2 corrects for the 2x carriage speed


        # Add the feedforward to the PID output to get the motor output
        # TODO - check if the feedforward is correct in units for the sparkmax - documentation says 32, not 12
        self.controller.setReference(setpoint.position, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0, arbFeedforward=feedforward)
        # self.goal = setpoint.position  # don't want this - unless we want to plot the trapezoid

    def get_position(self) -> float:
        return self.encoder.getPosition()
    
    def printHeight(self):
        print(self.encoder.getPosition())

    def set_goal(self, goal):
        # make our own sanity-check on the subsystem's setGoal function
        goal = goal if goal < IntakeDrawerConstants.kDrawerDeployedPosition else IntakeDrawerConstants.kDrawerDeployedPosition
        goal = goal if goal > IntakeDrawerConstants.kDrawerStartPosition else IntakeDrawerConstants.kDrawerStartPosition
        self.goal = goal
        # print(f'setting goal to {self.goal}')
        self.setGoal(self.goal)
        self.at_goal = False
        
    def get_at_goal(self):
        return self.at_goal


    def set_encoder_position(self, meters: float):
        self.encoder.setPosition(meters)

    # note - this won't work with the way scoring are all sequential commands with no initialize phase

    def periodic(self) -> None:
        # What if we didn't call the below for a few cycles after we set the position?
        super().periodic()  # this does the automatic motion profiling in the background
        self.counter += 1
        if self.counter % 10 == 0:
            self.position = self.encoder.getPosition()
            self.at_goal = math.fabs(self.position - self.goal) < self.tolerance  # maybe we want to call this an error
            self.error = self.position - self.goal
            self.is_moving = abs(self.encoder.getVelocity()) > 0.001  # m per second
