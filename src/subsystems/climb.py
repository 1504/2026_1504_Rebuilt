import commands2
import wpimath.controller
import wpimath.trajectory
import rev
import wpilib
from wpimath.units import inchesToMeters
import math
import time


from src.constants import ElevatorConstants

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

        self.climbingMotor1 = rev.SparkMax(ElevatorConstants.k_CAN_id, rev.SparkMax.MotorType.kBrushless)
        self.follower = rev.SparkMax(ElevatorConstants.k_follower_CAN_id, rev.SparkMax.MotorType.kBrushless)
        self.sparks = [self.climbingMotor1, self.follower]

        self.rev_resets = rev.SparkMax.ResetMode.kResetSafeParameters
        self.rev_persists = rev.SparkMax.PersistMode.kPersistParam

        controller_revlib_error_source = self.motor.configure(ElevatorConstants.k_config, self.rev_resets, self.rev_persists)
        controller_revlib_error_follower = self.follower.configure(ElevatorConstants.k_follower_config, self.rev_resets, self.rev_persists)
        print(f"Reconfigured elevator sparkmaxes. Controller status: \n {controller_revlib_error_source}\n {controller_revlib_error_follower}")

        self.controller = self.climbingMotor1.getClosedLoopController()

        self.encoder = self.climbingMotor1.getEncoder()

        self.enable()

    def useState(self, setpoint: wpimath.trajectory.TrapezoidProfile.State) -> None:
        # Calculate the feedforward from the setpoint
        # print("SETPOINT POSITION: " + str(math.degrees(setpoint.position)))
        feedforward = self.feedforward.calculate(setpoint.velocity/2)  # the 2 corrects for the 2x carriage speed


        # Add the feedforward to the PID output to get the motor output
        # TODO - check if the feedforward is correct in units for the sparkmax - documentation says 32, not 12
        self.controller.setReference(setpoint.position, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0, arbFeedforward=feedforward)
        # self.goal = setpoint.position  # don't want this - unless we want to plot the trapezoid

    def set_brake_mode(self, mode='brake'):
        if mode == 'brake':
            ElevatorConstants.k_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            ElevatorConstants.k_follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        else:
            ElevatorConstants.k_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
            ElevatorConstants.k_follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)

        # do not make the changes permanent
        rev_resets = rev.SparkMax.ResetMode.kNoResetSafeParameters
        rev_persists = rev.SparkMax.PersistMode.kNoPersistParameters
        self.climbingMotor1.configure(ElevatorConstants.k_config, rev_resets, rev_persists)
        self.follower.configure(ElevatorConstants.k_follower_config, rev_resets, rev_persists)
    
    def get_height(self):
        return self.encoder.getPosition()
    
    def printHeight(self):
        print(self.climbingEncoder1.getPosition())



    def defaultPos(self):
        #while self.elevatorEncoder2.getPosition() > constants.kDefaultPosRotation:
           # self.elevatorMotor1.set(-constants.kDefaultPosSpeed)
           # self.elevatorMotor2.set
        self.climbingEncoder1.setPosition(0)
        self.climbingEncoder2.setPosition(0)

    def set_goal(self, goal):
        # make our own sanity-check on the subsystem's setGoal function
        goal = goal if goal < ElevatorConstants.k_max_height else ElevatorConstants.k_max_height
        goal = goal if goal > ElevatorConstants.k_min_height else ElevatorConstants.k_min_height
        self.goal = goal
        # print(f'setting goal to {self.goal}')
        self.setGoal(self.goal)
        self.at_goal = False


    def move_meters(self, delta_meters: float, silent=False) -> None:  # way to bump up and down for testing
        current_position = self.get_height()
        goal = current_position + delta_meters
        self.set_goal(goal)  # check and set
        if not silent:
            message = f'setting {self.getName()} from {current_position:.2f} to {self.goal:.2f}'
            print(message)


    def get_at_goal(self):
        return self.at_goal


    def set_encoder_position(self, meters: float):
        self.encoder.setPosition(meters)


    def offset_encoder_position_meters(self, offset_meters):
        # allow the drivers to fine-tune the elevator until heights are good enough
        current_position = self.encoder.getPosition()
        new_position = current_position + offset_meters
        self.set_encoder_position(new_position)
        print(f' -- offset elevator by {offset_meters:.3f}m  (from {current_position:.3f}m  to {new_position:.3f}m) --')


    # note - this won't work with the way scoring are all sequential commands with no initialize phase
    def increment_scoring(self, delta_inches):  # correct for elevator sag by adding an inch or more to scoring positions
        positions = ['l1', 'l2', 'l3', 'l4']
        current_positions = []
        final_positions = []
        for position in positions:
            current_positions.append(round(ElevatorConstants.k_positions[position]['elevator'], 2))
            ElevatorConstants.k_positions[position]['elevator'] = ElevatorConstants.k_positions[position]['elevator'] + inchesToMeters(delta_inches)
            final_positions.append(round(ElevatorConstants.k_positions[position]['elevator'], 2))
        print(f'Elevator scoring changed from {current_positions} to {final_positions}')


    def increment_pickup(self, delta_inches):  # correct for elevator sag by adding an inch or more to human player positions
        positions = ['coral station']
        current_positions = []
        final_positions = []
        for position in positions:
            current_positions.append(round(ElevatorConstants.k_positions[position]['elevator'], 2))
            ElevatorConstants.k_positions[position]['elevator'] = ElevatorConstants.k_positions[position]['elevator'] + inchesToMeters(delta_inches)
            final_positions.append(round(ElevatorConstants.k_positions[position]['elevator'], 2))
        print(f'Coral pickup changed from {current_positions} to {final_positions}')


    def periodic(self) -> None:
        # What if we didn't call the below for a few cycles after we set the position?
        super().periodic()  # this does the automatic motion profiling in the background
        self.counter += 1
        if self.counter % 10 == 0:
            self.position = self.encoder.getPosition()
            self.at_goal = math.fabs(self.position - self.goal) < self.tolerance  # maybe we want to call this an error
            self.error = self.position - self.goal


            if ElevatorConstants.k_nt_debugging:  # add additional info to NT for debugging
                wpilib.SmartDashboard.putBoolean(f'{self.getName()}_at_goal', self.at_goal)
                wpilib.SmartDashboard.putNumber(f'{self.getName()}_error', self.error)
                wpilib.SmartDashboard.putNumber(f'{self.getName()}_goal', self.goal)
                # wpilib.SmartDashboard.putNumber(f'{self.getName()}_curr_sp',) not sure how to ask for this - controller won't give it
                wpilib.SmartDashboard.putNumber(f'{self.getName()}_output', self.motor.getAppliedOutput())
            self.is_moving = abs(self.encoder.getVelocity()) > 0.001  # m per second
            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_is_moving', self.is_moving)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_spark_pos', self.position * 1000)  #  make it mm

class MoveElevator(commands2.Command):  # change the name for your command


    def __init__(self, container, elevator: Elevator, mode='scoring', height=inchesToMeters(8), use_dash=True, offset=0, wait_to_finish=False, indent=0) -> None:
        super().__init__()
        self.setName('Move Elevator')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.elevator = elevator
        self.mode = mode
        self.height = height
        self.use_dash = use_dash  # testing mode - read target from dashboard?
        self.offset = offset  # attempt to have an offset
        self.wait_to_finish = wait_to_finish
        self.addRequirements(self.elevator)  # commandsv2 version of requirements
  


        # sick of IDE complaining
        self.start_time = None
        self.goal = None


    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)


        # a little bit complicated because I want to test everything here
        if self.mode == 'scoring':  # what will eventually be the norm
            self.goal = self.container.robot_state.get_elevator_goal() + self.offset
            self.elevator.set_goal(self.goal)
        elif self.mode == 'specified':  # send to a specific height
            self.goal = self.height
            self.elevator.set_goal(self.goal)
        elif self.mode == 'incremental':  # call from GUI to increment up and down
            self.goal = self.height  # height is a delta in this case
            self.elevator.move_meters(delta_meters=self.goal)

        else:
            print(f'Invalid Elevator move mode: {self.mode}')


        print(f"{self.indent * '    '}** Started {self.getName()} with mode {self.mode} and goal {self.goal:.2f} at {self.start_time} s **", flush=True)


    def execute(self) -> None:
        pass


    def isFinished(self) -> bool:
        if self.wait_to_finish:
            return self.elevator.get_at_goal() # TODO - put in a timeout, and probably a minimum time to allow to start moving
        else:
            return True




    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        end_message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        end_location = self.elevator.get_height()
        msg = f"{self.indent * '    '}** {end_message} {self.getName()} at {end_location:.3f}m after {end_time - self.start_time:.1f} s **"
        if print_end_message:
            print(msg)





class ClimbingPullUpManualCommand(commands2.Command):
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

class ClimbingPullUpCommand(commands2.Command):
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

class printHeightCommand(commands2.Command):
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

class pullUpClimbCommand(commands2.Command):
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

class pushDownClimbCommand(commands2.Command):
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