import commands2
import wpimath.controller
import wpimath.trajectory
import rev
import wpilib
from wpimath.units import inchesToMeters
import math
import time
from src.subsystems.intake_Drawer import drawerSubsystem as drawer 

class DrawerCommand(commands2.Command):  # change the name for your command


    def __init__(self, drawer: drawer, length=inchesToMeters(8), use_dash=True, offset=0, wait_to_finish=False, indent=0) -> None:
        super().__init__()
        self.setName('Move Drawer')  # change this to something appropriate for this command
        self.indent = indent
        self.drawer = drawer

        self.length = length # testing mode - read target from dashboard?  
        self.wait_to_finish = wait_to_finish
        self.addRequirements(self.drawer)  # commandsv2 version of requirements
  


        # sick of IDE complaining
        self.start_time = None
        self.goal = None


    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
       


        # a little bit complicated because I want to test everything here
      # if self.mode == 'specified':  # send to a specific height
       #    self.goal = self.length
       #    self.drawer.set_goal(self.goal)

        #else:
            #print(f'Invalid Elevator move mode: {self.mode}')


        print(f"{self.indent * '    '}** Started {self.getName()} with mode {self.mode} from {self.drawer.get_length} M and goal {self.goal:.2f} **", flush=True)


    def execute(self) -> None:
        pass


    def isFinished(self) -> bool:
        if self.wait_to_finish:
            return self.drawer.get_at_goal() # TODO - put in a timeout, and probably a minimum time to allow to start moving
        else:
            return True




    def end(self, interrupted: bool) -> None:
        
        end_message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        end_location = self.drawer.get_position()
        msg = f"{self.indent * '    '}** {end_message} {self.getName()} at {end_location:.3f}m  **"
        if print_end_message:
            print(msg)


