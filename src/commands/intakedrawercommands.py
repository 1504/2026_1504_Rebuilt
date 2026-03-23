# import commands2
# import wpimath.controller
# import wpimath.trajectory
# import rev
# import wpilib
# from wpimath.units import inchesToMeters
# import math
# import time
# from src.subsystems.intake_Drawer import drawerSubsystem as drawer 

# class InCommand(commands2.Command):
#     """
#     Spin up flywheel to current shared target velocity, then feed once at speed.
#     Runs indefinitely until button is released.
#     """

#     def __init__(self, shooter: drawer) -> None:
#         super().__init__()
#         self._shooter = shooter
#         self.addRequirements(shooter)

#     def initialize(self) -> None:
#         pass

#     def execute(self) -> None:
#         # Re-apply each loop so live adjustments take effect while held
#         self._shooter.inhop()

#     def end(self, interrupted: bool) -> None:
#         self._shooter.stopmotor()

#     def isFinished(self) -> bool:
#         return False

# class OutCommand(commands2.Command):
#     """
#     Spin up flywheel to current shared target velocity, then feed once at speed.
#     Runs indefinitely until button is released.
#     """

#     def __init__(self, shooter: drawer) -> None:
#         super().__init__()
#         self._shooter = shooter
#         self.addRequirements(shooter)

#     def initialize(self) -> None:
#         pass

#     def execute(self) -> None:
#         # Re-apply each loop so live adjustments take effect while held
#         self._shooter.outhop()

#     def end(self, interrupted: bool) -> None:
#         self._shooter.stopmotor()

#     def isFinished(self) -> bool:
#         return False
