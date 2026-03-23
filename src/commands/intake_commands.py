# """
# Team 1504 - Intake Commands
# """

# import commands2
# from src.subsystems.intake import IntakeSubsystem


# class IntakeCommand(commands2.Command):
#     """Run intake until interrupted or fuel sensor triggers."""

#     def __init__(self, intake: IntakeSubsystem, stop_on_fuel: bool = False) -> None:
#         super().__init__()
#         self._intake = intake
#         self._stop_on_fuel = stop_on_fuel
#         self.addRequirements(intake)

#     def execute(self) -> None:
#         self._intake.intake_run()

#     def end(self, interrupted: bool) -> None:
#         self._intake.stop()

#     def isFinished(self) -> bool:
#     # stop_on_fuel is True in auto but we have no sensor,
#     # so always return False — command runs until interrupted
#         return False


# class ReverseIntakeCommand(commands2.Command):
#     """Reverse intake to unjam."""

#     def __init__(self, intake: IntakeSubsystem) -> None:
#         super().__init__()
#         self._intake = intake
#         self.addRequirements(intake)

#     def execute(self) -> None:
#         self._intake.reverse()

#     def end(self, interrupted: bool) -> None:
#         self._intake.stop()

#     def isFinished(self) -> bool:
#         return False