"""
Team 1504 - Climb Commands
"""

import commands2
from src.subsystems.climber import ClimberSubsystem


class ClimbUpCommand(commands2.Command):
    def __init__(self, climber: ClimberSubsystem) -> None:
        super().__init__()
        self._climber = climber
        self.addRequirements(climber)

    def execute(self) -> None:
        self._climber.climb_up()

    def end(self, interrupted: bool) -> None:
        self._climber.stop()

    def isFinished(self) -> bool:
        return False


class ClimbDownCommand(commands2.Command):
    def __init__(self, climber: ClimberSubsystem) -> None:
        super().__init__()
        self._climber = climber
        self.addRequirements(climber)

    def execute(self) -> None:
        self._climber.climb_down()

    def end(self, interrupted: bool) -> None:
        self._climber.stop()

    def isFinished(self) -> bool:
        return False

class ClimbLevel1(commands2.Command):
    def __init__(self, climber: ClimberSubsystem) -> None:
        super().__init__()
        self._climber = climber
        self.addRequirements(climber)

    def execute(self) -> None:
        self._climber.levelone()

    def end(self, interrupted: bool) -> None:
        self._climber.stop()

    def isFinished(self) -> bool:
        return False

class ClimbLevel2(commands2.Command):
    def __init__(self, climber: ClimberSubsystem) -> None:
        super().__init__()
        self._climber = climber
        self.addRequirements(climber)

    def execute(self) -> None:
        self._climber.leveltwo()

    def end(self, interrupted: bool) -> None:
        self._climber.stop()

    def isFinished(self) -> bool:
        return False