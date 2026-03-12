"""
Team 1504 - Shooter Commands
Each command is small and focused. Compose them in auto routines as needed.
"""

import wpilib
import commands2
from src.subsystems.shooter import ShooterSubsystem
from src.constants import ShooterConstants


class SpinUpCommand(commands2.Command):
    """Spin the flywheel to a fixed velocity. Runs until interrupted."""

    def __init__(self, shooter: ShooterSubsystem, target_rps: float = 42.0) -> None:
        super().__init__()
        self._shooter = shooter
        self._target_rps = target_rps
        self.addRequirements(shooter)

    def initialize(self) -> None:
        self._shooter.set_velocity_rps(self._target_rps)

    def execute(self) -> None:
        pass  # Motor runs on its own PID; nothing to do each loop

    def end(self, interrupted: bool) -> None:
        self._shooter.stop_shooter()

    def isFinished(self) -> bool:
        return False


class FeedCommand(commands2.Command):
    """Run the feeder only. Does NOT spin the flywheel."""

    def __init__(self, shooter: ShooterSubsystem) -> None:
        super().__init__()
        self._shooter = shooter
        self.addRequirements(shooter)

    def execute(self) -> None:
        self._shooter.run_feeder()

    def end(self, interrupted: bool) -> None:
        self._shooter.stop_feeder()

    def isFinished(self) -> bool:
        return False


class ShootCommand(commands2.Command):
    """
    Spin up flywheel, then feed once at speed.
    Runs indefinitely until button is released.
    """

    def __init__(self, shooter: ShooterSubsystem, target_rps: float = 42.0) -> None:
        super().__init__()
        self._shooter = shooter
        self._target_rps = target_rps
        self.addRequirements(shooter)

    def initialize(self) -> None:
        self._shooter.set_velocity_rps(self._target_rps)
        self._shooter.stop_feeder()

    def execute(self) -> None:
        if self._shooter.is_at_speed():
            self._shooter.run_feeder()
        else:
            self._shooter.stop_feeder()

    def end(self, interrupted: bool) -> None:
        self._shooter.stop_all()

    def isFinished(self) -> bool:
        return False


class AutoShootCommand(commands2.Command):
    """
    Autonomous shoot: spin up, wait for speed, feed for duration, then finish.

    FIXED: previously incremented a float counter by 0.02 each loop, which
    drifts whenever the loop runs late (brownout, CAN delays, etc.).
    Now uses wpilib.Timer for accurate wall-clock measurement.
    """

    def __init__(
        self,
        shooter: ShooterSubsystem,
        target_rps: float = 42.0,
        feed_duration_sec: float = 1.0,
    ) -> None:
        super().__init__()
        self._shooter = shooter
        self._target_rps = target_rps
        self._feed_duration = feed_duration_sec
        self._feeding = False
        self._timer = wpilib.Timer()
        self.addRequirements(shooter)

    def initialize(self) -> None:
        self._shooter.set_velocity_rps(self._target_rps)
        self._shooter.stop_feeder()
        self._feeding = False
        self._timer.reset()
        self._timer.stop()

    def execute(self) -> None:
        if self._shooter.is_at_speed():
            if not self._feeding:
                self._feeding = True
                self._timer.reset()
                self._timer.start()
            self._shooter.run_feeder()
        else:
            self._shooter.stop_feeder()

    def end(self, interrupted: bool) -> None:
        self._timer.stop()
        self._shooter.stop_all()

    def isFinished(self) -> bool:
        return self._feeding and self._timer.hasElapsed(self._feed_duration)