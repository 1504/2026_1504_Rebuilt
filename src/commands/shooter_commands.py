"""
Team 1504 - Shooter Commands
Each command is small and focused. Compose them in auto routines as needed.
"""

import wpilib
import commands2
from wpilib import SmartDashboard
from src.subsystems.shooter import ShooterSubsystem
from src.constants import ShooterConstants


# Shared mutable velocity state — all commands read/write this.
# Stored as a module-level list so it's a mutable reference (not a rebind).
_current_target_rps: list[float] = [ShooterConstants.kDefaultShooterRps]


def get_shooter_rps() -> float:
    return _current_target_rps[0]


def set_shooter_rps(rps: float) -> None:
    _current_target_rps[0] = max(
        ShooterConstants.kShooterMinRps,
        min(ShooterConstants.kShooterMaxRps, rps),
    )


def reset_shooter_rps() -> None:
    _current_target_rps[0] = ShooterConstants.kDefaultShooterRps


class SpinUpCommand(commands2.Command):
    """Spin the flywheel to the current shared target velocity. Runs until interrupted."""

    def __init__(self, shooter: ShooterSubsystem) -> None:
        super().__init__()
        self._shooter = shooter
        self.addRequirements(shooter)

    def initialize(self) -> None:
        self._shooter.set_velocity_rps(get_shooter_rps())

    def execute(self) -> None:
        # Re-apply each loop so live adjustments take effect immediately
        self._shooter.set_velocity_rps(get_shooter_rps())

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
    Spin up flywheel to current shared target velocity and run feeder.
    No speed gate — feeder runs immediately alongside the flywheel.
    Runs indefinitely until button is released.
    """

    def __init__(self, shooter: ShooterSubsystem) -> None:
        super().__init__()
        self._shooter = shooter
        self.addRequirements(shooter)

    def initialize(self) -> None:
        self._shooter.set_velocity_rps(get_shooter_rps())
        self._shooter.stop_feeder()

    def execute(self) -> None:
        self._shooter.set_velocity_rps(get_shooter_rps())
        self._shooter.run_feeder()

    def end(self, interrupted: bool) -> None:
        self._shooter.stop_all()

    def isFinished(self) -> bool:
        return False


class IncreaseShooterVelocityCommand(commands2.InstantCommand):
    """
    Bump the shared shooter target RPS up by one step.
    Bind with onTrue() so it fires once per button press.
    """

    def __init__(self, shooter: ShooterSubsystem) -> None:
        super().__init__(lambda: self._adjust(shooter))

    def _adjust(self, shooter: ShooterSubsystem) -> None:
        set_shooter_rps(get_shooter_rps() + ShooterConstants.kShooterRpsStep)
        rps = get_shooter_rps()
        SmartDashboard.putNumber("Shooter/TargetRPS_Adjusted", rps)
        wpilib.reportWarning(f"[Shooter] velocity → {rps:.1f} RPS", printTrace=False)


class DecreaseShooterVelocityCommand(commands2.InstantCommand):
    """
    Bump the shared shooter target RPS down by one step.
    Bind with onTrue() so it fires once per button press.
    """

    def __init__(self, shooter: ShooterSubsystem) -> None:
        super().__init__(lambda: self._adjust(shooter))

    def _adjust(self, shooter: ShooterSubsystem) -> None:
        set_shooter_rps(get_shooter_rps() - ShooterConstants.kShooterRpsStep)
        rps = get_shooter_rps()
        SmartDashboard.putNumber("Shooter/TargetRPS_Adjusted", rps)
        wpilib.reportWarning(f"[Shooter] velocity → {rps:.1f} RPS", printTrace=False)


class ResetShooterVelocityCommand(commands2.InstantCommand):
    """
    Reset the shared shooter target RPS to the compile-time default.
    Bind with onTrue().
    """

    def __init__(self, shooter: ShooterSubsystem) -> None:
        super().__init__(lambda: self._reset(shooter))

    def _reset(self, shooter: ShooterSubsystem) -> None:
        reset_shooter_rps()
        rps = get_shooter_rps()
        SmartDashboard.putNumber("Shooter/TargetRPS_Adjusted", rps)
        wpilib.reportWarning(f"[Shooter] velocity reset → {rps:.1f} RPS", printTrace=False)


class AutoShootCommand(commands2.Command):
    """
    Autonomous shoot: spin up and feed for duration, then finish.
    No speed gate — feeder runs immediately. Uses wpilib.Timer for
    accurate wall-clock measurement.
    """

    def __init__(
        self,
        shooter: ShooterSubsystem,
        target_rps: float | None = None,
        feed_duration_sec: float = 1.0,
    ) -> None:
        super().__init__()
        self._shooter = shooter
        self._explicit_rps = target_rps
        self._feed_duration = feed_duration_sec
        self._timer = wpilib.Timer()
        self.addRequirements(shooter)

    def initialize(self) -> None:
        rps = self._explicit_rps if self._explicit_rps is not None else get_shooter_rps()
        self._shooter.set_velocity_rps(rps)
        self._timer.reset()
        self._timer.start()

    def execute(self) -> None:
        # Keep commanding velocity every loop so PID stays active
        rps = self._explicit_rps if self._explicit_rps is not None else get_shooter_rps()
        self._shooter.set_velocity_rps(rps)
        self._shooter.run_feeder()

    def end(self, interrupted: bool) -> None:
        self._timer.stop()
        self._shooter.stop_all()

    def isFinished(self) -> bool:
        return self._timer.hasElapsed(self._feed_duration)