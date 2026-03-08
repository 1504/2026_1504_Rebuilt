"""
Team 1504 - Drive Commands
"""

from typing import Callable
import commands2

from src.subsystems.drive import DriveSubsystem
from src.constants import OIConstants, DriveConstants


class TeleopDriveCommand(commands2.Command):
    """
    Field-relative swerve drive from joystick axes.

    Fixes for drift:
    - Rescaled deadband so output starts at exactly 0 at the edge (no jump)
    - Separate, slightly wider deadband for rotation axis
    - reset_slew() called in initialize() so stale limiter state from
      auto/disabled never bleeds into the first teleop loops

    Slow mode:
    - Pass a boolean supplier (e.g. lambda: driver.getLeftBumper())
    - While held, all inputs are scaled by kSlowModeMultiplier (default 30%)
    """

    def __init__(
        self,
        drive: DriveSubsystem,
        x_supplier: Callable[[], float],
        y_supplier: Callable[[], float],
        rot_supplier: Callable[[], float],
        field_relative: bool = True,
        slow_mode_supplier: Callable[[], bool] | None = None,
    ) -> None:
        super().__init__()
        self._drive = drive
        self._x = x_supplier
        self._y = y_supplier
        self._rot = rot_supplier
        self._field_relative = field_relative
        self._slow_mode = slow_mode_supplier or (lambda: False)
        self.addRequirements(drive)

    def initialize(self) -> None:
        # Always reset slew state when this command starts so the robot
        # doesn't drift from whatever the limiters were holding before.
        self._drive.reset_slew()

    def execute(self) -> None:
        x = _apply_deadband(self._x(), OIConstants.kDriveDeadband)
        y = _apply_deadband(self._y(), OIConstants.kDriveDeadband)
        r = _apply_deadband(self._rot(), OIConstants.kRotDeadband)

        if self._slow_mode():
            x *= OIConstants.kSlowModeMultiplier
            y *= OIConstants.kSlowModeMultiplier
            r *= OIConstants.kSlowModeMultiplier

        self._drive.drive(x, y, r, self._field_relative)

    def isFinished(self) -> bool:
        return False


class SetXCommand(commands2.InstantCommand):
    """Lock wheels in X pattern."""

    def __init__(self, drive: DriveSubsystem) -> None:
        super().__init__(drive.set_x, drive)


class ResetHeadingCommand(commands2.InstantCommand):
    """Zero the gyro heading."""

    def __init__(self, drive: DriveSubsystem) -> None:
        super().__init__(drive.zero_heading, drive)


class VisionSnapCommand(commands2.Command):
    """Rotate to align with AprilTag target. Placeholder — tune once vision is working."""

    def __init__(self, drive: DriveSubsystem, vision) -> None:
        super().__init__()
        self._drive = drive
        self._vision = vision
        self.addRequirements(drive)

    def execute(self) -> None:
        # TODO: get tx from vision, close-loop on rotation
        pass

    def isFinished(self) -> bool:
        return False


def _apply_deadband(value: float, deadband: float) -> float:
    """
    Deadband with rescaling so output starts at 0 right at the threshold
    and reaches 1.0 at full stick — no jump, no residual drift.
    """
    if abs(value) < deadband:
        return 0.0
    scaled = (abs(value) - deadband) / (1.0 - deadband)
    return scaled if value > 0 else -scaled