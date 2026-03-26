"""
Team 1504 - Drive Commands
"""

from typing import Callable
import commands2
import wpilib

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


class RobotRelativeDriveCommand(commands2.Command):
    """
    Robot-relative swerve drive — forward on the stick is always forward
    from the robot's perspective, regardless of which way it's facing.

    Bind with whileTrue() in RobotContainer. While held it overrides the
    default field-relative command. When released the scheduler automatically
    restores TeleopDriveCommand.
    """

    def __init__(
        self,
        drive: DriveSubsystem,
        x_supplier: Callable[[], float],
        y_supplier: Callable[[], float],
        rot_supplier: Callable[[], float],
        slow_mode_supplier: Callable[[], bool] | None = None,
    ) -> None:
        super().__init__()
        self._drive = drive
        self._x = x_supplier
        self._y = y_supplier
        self._rot = rot_supplier
        self._slow_mode = slow_mode_supplier or (lambda: False)
        self.addRequirements(drive)

    def initialize(self) -> None:
        # Reset slew so there's no jerk when switching modes mid-drive
        self._drive.reset_slew()

    def execute(self) -> None:
        x = _apply_deadband(self._x(), OIConstants.kDriveDeadband)
        y = _apply_deadband(self._y(), OIConstants.kDriveDeadband)
        r = _apply_deadband(self._rot(), OIConstants.kRotDeadband)

        if self._slow_mode():
            x *= OIConstants.kSlowModeMultiplier
            y *= OIConstants.kSlowModeMultiplier
            r *= OIConstants.kSlowModeMultiplier

        # field_relative=False is the only difference from TeleopDriveCommand
        self._drive.drive(x, y, r, field_relative=False)

    def end(self, interrupted: bool) -> None:
        # Reset slew on exit so the default command resumes cleanly
        self._drive.reset_slew()

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
    """
    Rotate to align with AprilTag target.
    Returns immediately until the PID loop is implemented.
    """

    def __init__(self, drive: DriveSubsystem, vision) -> None:
        super().__init__()
        self._drive = drive
        self._vision = vision
        self.addRequirements(drive)

    def initialize(self) -> None:
        wpilib.reportWarning(
            "[VisionSnapCommand] Not yet implemented — command will exit immediately.",
            printTrace=False,
        )

    def execute(self) -> None:
        # TODO: closed-loop on self._vision.get_tx() once validated
        pass

    def isFinished(self) -> bool:
        return True


def _apply_deadband(value: float, deadband: float) -> float:
    """
    Deadband with rescaling so output starts at 0 right at the threshold
    and reaches 1.0 at full stick — no jump, no residual drift.
    """
    if abs(value) < deadband:
        return 0.0
    scaled = (abs(value) - deadband) / (1.0 - deadband)
    return scaled if value > 0 else -scaled