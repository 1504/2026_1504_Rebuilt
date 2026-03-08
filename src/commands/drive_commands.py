"""
Team 1504 - Drive Commands
"""

from typing import Callable
import commands2
import wpimath.geometry

from src.subsystems.drive import DriveSubsystem
from src.constants import OIConstants


class TeleopDriveCommand(commands2.Command):
    """Standard field-relative swerve drive from joystick axes."""

    def __init__(
        self,
        drive: DriveSubsystem,
        x_supplier: Callable[[], float],
        y_supplier: Callable[[], float],
        rot_supplier: Callable[[], float],
        field_relative: bool = True,
    ) -> None:
        super().__init__()
        self._drive = drive
        self._x = x_supplier
        self._y = y_supplier
        self._rot = rot_supplier
        self._field_relative = field_relative
        self.addRequirements(drive)

    def execute(self) -> None:
        x = _deadband(self._x(), OIConstants.kDriveDeadband)
        y = _deadband(self._y(), OIConstants.kDriveDeadband)
        r = _deadband(self._rot(), OIConstants.kDriveDeadband)
        self._drive.drive(x, y, r, self._field_relative)

    def isFinished(self) -> bool:
        return False


class SetXCommand(commands2.InstantCommand):
    """Lock wheels in X pattern."""

    def __init__(self, drive: DriveSubsystem) -> None:
        super().__init__(drive.set_x, drive)


class ResetHeadingCommand(commands2.InstantCommand):
    """Zero the gyro heading (sets forward to current facing direction)."""

    def __init__(self, drive: DriveSubsystem) -> None:
        super().__init__(drive.zero_heading, drive)


class VisionSnapCommand(commands2.Command):
    """
    Rotate to align with AprilTag target while driver controls translation.
    Placeholder - expand once vision pipeline is tuned.
    """

    def __init__(self, drive: DriveSubsystem, vision) -> None:
        super().__init__()
        self._drive = drive
        self._vision = vision
        self.addRequirements(drive)

    def execute(self) -> None:
        # TODO: get yaw offset from vision, feed to drive rotation
        pass

    def isFinished(self) -> bool:
        return False


def _deadband(value: float, deadband: float) -> float:
    return value if abs(value) > deadband else 0.0
