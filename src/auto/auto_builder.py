"""
Team 1504 - AutoBuilder
Builds autonomous command sequences by name.

PathPlanner is the preferred path-following library for 2026.
Install: robotpy-pathplannerlib

Each auto is a method that returns a commands2.Command.
The chooser in RobotContainer selects by string key.
"""

import commands2

from src.subsystems.drive import DriveSubsystem
from src.subsystems.shooter import ShooterSubsystem
from src.subsystems.intake import IntakeSubsystem
from src.commands.shooter_commands import AutoShootCommand
from src.commands.intake_commands import IntakeCommand


class AutoBuilder:
    def __init__(
        self,
        drive: DriveSubsystem,
        shooter: ShooterSubsystem,
        intake: IntakeSubsystem,
    ) -> None:
        self._drive = drive
        self._shooter = shooter
        self._intake = intake

        # ── Register PathPlanner auto builder ─────────────────────
        # Uncomment after adding pathplannerlib to requirements:
        #
        # from pathplannerlib.auto import AutoBuilder as PPAutoBuilder
        # from pathplannerlib.config import HolonomicPathFollowerConfig, PIDConstants, ReplanningConfig
        # from src.constants import DriveConstants, AutoConstants
        #
        # PPAutoBuilder.configureHolonomic(
        #     self._drive.get_pose,
        #     self._drive.reset_pose,
        #     self._get_chassis_speeds,
        #     self._drive_chassis_speeds,
        #     HolonomicPathFollowerConfig(
        #         PIDConstants(AutoConstants.kPxController, 0, 0),
        #         PIDConstants(AutoConstants.kPThetaController, 0, 0),
        #         DriveConstants.kMaxSpeedMps,
        #         DriveConstants.kTrackWidth / 2,
        #         ReplanningConfig(),
        #     ),
        #     self._drive,
        # )

    # ─────────────────────────────────────────────────────────────
    # PUBLIC: build by name
    # ─────────────────────────────────────────────────────────────
    def build(self, name: str) -> commands2.Command:
        routes = {
            "two_piece_center":   self._two_piece_center,
            "one_piece_mobility": self._one_piece_mobility,
        }
        builder = routes.get(name)
        if builder is None:
            return commands2.InstantCommand()
        return builder()

    # ─────────────────────────────────────────────────────────────
    # AUTO ROUTINES
    # ─────────────────────────────────────────────────────────────
    def _one_piece_mobility(self) -> commands2.Command:
        """Shoot preloaded piece, drive forward for mobility points."""
        return commands2.SequentialCommandGroup(
            AutoShootCommand(self._shooter, target_rps=42.0, feed_duration_sec=1.0),
            self._drive_seconds(2.0, x_speed=0.3),
        )

    def _two_piece_center(self) -> commands2.Command:
        """
        Shoot preload, intake second piece, shoot again.
        Replace drive_seconds with PathPlanner paths for accurate movement.
        """
        return commands2.SequentialCommandGroup(
            AutoShootCommand(self._shooter, target_rps=42.0, feed_duration_sec=1.0),
            commands2.ParallelRaceGroup(
                IntakeCommand(self._intake, stop_on_fuel=True),
                self._drive_seconds(1.5, x_speed=0.3),
            ),
            AutoShootCommand(self._shooter, target_rps=42.0, feed_duration_sec=1.0),
        )

    # ─────────────────────────────────────────────────────────────
    # HELPERS
    # ─────────────────────────────────────────────────────────────
    def _drive_seconds(self, seconds: float, x_speed: float = 0.0, y_speed: float = 0.0) -> commands2.Command:
        """Drive at fixed speed for a fixed duration. Rough — use PathPlanner for real paths."""
        return commands2.RunCommand(
            lambda: self._drive.drive(x_speed, y_speed, 0.0, False, False),
            self._drive,
        ).withTimeout(seconds)
