"""
Team 1504 - AutoBuilder
PathPlanner-based autonomous routines.

HOW TO ADD A NEW AUTO:
1. Create the path in PathPlanner GUI → deploy/pathplanner/paths/
2. Create a named auto in the GUI  → deploy/pathplanner/autos/
3. Add an option in RobotContainer._configure_auto_chooser() using the exact GUI name.

HOW NAMED COMMANDS WORK:
PathPlanner triggers these at markers during a path. Strings are case-sensitive
and must match exactly what's typed in the GUI. Current .auto files use:
  "Shoot", "Intake", "Climb", "spinup", "stop_shooter"
"""

import commands2
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, NamedCommands

from src.subsystems.drive import DriveSubsystem
from src.subsystems.shooter import ShooterSubsystem
from src.subsystems.intake import IntakeSubsystem
from src.vision.limelight import LimelightVision
from src.commands.shooter_commands import AutoShootCommand
from src.commands.intake_commands import IntakeCommand
from src.commands.drive_to_shoot_command import DriveToShootCommand
from src.constants import ShootingConstants


class PPAutoBuilder:
    def __init__(
        self,
        drive: DriveSubsystem,
        shooter: ShooterSubsystem,
        intake: IntakeSubsystem,
        vision: LimelightVision,   # needed for DriveToShootCommand in named commands
    ) -> None:
        self._drive   = drive
        self._shooter = shooter
        self._intake  = intake
        self._vision  = vision
        self._register_named_commands()

    def _register_named_commands(self) -> None:
        # "Shoot" drives to position then fires — used in all .auto files
        NamedCommands.registerCommand(
            "Shoot",
            DriveToShootCommand(self._drive, self._vision)
            .andThen(
                AutoShootCommand(
                    self._shooter,
                    target_rps=ShootingConstants.kTargetRps,
                    feed_duration_sec=20.0,
                )
            ),
        )
        NamedCommands.registerCommand(
            "Intake",
            IntakeCommand(self._intake, stop_on_fuel=False),
        )
        NamedCommands.registerCommand(
            "Climb",
            # Climbing is teleop-only. This is a no-op placeholder so the .auto
            # file doesn't crash if the marker fires during testing.
            commands2.InstantCommand(),
        )
        NamedCommands.registerCommand(
            "spinup",
            commands2.InstantCommand(
                lambda: self._shooter.set_velocity_rps(ShootingConstants.kTargetRps),
                self._shooter,
            ),
        )
        NamedCommands.registerCommand(
            "stop_shooter",
            commands2.InstantCommand(self._shooter.stop_all, self._shooter),
        )

    def build(self, name: str | None) -> commands2.Command:
        """Build a PathPlanner auto by its GUI name. Returns a no-op if not found."""
        if not name:
            return commands2.InstantCommand()
        try:
            return PathPlannerAuto(name)
        except Exception as e:
            print(f"[AutoBuilder] Could not load auto '{name}': {e}")
            return commands2.InstantCommand()

    def build_with_shoot(self, name: str | None) -> commands2.Command:
    #"""Build a PathPlanner auto and chain a shoot command at the end."""
        if not name:
            return commands2.InstantCommand()
        try:
            return PathPlannerAuto(name).andThen(
            AutoShootCommand(
                self._shooter,
                target_rps=ShootingConstants.kTargetRps,
                feed_duration_sec=20.0,
            )
        )
        except Exception as e:
            print(f"[AutoBuilder] Could not load auto '{name}': {e}")
        return commands2.InstantCommand()