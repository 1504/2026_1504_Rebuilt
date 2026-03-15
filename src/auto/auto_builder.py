"""
Team 1504 - AutoBuilder
PathPlanner-based autonomous routines.

HOW TO ADD A NEW AUTO:
1. Create the path in the PathPlanner GUI and save it to
   src/main/deploy/pathplanner/paths/   (the GUI does this automatically)
2. Create a named auto in the PathPlanner GUI (combines paths + events)
   and save it to src/main/deploy/pathplanner/autos/
3. Add an option to the chooser in RobotContainer._configure_auto_chooser()
   using the exact name you gave the auto in the GUI.

HOW NAMED COMMANDS WORK:
PathPlanner can trigger named commands at waypoints during a path.
Register every command you want to trigger in _register_named_commands()
below using the exact string you typed in the PathPlanner GUI.

COORDINATE SYSTEM:
PathPlanner uses WPILib field coordinates (blue alliance origin, +X away
from blue driver station, +Y toward the left when viewed from above).
Paths are automatically mirrored for red alliance via _should_flip_path()
in drive.py.
"""

import commands2
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, NamedCommands

from src.subsystems.drive import DriveSubsystem
from src.subsystems.shooter import ShooterSubsystem
from src.subsystems.intake import IntakeSubsystem
from src.commands.shooter_commands import AutoShootCommand
from src.commands.intake_commands import IntakeCommand


class PPAutoBuilder:
    def __init__(
        self,
        drive: DriveSubsystem,
        shooter: ShooterSubsystem,
        intake: IntakeSubsystem,
    ) -> None:
        self._drive   = drive
        self._shooter = shooter
        self._intake  = intake

        # Register every command that PathPlanner paths can trigger by name.
        # The string here must match exactly what you typed in the GUI.
        self._register_named_commands()

    # ─────────────────────────────────────────────────────────────
    # NAMED COMMANDS
    # ─────────────────────────────────────────────────────────────
    def _register_named_commands(self) -> None:
        """
        Map string names → commands for PathPlanner event markers.
        Add an entry here for every named command you place in the GUI.
        """
        NamedCommands.registerCommand(
            "shoot",
            AutoShootCommand(self._shooter, target_rps=42.0, feed_duration_sec=1.0),
        )
        NamedCommands.registerCommand(
            "intake",
            IntakeCommand(self._intake, stop_on_fuel=False),  # was True
        )
        NamedCommands.registerCommand(
            "spinup",
            commands2.InstantCommand(
                lambda: self._shooter.set_velocity_rps(42.0), self._shooter
            ),
        )
        NamedCommands.registerCommand(
            "stop_shooter",
            commands2.InstantCommand(self._shooter.stop_all, self._shooter),
        )

    # ─────────────────────────────────────────────────────────────
    # BUILD BY NAME
    # ─────────────────────────────────────────────────────────────
    def build(self, name: str | None) -> commands2.Command:
        """
        Build a PathPlanner auto by its GUI name.
        Returns a do-nothing command if name is None or not found.
        """
        if not name:
            return commands2.InstantCommand()

        try:
            return PathPlannerAuto(name)
        except Exception as e:
            print(f"[AutoBuilder] Could not load auto '{name}': {e}")
            return commands2.InstantCommand()