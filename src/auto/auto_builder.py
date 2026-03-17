"""
Team 1504 - AutoBuilder
PathPlanner-based autonomous routines.

HOW TO ADD A NEW AUTO:
1. Create the path in the PathPlanner GUI and save it to
   deploy/pathplanner/paths/  (the GUI does this automatically).
2. Create a named auto in the PathPlanner GUI (combines paths + events)
   and save it to deploy/pathplanner/autos/.
3. Add an option to the chooser in RobotContainer._configure_auto_chooser()
   using the exact name you gave the auto in the GUI.

HOW NAMED COMMANDS WORK:
PathPlanner triggers named commands at markers during a path.
Register every command here using the EXACT string typed in the GUI —
matching is case-sensitive.  The .auto files currently use:
  "Shoot", "Intake", "Climb"
so those are the strings registered below.

COORDINATE SYSTEM:
PathPlanner uses WPILib field coordinates (blue alliance origin, +X away
from the blue driver station, +Y toward the left when viewed from above).
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

        self._register_named_commands()

    # ─────────────────────────────────────────────────────────────
    # NAMED COMMANDS
    # ─────────────────────────────────────────────────────────────
    def _register_named_commands(self) -> None:
        """
        Map string names to commands for PathPlanner event markers.

        FIXED: names were all lowercase ("shoot", "intake") but the .auto
        files use Title Case ("Shoot", "Intake", "Climb").  PathPlanner's
        named command lookup is case-sensitive, so the mismatched names
        silently did nothing during autonomous.  All names now match the
        .auto files exactly.
        """
        NamedCommands.registerCommand(
            "Shoot",
            AutoShootCommand(self._shooter, target_rps=42.0, feed_duration_sec=1.0),
        )
        NamedCommands.registerCommand(
            "Intake",
            IntakeCommand(self._intake, stop_on_fuel=False),
        )
        NamedCommands.registerCommand(
            "Climb",
            # Climb during auto is a no-op placeholder — climbing is teleop only.
            # Replace with a real ClimbCommand if autonomous climbing is added.
            commands2.InstantCommand(),
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