"""
Team 1504 Desperate Penguins - RobotContainer
Inspired by 6328 Mechanical Advantage's architecture.

All subsystem instantiation, controller binding, and auto selection
lives here. robot.py stays minimal.
"""

import commands2
import commands2.button
import wpilib
from wpilib import SmartDashboard

from src.constants import OIConstants
from src.subsystems.drive import DriveSubsystem
from src.subsystems.shooter import ShooterSubsystem
from src.subsystems.intake import IntakeSubsystem
from src.subsystems.climber import ClimberSubsystem
from src.subsystems.leds import LEDSubsystem
from src.vision.limelight import LimelightVision
from src.auto.auto_builder import AutoBuilder

import src.commands.drive_commands as drive_cmds
import src.commands.shooter_commands as shoot_cmds
import src.commands.intake_commands as intake_cmds
import src.commands.climb_commands as climb_cmds


class RobotContainer:
    """
    Wires together all subsystems, controllers, and commands.

    Teleop bindings live in configure_teleop_bindings().
    Autonomous selection is driven by a SmartDashboard chooser.
    """

    def __init__(self) -> None:
        # ── Subsystems ────────────────────────────────────────────
        self.drive = DriveSubsystem()
        self.shooter = ShooterSubsystem()
        self.intake = IntakeSubsystem()
        self.climber = ClimberSubsystem()
        self.leds = LEDSubsystem()
        self.vision = LimelightVision(self.drive)

        # ── Controllers ───────────────────────────────────────────
        self.driver = commands2.button.CommandXboxController(OIConstants.kDriverPort)
        self.operator = commands2.button.CommandXboxController(OIConstants.kOperatorPort)

        # ── Default commands ──────────────────────────────────────
        # Drive with joystick by default (field-relative)
        # Hold Left Bumper for slow mode
        self.drive.setDefaultCommand(
            drive_cmds.TeleopDriveCommand(
                self.drive,
                lambda: -self.driver.getLeftY(),
                lambda: -self.driver.getLeftX(),
                lambda: -self.driver.getRightX(),
                field_relative=True,
                slow_mode_supplier=lambda: self.driver.getHID().getLeftBumper(),
            )
        )

        # ── Auto chooser ──────────────────────────────────────────
        self.auto_builder = AutoBuilder(self.drive, self.shooter, self.intake)
        self._configure_auto_chooser()

        # ── Teleop bindings are configured in teleopInit ──────────
        # (allows bindings to be refreshed without reboot if needed)
        self.configure_teleop_bindings()

    # ─────────────────────────────────────────────────────────────
    # TELEOP BINDINGS
    # ─────────────────────────────────────────────────────────────
    def configure_teleop_bindings(self) -> None:
        """
        Map controller buttons → commands.

        Driver (port 0):
          Left stick      → translate
          Right stick X   → rotate
          LB              → slow mode (hold)
          RB              → set X / lock wheels
          Start           → reset gyro heading
          Back            → vision snap to target

        Operator (port 1):
          A               → shoot (spin up + feed)
          X               → feed only
          Y               → spin up only
          B               → intake + index
          Left bumper     → reverse intake
          Right bumper    → climb up
          Left trigger    → climb down
        """

        # ── Driver ────────────────────────────────────────────────
        self.driver.rightBumper().whileTrue(
            drive_cmds.SetXCommand(self.drive)
        )
        self.driver.start().onTrue(
            drive_cmds.ResetHeadingCommand(self.drive)
        )
        self.driver.back().whileTrue(
            drive_cmds.VisionSnapCommand(self.drive, self.vision)
        )

        # ── Operator ──────────────────────────────────────────────
        # Shooting
        self.operator.a().whileTrue(
            shoot_cmds.ShootCommand(self.shooter)          # spin + feed
        )
        self.operator.x().whileTrue(
            shoot_cmds.FeedCommand(self.shooter)           # feed only
        )
        self.operator.y().whileTrue(
            shoot_cmds.SpinUpCommand(self.shooter)         # spin only
        )
        self.operator.b().whileTrue(
            intake_cmds.IntakeCommand(self.intake)         # intake + index
        )
        self.operator.leftBumper().whileTrue(
            intake_cmds.ReverseIntakeCommand(self.intake)  # unjam
        )

        # Climbing
        self.operator.rightBumper().whileTrue(
            climb_cmds.ClimbUpCommand(self.climber)
        )
        commands2.button.Trigger(
            lambda: self.operator.getLeftTriggerAxis() > 0.5
        ).whileTrue(climb_cmds.ClimbDownCommand(self.climber))

    def configure_teleop(self) -> None:
        """Called by teleopInit on every teleop enable."""
        # Reset slew limiters so the robot doesn't drift or lurch from
        # whatever state they held during auto or disabled.
        self.drive.reset_slew()

    # ─────────────────────────────────────────────────────────────
    # AUTONOMOUS
    # ─────────────────────────────────────────────────────────────
    def _configure_auto_chooser(self) -> None:
        self.auto_chooser = wpilib.SendableChooser()
        self.auto_chooser.setDefaultOption("Do Nothing", None)
        self.auto_chooser.addOption("2-Piece Center", "two_piece_center")
        self.auto_chooser.addOption("1-Piece + Mobility", "one_piece_mobility")
        SmartDashboard.putData("Auto Mode", self.auto_chooser)

    def get_autonomous_command(self) -> commands2.Command | None:
        selected = self.auto_chooser.getSelected()
        if selected is None:
            return None
        return self.auto_builder.build(selected)