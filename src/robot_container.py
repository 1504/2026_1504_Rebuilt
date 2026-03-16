"""
Team 1504 Desperate Penguins - RobotContainer
Inspired by 6328 Mechanical Advantage's architecture.
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
from src.vision.limelight import LimelightVision
from src.auto.auto_builder import PPAutoBuilder
from src.subsystems.intake_Drawer import drawerSubsystem

import src.commands.drive_commands as drive_cmds
import src.commands.shooter_commands as shoot_cmds
import src.commands.intake_commands as intake_cmds
import src.commands.climb_commands as climb_cmds
import src.commands.intakedrawercommands as drawer_command


class RobotContainer:
    def __init__(self) -> None:
        # ── Subsystems ────────────────────────────────────────────
        self.drive   = DriveSubsystem()
        self.shooter = ShooterSubsystem()
        self.intake  = IntakeSubsystem()
        self.drawer = drawerSubsystem()
        self.climber = ClimberSubsystem()
        self.vision  = LimelightVision(self.drive)

        # ── Controllers ───────────────────────────────────────────
        self.driver   = commands2.button.CommandXboxController(OIConstants.kDriverPort)
        self.operator = commands2.button.CommandXboxController(OIConstants.kOperatorPort)

        # ── Default command: field-relative drive ─────────────────
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

        # ── Auto builder ──────────────────────────────────────────
        self.auto_builder = PPAutoBuilder(self.drive, self.shooter, self.intake)
        self._configure_auto_chooser()

        # ── Teleop bindings ───────────────────────────────────────
        self.configure_teleop_bindings()

    # ─────────────────────────────────────────────────────────────
    # TELEOP BINDINGS
    # ─────────────────────────────────────────────────────────────
    def configure_teleop_bindings(self) -> None:
        """
        Driver (port 0):
          Left stick      → translate
          Right stick X   → rotate
          LB              → slow mode (hold)
          RB              → set X / lock wheels
          Right trigger   → robot-relative drive (hold)
          Start           → reset gyro heading
          Back            → vision snap to target

        Operator (port 1):
          A               → shoot (spin up + feed)
          X               → feed only
          Y               → spin up only
          B               → intake
          Left bumper     → reverse intake
          Right bumper    → climb up
          Left trigger    → climb down
        """

        # ── Driver ────────────────────────────────────────────────
        self.driver.rightBumper().whileTrue(drive_cmds.SetXCommand(self.drive))
        self.driver.start().onTrue(drive_cmds.ResetHeadingCommand(self.drive))
        self.driver.back().whileTrue(drive_cmds.VisionSnapCommand(self.drive, self.vision))

        # Robot-relative drive while right trigger is held.
        # Uses the same sticks as normal driving — just ignores field heading.
        # Useful for precise alignment maneuvers against a field element.
        self.driver.rightTrigger(0.5).whileTrue(
            drive_cmds.RobotRelativeDriveCommand(
                self.drive,
                lambda: -self.driver.getLeftY(),
                lambda: -self.driver.getLeftX(),
                lambda: -self.driver.getRightX(),
                slow_mode_supplier=lambda: self.driver.getHID().getLeftBumper(),
            )
        )

        # ── Operator ──────────────────────────────────────────────
        self.operator.rightTrigger().whileTrue(shoot_cmds.ShootCommand(self.shooter))
        self.operator.rightBumper().whileTrue(shoot_cmds.FeedCommand(self.shooter))
        self.operator.a().whileTrue(shoot_cmds.SpinUpCommand(self.shooter))
        self.operator.leftTrigger().whileTrue(intake_cmds.IntakeCommand(self.intake))
        self.operator.leftBumper().whileTrue(intake_cmds.ReverseIntakeCommand(self.intake))
        self.operator.x().whileTrue(climb_cmds.ClimbUpCommand(self.climber))
        self.operator.y().whileTrue(climb_cmds.ClimbDownCommand(self.climber))
        self.operator.b().onTrue()

        # commands2.button.Trigger(
        #     lambda: self.operator.getLeftTriggerAxis() > 0.5
        # ).whileTrue(climb_cmds.ClimbDownCommand(self.climber))

    def configure_teleop(self) -> None:
        """Called by teleopInit — resets slew so the robot doesn't jerk on enable."""
        self.drive.reset_slew()

    # ─────────────────────────────────────────────────────────────
    # AUTONOMOUS
    # ─────────────────────────────────────────────────────────────
    def _configure_auto_chooser(self) -> None:
        self.auto_chooser = wpilib.SendableChooser()
        self.auto_chooser.setDefaultOption("Do Nothing", None)

        # ── Add your PathPlanner autos here ───────────────────────
        self.auto_chooser.addOption("showoff", "showoff")
        self.auto_chooser.addOption("B1",    "Blue One")
        self.auto_chooser.addOption("B2",    "Blue Two")
        self.auto_chooser.addOption("B3",    "Blue Three")
        self.auto_chooser.addOption("R1",    "Red One")
        self.auto_chooser.addOption("R2",    "Red Two")
        self.auto_chooser.addOption("R3",    "Red Three")
        self.auto_chooser.addOption("Test",    "TestAuto")
        self.auto_chooser.addOption("!",    "New New Path")
        # Add more as you create them in the PathPlanner GUI

        SmartDashboard.putData("Auto Mode", self.auto_chooser)

    def get_autonomous_command(self) -> commands2.Command | None:
        selected = self.auto_chooser.getSelected()
        return self.auto_builder.build(selected)