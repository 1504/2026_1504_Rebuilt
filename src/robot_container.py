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
#from src.subsystems.leds import LEDSubsystem #later thingy
from src.vision.limelight import LimelightVision
from src.auto.auto_builder import PPAutoBuilder

import src.commands.drive_commands as drive_cmds
import src.commands.shooter_commands as shoot_cmds
import src.commands.intake_commands as intake_cmds
import src.commands.climb_commands as climb_cmds


class RobotContainer:
    def __init__(self) -> None:
        # ── Subsystems ────────────────────────────────────────────
        self.drive   = DriveSubsystem()   # registers PathPlanner AutoBuilder
        self.shooter = ShooterSubsystem()
        self.intake  = IntakeSubsystem()
        self.climber = ClimberSubsystem()
#        self.leds    = LEDSubsystem()  #later thingy
        self.vision  = LimelightVision(self.drive)

        # ── Controllers ───────────────────────────────────────────
        self.driver   = commands2.button.CommandXboxController(OIConstants.kDriverPort)
        self.operator = commands2.button.CommandXboxController(OIConstants.kOperatorPort)

        # ── Default commands ──────────────────────────────────────
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

        # ── Auto builder (must be created AFTER DriveSubsystem so
        #    PathPlanner AutoBuilder.configure() has already run) ──
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

        # ── Operator ──────────────────────────────────────────────
        self.operator.a().whileTrue(shoot_cmds.ShootCommand(self.shooter))
        self.operator.x().whileTrue(shoot_cmds.FeedCommand(self.shooter))
        self.operator.y().whileTrue(shoot_cmds.SpinUpCommand(self.shooter))
        self.operator.b().whileTrue(intake_cmds.IntakeCommand(self.intake))
        self.operator.leftBumper().whileTrue(intake_cmds.ReverseIntakeCommand(self.intake))
        self.operator.rightBumper().whileTrue(climb_cmds.ClimbUpCommand(self.climber))
        commands2.button.Trigger(
            lambda: self.operator.getLeftTriggerAxis() > 0.5
        ).whileTrue(climb_cmds.ClimbDownCommand(self.climber))

    def configure_teleop(self) -> None:
        """Called by teleopInit — resets slew so the robot doesn't jerk on enable."""
        self.drive.reset_slew()

    # ─────────────────────────────────────────────────────────────
    # AUTONOMOUS
    # ─────────────────────────────────────────────────────────────
    def _configure_auto_chooser(self) -> None:
        """
        Add every PathPlanner auto name here.
        The string must match the filename in deploy/pathplanner/autos/
        exactly (without the .auto extension).
        """
        self.auto_chooser = wpilib.SendableChooser()
        self.auto_chooser.setDefaultOption("Do Nothing", None)

        # ── Add your PathPlanner autos here ───────────────────────
        self.auto_chooser.addOption("Right",    "2PieceCenter")
        self.auto_chooser.addOption("Left",  "1PieceMobility")
        self.auto_chooser.addOption("Center",    "3PieceCenter")
        self.auto_chooser.addOption("Easy auton", "ExactFileNameNoExtension")
        # Add more as you create them in the PathPlanner GUI

        SmartDashboard.putData("Auto Mode", self.auto_chooser)

    def get_autonomous_command(self) -> commands2.Command | None:
        selected = self.auto_chooser.getSelected()
        return self.auto_builder.build(selected)