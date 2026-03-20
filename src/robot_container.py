"""
Team 1504 Desperate Penguins - RobotContainer
Inspired by 6328 Mechanical Advantage's architecture.

────────────────────────────────────────────────────────────
CONTROLLER MODES
────────────────────────────────────────────────────────────
Flip the flag below to switch between layouts:

  SINGLE_CONTROLLER_TEST = True   → one Xbox controller, port 0
  SINGLE_CONTROLLER_TEST = False  → driver (port 0) + operator (port 1)

────────────────────────────────────────────────────────────
SINGLE-CONTROLLER TEST LAYOUT  (one Xbox, port 0)
────────────────────────────────────────────────────────────
  Left stick          → translate (field-relative)
  Right stick X       → rotate
  Left bumper (hold)  → slow mode

  A                   → shoot (spin up + feed while held)
  B                   → intake (while held)
  X                   → reverse intake (while held)
  Y                   → spin up flywheel only (while held)
  Right bumper        → feed only (while held)
  Right trigger       → climb up (while held)
  Left trigger        → climb down (while held)
  Start               → reset gyro heading
  Back                → lock wheels (X pattern)
  D-pad Up            → increase shooter velocity by one step
  D-pad Down          → decrease shooter velocity by one step
  D-pad Left          → reset shooter velocity to default

────────────────────────────────────────────────────────────
TWO-CONTROLLER COMP LAYOUT
────────────────────────────────────────────────────────────
Driver (port 0):
  Left stick          → translate (field-relative)
  Right stick X       → rotate
  Left bumper (hold)  → slow mode
  Right bumper        → lock wheels (X pattern)
  Right trigger       → robot-relative drive (hold)
  Start               → reset gyro heading
  Back                → vision snap to target

Operator (port 1):
  Right trigger       → shoot (spin up + feed while held)
  Right bumper        → feed only (while held)
  A                   → spin up flywheel only (while held)
  Left trigger        → intake (while held)
  Left bumper         → reverse intake (while held)
  X                   → climb up (while held)
  Y                   → climb down (while held)
  D-pad Up            → increase shooter velocity by one step
  D-pad Down          → decrease shooter velocity by one step
  D-pad Left          → reset shooter velocity to default
"""

import commands2
import commands2.button
import wpilib
from wpilib import SmartDashboard

from src.constants import OIConstants, ShooterConstants
from src.subsystems.drive import DriveSubsystem
from src.subsystems.shooter import ShooterSubsystem
from src.subsystems.intake import IntakeSubsystem
from src.subsystems.climber import ClimberSubsystem
from src.vision.limelight import LimelightVision
from src.auto.auto_builder import PPAutoBuilder
#from src.subsystems.intake_Drawer import drawerSubsystem

import src.commands.drive_commands as drive_cmds
import src.commands.shooter_commands as shoot_cmds
import src.commands.intake_commands as intake_cmds
import src.commands.climb_commands as climb_cmds
import src.commands.intakedrawercommands as drawer_cmds


# ─────────────────────────────────────────────────────────────────────────────
# ← FLIP THIS FLAG BEFORE PLUGGING IN / UNPLUGGING THE OPERATOR CONTROLLER
# ─────────────────────────────────────────────────────────────────────────────
SINGLE_CONTROLLER_TEST = False


class RobotContainer:
    def __init__(self) -> None:
        # ── Subsystems ────────────────────────────────────────────
        self.drive   = DriveSubsystem()
        self.shooter = ShooterSubsystem()
        self.intake  = IntakeSubsystem()
#        self.drawer = drawerSubsystem()
        self.climber = ClimberSubsystem()
        self.vision  = LimelightVision(self.drive)

        # ── Controllers ───────────────────────────────────────────
        self.driver = commands2.button.CommandXboxController(OIConstants.kDriverPort)

        if not SINGLE_CONTROLLER_TEST:
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

        # ── Publish default target RPS so it shows up on dashboard ─
        SmartDashboard.putNumber(
            "Shooter/TargetRPS_Adjusted", ShooterConstants.kDefaultShooterRps
        )

        # ── Teleop bindings ───────────────────────────────────────
        self.configure_teleop_bindings()

    # ─────────────────────────────────────────────────────────────
    # TELEOP BINDINGS
    # ─────────────────────────────────────────────────────────────
    def configure_teleop_bindings(self) -> None:
        if SINGLE_CONTROLLER_TEST:
            self._bind_single_controller()
        else:
            self._bind_two_controllers()

    # ── Single-controller test layout ────────────────────────────
    def _bind_single_controller(self) -> None:
        d = self.driver

        # Drive utilities
        d.back().onTrue(drive_cmds.SetXCommand(self.drive))
        d.start().onTrue(drive_cmds.ResetHeadingCommand(self.drive))

        # Shooter
        d.a().whileTrue(shoot_cmds.ShootCommand(self.shooter))
        d.y().whileTrue(shoot_cmds.SpinUpCommand(self.shooter))
        d.rightBumper().whileTrue(shoot_cmds.FeedCommand(self.shooter))

        # Intake
        d.b().whileTrue(intake_cmds.IntakeCommand(self.intake))
        d.x().whileTrue(intake_cmds.ReverseIntakeCommand(self.intake))

        # Climber
        d.rightTrigger(0.5).whileTrue(climb_cmds.ClimbUpCommand(self.climber))
        d.leftTrigger(0.5).whileTrue(climb_cmds.ClimbDownCommand(self.climber))

        # ── Shooter velocity tuning (D-pad) ───────────────────────
        d.povUp().onTrue(shoot_cmds.IncreaseShooterVelocityCommand(self.shooter))
        d.povDown().onTrue(shoot_cmds.DecreaseShooterVelocityCommand(self.shooter))
        d.povLeft().onTrue(shoot_cmds.ResetShooterVelocityCommand(self.shooter))

    # ── Two-controller comp layout ────────────────────────────────
    def _bind_two_controllers(self) -> None:
        d  = self.driver
        op = self.operator

        # ── Driver ────────────────────────────────────────────────
        d.rightBumper().whileTrue(drive_cmds.SetXCommand(self.drive))
        d.start().onTrue(drive_cmds.ResetHeadingCommand(self.drive))
        d.back().whileTrue(drive_cmds.VisionSnapCommand(self.drive, self.vision))

        d.rightTrigger(0.5).whileTrue(
            drive_cmds.RobotRelativeDriveCommand(
                self.drive,
                lambda: -d.getLeftY(),
                lambda: -d.getLeftX(),
                lambda: -d.getRightX(),
                slow_mode_supplier=lambda: d.getHID().getLeftBumper(),
            )
        )

        # ── Operator ──────────────────────────────────────────────
        op.rightTrigger(0.5).whileTrue(shoot_cmds.ShootCommand(self.shooter))
        op.rightBumper().whileTrue(shoot_cmds.FeedCommand(self.shooter))
        op.a().whileTrue(shoot_cmds.SpinUpCommand(self.shooter))

        op.b().onTrue(
            climb_cmds.ClimbLevel1(self.climber)
            .andThen(climb_cmds.ClimbLevel2(self.climber))
            .andThen(climb_cmds.ClimbLevel1(self.climber))
        )

        op.leftTrigger(0.5).whileTrue(intake_cmds.IntakeCommand(self.intake))
        op.leftBumper().whileTrue(intake_cmds.ReverseIntakeCommand(self.intake))

        op.x().whileTrue(climb_cmds.ClimbUpCommand(self.climber))
        op.y().whileTrue(climb_cmds.ClimbDownCommand(self.climber))

        # ── Shooter velocity tuning (operator D-pad) ──────────────
        # Up   → faster   |   Down → slower   |   Left → reset to default
        op.povUp().onTrue(shoot_cmds.IncreaseShooterVelocityCommand(self.shooter))
        op.povDown().onTrue(shoot_cmds.DecreaseShooterVelocityCommand(self.shooter))
        op.povLeft().onTrue(shoot_cmds.ResetShooterVelocityCommand(self.shooter))

    def configure_teleop(self) -> None:
        """Called by teleopInit — resets slew so the robot doesn't jerk on enable."""
        self.drive.reset_slew()

    # ─────────────────────────────────────────────────────────────
    # AUTONOMOUS
    # ─────────────────────────────────────────────────────────────
    def _configure_auto_chooser(self) -> None:
        self.auto_chooser = wpilib.SendableChooser()
        self.auto_chooser.setDefaultOption("Do Nothing", None)

        self.auto_chooser.addOption("showoff",  "showoff")
        self.auto_chooser.addOption("B1",       "Blue One")
        self.auto_chooser.addOption("B2",       "Blue Two")
        self.auto_chooser.addOption("B3",       "Blue Three")
        self.auto_chooser.addOption("R1",       "Red One")
        self.auto_chooser.addOption("R2",       "Red Two")
        self.auto_chooser.addOption("R3",       "Red Three")
        self.auto_chooser.addOption("Test",     "TestAuto")
        self.auto_chooser.addOption("!",        "New New Path")

        SmartDashboard.putData("Auto Mode", self.auto_chooser)

    def get_autonomous_command(self) -> commands2.Command | None:
        selected = self.auto_chooser.getSelected()
        return self.auto_builder.build(selected)