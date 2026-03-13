"""
Team 1504 Desperate Penguins - RobotContainer
IO layer injected here — subsystems are hardware-agnostic.
"""

import wpilib
import commands2
import commands2.button
from wpilib import SmartDashboard

from src.constants import OIConstants
from src.subsystems.drive import DriveSubsystem
from src.subsystems.shooter import ShooterSubsystem
from src.subsystems.intake import IntakeSubsystem
from src.subsystems.climber import ClimberSubsystem
from src.vision.limelight import LimelightVision
from src.auto.auto_builder import PPAutoBuilder

import src.commands.drive_commands as drive_cmds
import src.commands.shooter_commands as shoot_cmds
import src.commands.intake_commands as intake_cmds
import src.commands.climb_commands as climb_cmds

# IO implementations
from src.io.shooter_io import ShooterIOReal, ShooterIOSim
from src.io.intake_io import IntakeIOReal, IntakeIOSim
from src.io.gyro_io import GyroIONavX, GyroIOSim


class RobotContainer:
    def __init__(self) -> None:
        is_real = wpilib.RobotBase.isReal()

        # ── IO selection: Real on robot, Sim in simulation ────────
        shooter_io = ShooterIOReal() if is_real else ShooterIOSim()
        intake_io  = IntakeIOReal()  if is_real else IntakeIOSim()
        gyro_io    = GyroIONavX()    if is_real else GyroIOSim()

        # ── Subsystems ────────────────────────────────────────────
        self.drive   = DriveSubsystem(gyro_io)
        self.shooter = ShooterSubsystem(shooter_io)
        self.intake  = IntakeSubsystem(intake_io)
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

        # ── Sim-only: expose intake IO for sim triggers ───────────
        if not is_real:
            self._intake_io_sim: IntakeIOSim = intake_io  # type: ignore[assignment]
            self._gyro_io_sim: GyroIOSim     = gyro_io    # type: ignore[assignment]

    # ─────────────────────────────────────────────────────────────
    # SIM HELPERS  (called from robot.py simulationPeriodic)
    # ─────────────────────────────────────────────────────────────
    def update_sim(self) -> None:
        """
        Feed sim-specific state back each loop.
        Only called when RobotBase.isSimulation() is True.
        """
        if hasattr(self, "_gyro_io_sim"):
            self._gyro_io_sim.update_sim_state(self.drive.get_chassis_speeds())

    # ─────────────────────────────────────────────────────────────
    # TELEOP BINDINGS
    # ─────────────────────────────────────────────────────────────
    def configure_teleop_bindings(self) -> None:
        # ── Driver ────────────────────────────────────────────────
        self.driver.rightBumper().whileTrue(drive_cmds.SetXCommand(self.drive))
        self.driver.start().onTrue(drive_cmds.ResetHeadingCommand(self.drive))
        self.driver.back().whileTrue(drive_cmds.VisionSnapCommand(self.drive, self.vision))

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
        self.drive.reset_slew()

    # ─────────────────────────────────────────────────────────────
    # AUTONOMOUS
    # ─────────────────────────────────────────────────────────────
    def _configure_auto_chooser(self) -> None:
        self.auto_chooser = wpilib.SendableChooser()
        self.auto_chooser.setDefaultOption("Do Nothing", None)
        self.auto_chooser.addOption("Right",      "2PieceCenter")
        self.auto_chooser.addOption("Left",       "1PieceMobility")
        self.auto_chooser.addOption("Center",     "3PieceCenter")
        self.auto_chooser.addOption("Easy auton", "simple")
        self.auto_chooser.addOption("showoff",    "showoff")
        SmartDashboard.putData("Auto Mode", self.auto_chooser)

    def get_autonomous_command(self) -> commands2.Command | None:
        selected = self.auto_chooser.getSelected()
        return self.auto_builder.build(selected)