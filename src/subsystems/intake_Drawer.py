import commands2
import wpimath.controller
import wpimath.trajectory
import rev
import wpilib
from wpimath.units import inchesToMeters
import math
import time
from wpilib import SmartDashboard

from src.constants import IntakeDrawerConstants


class drawerSubsystem(commands2.Subsystem):
    def __init__(self):
        self.leftDrawerMotor = rev.SparkMax(IntakeDrawerConstants.kLeftDrawerMotorId, rev.SparkMax.MotorType.kBrushless)
        self.rightDrawerMotor = rev.SparkMax(IntakeDrawerConstants.kRightDrawerMotorId, rev.SparkMax.MotorType.kBrushless)

        rev_resets = rev.ResetMode.kResetSafeParameters
        rev_persists = rev.PersistMode.kPersistParameters

        self.leftDrawerMotor.configure(IntakeDrawerConstants.k_config, rev_resets, rev_persists)
        self.rightDrawerMotor.configure(IntakeDrawerConstants.k_right_config, rev_resets, rev_persists)

        self._left_speed  = IntakeDrawerConstants.kBaseSpeed
        self._right_speed = IntakeDrawerConstants.kBaseSpeed * IntakeDrawerConstants.kRightSpeedScale

    def outhop(self):
        self.leftDrawerMotor.set(self._left_speed)
        self.rightDrawerMotor.set(self._right_speed)

    def inhop(self):
        self.leftDrawerMotor.set(-self._left_speed)
        self.rightDrawerMotor.set(-self._right_speed)

    def stopmotor(self):
        self.leftDrawerMotor.set(0)
        self.rightDrawerMotor.set(0)

    def periodic(self) -> None:
        SmartDashboard.putNumber("drawer pose", self.rightDrawerMotor.getEncoder().getPosition())
        SmartDashboard.putNumber("left-right drawer dist",
            self.leftDrawerMotor.getEncoder().getPosition() - self.rightDrawerMotor.getEncoder().getPosition())
        # Watch this value — it tells you how far out of sync they're getting
        SmartDashboard.putNumber("drawer/LeftSpeed",  self._left_speed)
        SmartDashboard.putNumber("drawer/RightSpeed", self._right_speed)