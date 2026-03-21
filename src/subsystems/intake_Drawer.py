import commands2
import wpimath.controller
import wpimath.trajectory
import rev
import wpilib
from wpimath.units import inchesToMeters
import math
import time


from src.constants import IntakeDrawerConstants


class drawerSubsystem(commands2.Subsystem):
    def __init__(self):
        
        self.leftDrawerMotor = rev.SparkMax(IntakeDrawerConstants.kLeftDrawerMotorId, rev.SparkMax.MotorType.kBrushless)
        self.rightDrawerMotor = rev.SparkMax(IntakeDrawerConstants.kRightDrawerMotorId, rev.SparkMax.MotorType.kBrushless)
        self.sparks = [self.leftDrawerMotor, self.rightDrawerMotor]

        IntakeDrawerConstants.k_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        IntakeDrawerConstants.k_follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        
        rev_resets = rev.ResetMode.kResetSafeParameters    
        rev_persists = rev.PersistMode.kPersistParameters

        self.leftDrawerMotor.configure(IntakeDrawerConstants.k_config, rev_resets, rev_persists)
        self.rightDrawerMotor.configure(IntakeDrawerConstants.k_follower_config, rev_resets, rev_persists)       
        self.sped=0.05
    def outhop(self):
        self.leftDrawerMotor.set(self.sped)
        self.rightDrawerMotor.set(self.sped)
    def inhop(self):
        self.rightDrawerMotor.set(-self.sped)
        self.leftDrawerMotor.set(-self.sped)
    def stopmotor(self):
        self.rightDrawerMotor.set(0)
        self.leftDrawerMotor.set(0)
        pass