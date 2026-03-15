"""
Team 1504 Desperate Penguins - Constants
2026 Season | FRC Game: Rebuilt
"""

import math
from wpimath.units import inchesToMeters, lbsToKilograms
from rev import ClosedLoopSlot, SparkClosedLoopController
import wpimath.units
import rev
import wpimath.trajectory
from rev import SparkMaxConfig


# ─────────────────────────────────────────────────────────────────────────────
# OPERATOR INTERFACE
# ─────────────────────────────────────────────────────────────────────────────
class OIConstants:
    kDriverPort = 0
    kOperatorPort = 1
    kDriveDeadband = 0.13
    kRotDeadband = 0.16
    kSlowModeMultiplier = 0.30


# ─────────────────────────────────────────────────────────────────────────────
# SWERVE DRIVE
# ─────────────────────────────────────────────────────────────────────────────
class DriveConstants:
    kMaxSpeedMps = 1.5          # capped change to 6 when ready.
    kMaxAngularSpeedRps = 2 * math.pi

    kMagnitudeSlewRate  = 3.0
    kRotationalSlewRate = 2.0
    kSlowModeMultiplier = 0.3

    kTrackWidth = 0.5715
    kWheelBase  = 0.5715

    kFrontLeftChassisOffset  = -math.pi / 2
    kFrontRightChassisOffset = 0.0
    kRearLeftChassisOffset   = math.pi
    kRearRightChassisOffset  = math.pi / 2

    kFrontLeftEncoderOffset  = 0.8546
    kFrontRightEncoderOffset = 0.665
    kRearLeftEncoderOffset   = 0.803
    kRearRightEncoderOffset  = 0.1814

    kFrontLeftDriveId  = 11
    kFrontLeftTurnId   = 10
    kFrontRightDriveId = 13
    kFrontRightTurnId  = 12
    kRearLeftDriveId   = 17
    kRearLeftTurnId    = 18
    kRearRightDriveId  = 7
    kRearRightTurnId   = 2

    kDrivePinionTeeth = 14
    kDriveMotorFreeSpeedRps = 5676.0 / 60
    kWheelDiameterMeters = 0.0762
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    kDriveMotorReduction = (45.0 * 22) / (kDrivePinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (kDriveMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDriveMotorReduction

    kDriveEncoderPositionFactor = kWheelCircumferenceMeters / kDriveMotorReduction
    kDriveEncoderVelocityFactor = kDriveEncoderPositionFactor / 60.0

    kTurnEncoderPositionFactor = 2 * math.pi
    kTurnEncoderVelocityFactor = (2 * math.pi) / 60.0
    kTurnEncoderInverted = True

    kDriveP  = 0.04
    kDriveI  = 0.0
    kDriveD  = 0.0
    kDriveFF = 1.0 / kDriveWheelFreeSpeedRps

    kTurnP  = 2.0
    kTurnI  = 0.0
    kTurnD  = 0.0
    kTurnFF = 0.0

    kDriveMinOutput = -1.0
    kDriveMaxOutput =  1.0
    kTurnMinOutput  = -1.0
    kTurnMaxOutput  =  1.0

    kDriveIdleMode = SparkMaxConfig.IdleMode.kBrake
    kTurnIdleMode  = SparkMaxConfig.IdleMode.kBrake

    kDriveCurrentLimit = 50  # Amps
    kTurnCurrentLimit  = 20  # Amps

    # FIXED: was 241, 158 (pixel coords — way off field).
    # Set to a real field position in meters, or (0, 0) as a safe default.
    # Update these to your actual starting pose before each match.
    k_start_x = 2.0   # meters from blue alliance wall
    k_start_y = 4.0   # meters from bottom wall (mid-field height)


# ─────────────────────────────────────────────────────────────────────────────
# SHOOTER
# ─────────────────────────────────────────────────────────────────────────────
class ShooterConstants:
    kShooterMotor1Id = 1
    kShooterMotor2Id = 3
    kFeederMotorId   = 16

    kShooterP  = 0.5
    kShooterI  = 0.0
    kShooterD  = 0.0
    kShooterKv = 0.12

    kVelocityToleranceRps = 2.0
    kFeederSpeed = 0.6
    kFeederCurrentLimit = 20
    kFlywheelCurrentLimit = 60

    kShooterTable = [
        (1.5,  35.0),
        (2.0,  40.0),
        (2.5,  45.0),
        (3.0,  50.0),
        (3.5,  55.0),
    ]


# ─────────────────────────────────────────────────────────────────────────────
# INTAKE
# ─────────────────────────────────────────────────────────────────────────────
class IntakeConstants:
   # kLeftMotorId  = 15 #Motor is not conneted to a sparkmax.
    kRightMotorId = 9
    #kFuelSensorChannel = 10

    kIntakeSpeed = 0.8
    kReverseSpeed = -0.5
    kCurrentLimit = 30


# ─────────────────────────────────────────────────────────────────────────────
# CLIMBER
# ─────────────────────────────────────────────────────────────────────────────
class ClimberConstants:
    kClimberMotorId = 4
    kClimberMotor2Id = 5
    kClimbSpeed = 0.8
    kDescendSpeed = -0.5
    kCurrentLimit = 40


# ─────────────────────────────────────────────────────────────────────────────
# VISION
# ─────────────────────────────────────────────────────────────────────────────
class VisionConstants:
    kLimelightName = "limelight"
    kCameraToRobotX = 0.0
    kCameraToRobotY = 0.0
    kCameraToRobotZ = 0.5
    kCameraRoll  = 0.0
    kCameraPitch = 25.0
    kCameraYaw   = 0.0

    kSingleTagStdDevs = (4.0, 4.0, 8.0)
    kMultiTagStdDevs  = (0.5, 0.5, 1.0)


# ─────────────────────────────────────────────────────────────────────────────
# AUTONOMOUS  (PathPlanner)
# ─────────────────────────────────────────────────────────────────────────────
class AutoConstants:
    kPxController = 5.0
    kPyController = 5.0

    kPThetaController = 5.0

    kMaxSpeedMps   = 1.5 #change to 3 when ready
    kMaxAccelMps2  = 2.0

    kRobotMassKg = 55.0
    kRobotMOI    = 4.5

    kWheelCOF = 1.0

# ─────────────────────────────────────────────────────────────────────────────
# INTAKE Drawer - this is the system that deploys out our ball getter.
# ─────────────────────────────────────────────────────────────────────────────
class IntakeDrawerConstants:
    kLeftDrawerMotorId  = 15 #Motor is not conneted to a sparkmax.
    kRightDrawerMotorId = 9
    
    kDrawerStartPosition = 0
    kDrawerDeployedPosition = wpimath.units.inchesToMeters(30) 

    kDrawerEffectivePulleyRadius = 2
    k_max_acceleration_meter_per_sec_squared = 0.5
    k_max_velocity_meter_per_second = 0.5
    k_name = "intakeDrawer"

    k_kS_volts = 0 # constant to always add, uses the sign of velocity
    k_kG_volts = 0.88 / 2.0  # 12kg at .2m COM, cuts in half with two motors, goes up with mass and distance, down with efficiency
    k_kV_volt_second_per_radian = 12.05  # stays the same with one or two motors, based on the NEO itself and gear ratio
    k_kA_volt_second_squared_per_meter = 0.10 / 2.0 # cuts in half with 2 motors

    k_gear_ratio = 3 # 9, 12, or 15 gear ratio said victor 1/30/25
                      # we need it seperate for the sim
    k_effective_pulley_diameter = inchesToMeters(1.91) # (https://www.andymark.com/products/25-24-tooth-0-375-in-hex-sprocket) although we're using rev, rev doesn't give a pitch diameter
    k_meters_per_revolution = math.pi * 2 * k_effective_pulley_diameter / k_gear_ratio # 2 because our elevator goes twice as fast as the chain because continuous rigging
    k_mass_kg = lbsToKilograms(4)
    

    k_config = SparkMaxConfig()
    k_config.voltageCompensation(12)            
    k_config.inverted(True)

    k_config.encoder.positionConversionFactor(k_meters_per_revolution)
    k_config.encoder.velocityConversionFactor(k_meters_per_revolution / 60)

    # k_config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.)
    k_config.closedLoop.pid(p=1.0, i=0, d=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IZone(iZone=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IMaxAccum(0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.outputRange(-1, 1)
        
    k_config.softLimit.forwardSoftLimit(kDrawerDeployedPosition)
    k_config.softLimit.reverseSoftLimit(kDrawerStartPosition)

    k_config.softLimit.forwardSoftLimitEnabled(True)
    k_config.softLimit.reverseSoftLimitEnabled(True)

    k_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
    k_config.smartCurrentLimit(40)

    k_follower_config = SparkMaxConfig()
    k_follower_config.follow(kRightDrawerMotorId, invert=True)
    k_follower_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)