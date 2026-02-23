import math
import rev
import wpilib
import wpimath.trajectory
from rev import SparkMax, SparkMaxConfig, SparkFlex, SparkFlexConfig, SparkBase

# from template to learn PID
from rev import ClosedLoopSlot, SparkClosedLoopController
from wpimath.units import inchesToMeters, lbsToKilograms
from wpimath.system.plant import DCMotor

""" DRIVE CONSTANTS """
# Driving parameters - Note that these are not the maximum capable speeds of
# the robot, rather the allowed maximum speeds
kMaxSpeed = 4.8
kMaxAngularSpeed = 2 * math.pi

kDirectionSlewRate = 1.2 # radians per second
kMagnitudeSlewRate = 1.8 # percent per second (1 = 100%)
kRotationalSlewRate = 2.0 # percent per second (1 = 100%)

# Chassis configuration
kTrackWidth = 0.5715 # Distance between centers of right and left wheels on robot METERS
kWheelBase = 0.5715 # Distance between centers of front and back wheels on robot METERS

# Angular offsets of the modules relative to the chassis in radians
kFrontLeftChassisAngularOffset = -math.pi / 2
kFrontRightChassisAngularOffset = 0
kRearLeftChassisAngularOffset = math.pi
kRearRightChassisAngularOffset = math.pi / 2

# SPARK MAX CAN IDs
kRearRightDrivingCanId = 1
kRearRightTurningCanId = 2

kRearLeftDrivingCanId = 3
kRearLeftTurningCanId = 4

kFrontLeftDrivingCanId = 5
kFrontLeftTurningCanId = 6

kFrontRightDrivingCanId = 7
kFrontRightTurningCanId = 8

""" MODULE CONSTANTS """
# Invert the turning encoder, since the output shaft rotates in the opposite direction
# of the steering motor in the MAXSwerve Module
kTurningEncoderInverted = True

# The MaxSwerve module can be configured with one of the three pinion gears: 12T, 13T, or 14T.
# This changes the drive speed of the module ( a pinion gear with more teeth will result in a robot that drives faster)
kDrivingMotorPinionTeeth = 14

# Calculations required for driving motor conversion factors and feed forward
kDrivingMotorFreeSpeedRps = 5676.0 / 60
kWheelDiameter = 0.0762
kWheelCircumference = kWheelDiameter * math.pi

# 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumference) / kDrivingMotorReduction
kDrivingEncoderPositionFactor = (kWheelDiameter * math.pi) / kDrivingMotorReduction # Meters
kDrivingEncoderVelocityFactor = ((kWheelDiameter * math.pi) / kDrivingMotorReduction) / 60.0 # Meters per second

kTurningEncoderPositionFactor = (2 * math.pi) # radians
kTurningEncoderVelocityFactor = (2 * math.pi) / 60.0 # meters per second

kTurningEncoderPositionPIDMinInput = 0
kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor

kDrivingP = 0.04
kDrivingI = 0
kDrivingD = 0
kDrivingFF = (1/kDriveWheelFreeSpeedRps)
kDrivingMinOutput = -1
kDrivingMaxOutput = 1

kTurningP = 2
kTurningI = 0
kTurningD = 0
kTurningFF = 0
kTurningMinOutput = -1
kTurningMaxOutput = 1

kDrivingMotorIdleMode: SparkMax.IdleMode = SparkMaxConfig.IdleMode.kCoast
kTurningMotorIdleMode: SparkMax.IdleMode = SparkMaxConfig.IdleMode.kCoast

kDrivingMotorCurrentLimit = 50 # Amps
kTurningMotorCurrentLimit = 20 # Amps

# Absolute encoder offsets for each module
kRearRightAbsoluteEncoderOffset  = 0.1814
kRearLeftAbsoluteEncoderOffset   = 0.803
kFrontLeftAbsoluteEncoderOffset  = 0.8546
kFrontRightAbsoluteEncoderOffset = 0.665

""" Auto Constants """
kMaxSpeed = 3 # meters per second
kMaxAcceleration = 2 # meters per second squared
kMaxAngularSpeed = 3.142 # radians per second
kMaxAngularAcceleration = 3.142 # radians per second squared

kPXController = 0.5
kPYController = 0.5
kPThetaController = 0.5

kThetaControllerConstraints = wpimath.trajectory.TrapezoidProfile.Constraints(kMaxAngularSpeed, kMaxAngularAcceleration)

""" OI Constants """
kDriverControllerPort = 0
kGadgetControllerPort = 1
kDriveDeadband = 0.08



class ElevatorConstants:
    # all in meters
    # although the tof uses mm, wpilib uses m, and we're using radians according to the wpilib standard
    # therefore, according to the wpilib standard, we will use m
    # 24 tooth sprocket, no. 25 chain

    k_counter_offset = 0
  
    k_CAN_id = 9
    k_follower_CAN_id = 10
    k_name = 'Main Elevator'
    k_max_velocity_meter_per_second = 2
    k_max_acceleration_meter_per_sec_squared = 5
    k_kS_volts = 0 # constant to always add, uses the sign of velocity
    k_kG_volts = 0.88 / 2.0  # 12kg at .2m COM, cuts in half with two motors, goes up with mass and distance, down with efficiency
    k_kV_volt_second_per_radian = 12.05  # stays the same with one or two motors, based on the NEO itself and gear ratio
    k_kA_volt_second_squared_per_meter = 0.10 / 2.0 # cuts in half with 2 motors

    k_gear_ratio = 15 # 9, 12, or 15 gear ratio said victor 1/30/25
                      # we need it seperate for the sim
    k_effective_pulley_diameter = inchesToMeters(1.91) # (https://www.andymark.com/products/25-24-tooth-0-375-in-hex-sprocket) although we're using rev, rev doesn't give a pitch diameter
    k_meters_per_revolution = math.pi * 2 * k_effective_pulley_diameter / k_gear_ratio # 2 because our elevator goes twice as fast as the chain because continuous rigging
    k_mass_kg = lbsToKilograms(19)
    k_plant = DCMotor.NEO(2)

    k_min_height = inchesToMeters(0)
    k_max_height = inchesToMeters(20)
    k_tolerance = 2 / 100 # 2 cm

    k_sim_starting_height = 2

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
        
    k_config.softLimit.forwardSoftLimit(k_max_height)
    k_config.softLimit.reverseSoftLimit(k_min_height)

    k_config.softLimit.forwardSoftLimitEnabled(True)
    k_config.softLimit.reverseSoftLimitEnabled(True)

    k_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
    k_config.smartCurrentLimit(40)

    k_follower_config = SparkMaxConfig()
    k_follower_config.follow(k_CAN_id, invert=False)
    k_follower_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

    kL1RotationDistance = 1.0 #unit is # rotations of the motor

