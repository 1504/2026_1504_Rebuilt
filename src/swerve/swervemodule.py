import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
import rev
from rev import SparkMax, SparkMaxConfig, SparkBase
import src.robotcontainer as robotcontainer

kWheelRadius = 0.0508
kEncoderResolution = 4096
#rev neo is 42 for encoder resolutionz: try this out
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau


class SwerveModule:
    def __init__(self, driveMotorChannel: int, turningMotorChannel: int, chassis_angular_offset: float, absolute_encoder_offset: float) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      PWM output for the drive motor.
        :param turningMotorChannel:    PWM output for the turning motor.
        """

        """ Initialize Spark Max motor controllers"""
        self.driving_spark_max: SparkMax = SparkMax(driveMotorChannel, SparkMax.MotorType.kBrushless)
        self.turning_spark_max: SparkMax = SparkMax(turningMotorChannel, SparkMax.MotorType.kBrushless)

        """ Create Config Variables """ 
        self.driving_config = SparkMaxConfig()
        self.turning_config = SparkMaxConfig()

        """ Initialize Spark Max encoders"""
        # Get Encoder Objects from Spark Max
        # driving: relative encoder
        # turning: absolute encoder
        self.driving_encoder = self.driving_spark_max.getEncoder()
        self.turningEncoder = self.turning_spark_max.getAbsoluteEncoder()

        # Apply position and velocity conversion factors for the driving encoder.
        # We want these in radians and radians per second to use with WPILibs swerve APIs
        self.driving_config.encoder.positionConversionFactor(robotcontainer.kDrivingEncoderPositionFactor)
        self.driving_config.encoder.velocityConversionFactor(robotcontainer.kDrivingEncoderVelocityFactor)

        # Apply position and velocity conversion factors for the turning encoder.
        # We want these in radians and radians per second to use with WPILibs swerve APIs
        self.turning_config.absoluteEncoder.positionConversionFactor(robotcontainer.kTurningEncoderPositionFactor)
        self.turning_config.absoluteEncoder.velocityConversionFactor(robotcontainer.kTurningEncoderVelocityFactor)
        self.turning_config.absoluteEncoder.zeroOffset(absolute_encoder_offset)

        # Invert the turning encoder, since the output shaft rotates in the opposite
        # direction of the steering motor in the MAXSwerve Module.
        # self.turningEncoder.setInverted(constants.kTurningEncoderInverted)
        # self.turning_config.encoder(constants.kTurningEncoderInverted)
        self.turning_config.absoluteEncoder.inverted(robotcontainer.kTurningEncoderInverted)

        """ Initialize PID Controllers"""
        # create spark max pid controllers
        # self.driving_pid_controller: rev.SparkPIDController = self.driving_spark_max.getPIDController()
        # self.turning_pid_controller: rev.SparkPIDController = self.turning_spark_max.getPIDController()
        self.driving_pid_controller = self.driving_spark_max.getClosedLoopController()
        self.turning_pid_controller = self.turning_spark_max.getClosedLoopController()

        self.driving_config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        self.turning_config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)


        # Enable PID wrap around for the turning motor. This will allow the PID
        # controller to go through 0 to get to the setpoint i.e. going from 350
        # degrees to 10 degrees will go through 0 rather than the other direction
        #  which is a longer route.
        self.turning_config.closedLoop.positionWrappingEnabled(True)
        self.turning_config.closedLoop.positionWrappingMinInput(robotcontainer.kTurningEncoderPositionPIDMinInput)
        self.turning_config.closedLoop.positionWrappingMaxInput(robotcontainer.kTurningEncoderPositionPIDMaxInput)
        
        
        # Set the PID gains for the driving motor. Note these are example gains, and
        # you may need to tune them for your own robot!
        self.driving_config.closedLoop.P(robotcontainer.kDrivingP)
        self.driving_config.closedLoop.I(robotcontainer.kDrivingI)
        self.driving_config.closedLoop.D(robotcontainer.kDrivingD)
        self.driving_config.closedLoop.velocityFF(robotcontainer.kDrivingFF)
        self.driving_config.closedLoop.outputRange(robotcontainer.kDrivingMinOutput, robotcontainer.kDrivingMaxOutput)

        # Set the PID gains for the turning motor. Note these are example gains, and
        # you may need to tune them for your own robot!
        self.turning_config.closedLoop.P(robotcontainer.kTurningP)
        self.turning_config.closedLoop.I(robotcontainer.kTurningI)
        self.turning_config.closedLoop.D(robotcontainer.kTurningD)
        self.turning_config.closedLoop.velocityFF(robotcontainer.kTurningFF)
        self.turning_config.closedLoop.outputRange(robotcontainer.kTurningMinOutput, robotcontainer.kTurningMaxOutput)

        """ Spark Max Mode Parameters"""
        self.driving_config.setIdleMode(robotcontainer.kDrivingMotorIdleMode)
        self.turning_config.setIdleMode(robotcontainer.kTurningMotorIdleMode)
        self.driving_config.smartCurrentLimit(robotcontainer.kDrivingMotorCurrentLimit)
        self.turning_config.smartCurrentLimit(robotcontainer.kTurningMotorCurrentLimit)

        # Save the SPARK MAX configurations. If a SPARK MAX browns out during
        # operation, it will maintain the above configurations
        # self.driving_spark_max.burnFlash()
        # self.turning_spark_max.burnFlash()

        self.driving_spark_max.configure(self.driving_config,
                                          SparkBase.ResetMode.kResetSafeParameters,
                                          SparkBase.PersistMode.kPersistParameters)
        self.turning_spark_max.configure(self.turning_config,
                                         SparkBase.ResetMode.kResetSafeParameters,
                                         SparkBase.PersistMode.kPersistParameters)

        # Swerve drive parameters
        self.chassis_angular_offset = chassis_angular_offset
        self.desired_state = wpimath.kinematics.SwerveModuleState(0.0, wpimath.geometry.Rotation2d(self.turningEncoder.getPosition())) #diff
        self.driving_encoder.setPosition(0)

    def get_state(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driving_encoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getPosition() - self.chassis_angular_offset),
        )

    def get_position(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driving_encoder.getPosition(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getPosition() - self.chassis_angular_offset),
        )

    def set_desired_state(
        self, desired_state: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desired_state: Desired state with speed and angle.
        """
        # Apply chassis angular offset to the desired state
        corrected_desired_state = wpimath.kinematics.SwerveModuleState(desired_state.speed, desired_state.angle + wpimath.geometry.Rotation2d(self.chassis_angular_offset))
        turning_encoder_position = wpimath.geometry.Rotation2d(self.turningEncoder.getPosition())

        # Optimize the reference state to avoid spinning further than 90 degrees
        corrected_desired_state.optimize(turning_encoder_position)

        # Command driving and turning SPARK MAX toward their respective setpoints
        self.driving_pid_controller.setReference(corrected_desired_state.speed, rev.SparkMax.ControlType.kVelocity)
        self.turning_pid_controller.setReference(corrected_desired_state.angle.radians(), rev.SparkMax.ControlType.kPosition)

        self.desired_state = desired_state

    def reset_encoders(self) -> None:
        self.driving_encoder.setPosition(0)