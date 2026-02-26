import rev
import wpilib
import commands2
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase, SparkBaseConfig
import math
import src.constants as constants
from phoenix6 import CANBus, controls, hardware
from wpilib import Timer, XboxController
from phoenix6 import CANBus, controls, hardware
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import DutyCycleOut, VelocityVoltage, PositionVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
import commands2
from wpilib import Timer

class ShootSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        
        self.feedMotor = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
        self.feeder_encoder = self.feedMotor.getEncoder()
        self.feeder_pid = self.feedMotor.getClosedLoopController()
        
        self.shootMotor1 = hardware.TalonFX(2)
        self.shootMotor2 = hardware.TalonFX(3)
        
        self._configure_shooter_motors()
        self._configure_feeder_motor()
        
        # Velocity control objects
        self.velocity_control = VelocityVoltage(0).with_slot(0)
        
        # Targets
        self.target_velocity = 0.0
        self.shooter_tolerance = 10.0
        self.target_feeder_rpm = 0.0
        self.feeder_tolerance = 50.0 #at speed check
        
        # Feeder speed
        self.feeder_speed = 0.5  # Adjust as needed
        
        # Add SmartDashboard outputs
        self.addChild("Shooter Left", self.shootMotor1)
        self.addChild("Shooter Right", self.shootMotor2)
        
    def allShoot(self):
        self.feedMotor.set(0.2)
        self.shootMotor1.set(0.5)
        self.shootMotor2.set(0.5)
    
    def feed(self):
        self.feedMotor.set(0.2)
    
    def shoot(self):  
        self.shootMotor1.set(0.5)
        self.shootMotor2.set(0.5)
        
    def stop_shooter(self):
        """Stop the shooter motors"""
        self.target_velocity = 0.0
        self.shootMotor1.set(0)
        self.shootMotor2.set(0)
        
    def stop_feeder(self):
        """Stop the feeder motor"""
        self.feedMotor.set(0)
        
    def stop_all(self):
        """Stop all motors"""
        self.stop_shooter()
        self.stop_feeder()
        
    def _configure_shooter_motors(self):
        """Configure the Kraken X60 shooter motors"""
        config = TalonFXConfiguration()
        
        # PID Configuration (Slot 0)
        # IMPORTANT: Tune these values
        config.slot0.k_p = 0.5  # Start with this, tune up or down
        config.slot0.k_i = 0.0   # Usually not needed for velocity
        config.slot0.k_d = 0.0   # Add if needed for stability
        config.slot0.k_v = 0.12  # Feedforward - critical for consistency!
        config.slot0.k_s = 0.1  # Static friction compensation
        
        # Current limits for safety
        config.current_limits.supply_current_limit = 50
        config.current_limits.supply_current_limit_enable = True
        config.current_limits.stator_current_limit = 60
        config.current_limits.stator_current_limit_enable = True
        
        # Motor output settings
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        
        # Apply configuration to left motor(inverted)
        config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.shootMotor1.configurator.apply(config)
        
        # Apply configuration to right motor 
        config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.shootMotor2.configurator.apply(config)
        
    def _configure_feeder_motor(self):
        """Configure the NEO feeder motor with PID control"""
        config = SparkMaxConfig()
        
        # Set idle mode
        #Coast or brake idk help
        config.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        
        # Current limit
        config.smartCurrentLimit(30)
        
        # Invert if needed (test and adjust)
        config.inverted(True)
        
        # Apply configuration
        self.feedMotor.configure(
            config, 
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters
        )
        
    def set_shooter_velocity(self, velocity_rps: float):
        """
        Set the target velocity for shooter wheels
        
        Args:
            velocity_rps: Target velocity in rotations per second
        """
        self.target_velocity = velocity_rps
        
        # Apply velocity control to both motors
        self.shootMotor1.set_control(
            self.velocity_control.with_velocity(velocity_rps+10)                 ###$#
        )
        self.shootMotor2.set_control(
            self.velocity_control.with_velocity(velocity_rps)
        )
        
    def set_feeder_speed(self, speed: float):
        """
        Set the feeder motor speed (open loop)
        
        Args:
            speed: Motor speed from -1.0 to 1.0
        """
        self.feedMotor.set(speed)
        
    def is_at_speed(self) -> bool:
        """
        Check if shooter is at target velocity
        
        Returns:
            True if both shooter motors are within tolerance of target
        """
        if self.target_velocity == 0:
            return False
            
        motor1_velocity = self.shootMotor1.get_velocity().value
        motor2_velocity = self.shootMotor2.get_velocity().value
        
        motor1_at_speed = abs(motor1_velocity - self.target_velocity) < self.shooter_tolerance
        motor2_at_speed = abs(motor2_velocity - self.target_velocity) < self.shooter_tolerance
        
        return motor1_at_speed and motor2_at_speed
    
    def get_shooter_velocity(self) -> tuple[float, float]:
        """
        Get current shooter velocities
        
        Returns:
            Tuple of (motor1_velocity, motor2_velocity) in RPS
        """
        motor1_velocity = self.shootMotor1.get_velocity().value
        motor2_velocity = self.shootMotor2.get_velocity().value
        return (motor1_velocity, motor2_velocity)
    
    def get_feeder_velocity(self) -> float:
        """
        Get current feeder velocity in RPM
        
        Returns:
            Feeder velocity in RPM
        """
        return self.feeder_encoder.getVelocity()
    
    def get_feeder_current(self) -> float:
        """
        Get feeder motor current draw
        Useful for detecting jams or knowing when note is feeding
        
        Returns:
            Current in amps
        """
        return self.feedMotor.getOutputCurrent()
    
    def periodic(self):
        """Update telemetry every robot loop"""
        motor1_vel, motor2_vel = self.get_shooter_velocity()
        
        wpilib.SmartDashboard.putNumber("Shooter/Motor1 Velocity", motor1_vel)
        wpilib.SmartDashboard.putNumber("Shooter/Motor2 Velocity", motor2_vel)
        wpilib.SmartDashboard.putNumber("Shooter/Target Velocity", self.target_velocity)
        wpilib.SmartDashboard.putBoolean("Shooter/At Speed", self.is_at_speed())
        wpilib.SmartDashboard.putNumber("Feeder/Velocity", self.get_feeder_velocity())
        wpilib.SmartDashboard.putNumber("Feeder/Current", self.get_feeder_current())
        
        # Velocity error for tuning
        avg_velocity = (motor1_vel + motor2_vel) / 2
        velocity_error = abs(avg_velocity - self.target_velocity)
        wpilib.SmartDashboard.putNumber("Shooter/Velocity Error", velocity_error)
        
        
class ShootCommand(Command):
    def __init__(self, shoot_subsystem, target_velocity: float=42):
        """
        Command to shoot at a specific velocity
        
        Args:
            shooter: The shooter subsystem
            target_velocity: Target shooter velocity in RPS
        """
        super().__init__()
        self.shooter = shoot_subsystem
        self.target_velocity = target_velocity
        self.addRequirements(shoot_subsystem)
        
    def initialize(self):
        """Start spinning up the shooter"""
        self.shooter.set_shooter_velocity(self.target_velocity)
        self.shooter.stop_feeder()  # Don't feed yet
        
    def execute(self):
        """Feed when shooter is at speed"""
        if self.shooter.is_at_speed():
            self.shooter.set_feeder_speed(0.5)  # Adjust speed as needed
        else:
            self.shooter.stop_feeder()
            
    def end(self, interrupted: bool):
        """Stop everything when command ends"""
        self.shooter.stop_all()
        
    def isFinished(self) -> bool:
        """This command runs until interrupted"""
        return False


class ShootSequenceCommand(Command):
    """
    Shoot command that automatically finishes after shooting
    Useful for autonomous or one-shot operations
    """
    def __init__(self, shoot_subsystem, target_velocity: float, 
                 shoot_duration: float = 0.5):
        """
        Args:
            shooter: The shooter subsystem
            target_velocity: Target shooter velocity in RPS
            shoot_duration: How long to run feeder after at speed (seconds)
        """
        super().__init__()
        self.shooter = shoot_subsystem
        self.target_velocity = target_velocity
        self.shoot_duration = shoot_duration
        self.timer = 0.0
        self.feeding = False
        self.addRequirements(shoot_subsystem)
        
    def initialize(self):
        """Start spinning up the shooter"""
        self.shooter.set_shooter_velocity(self.target_velocity)
        self.shooter.stop_feeder()
        self.timer = 0.0
        self.feeding = False
        
    def execute(self):
        """Feed when at speed, track time"""
        if self.shooter.is_at_speed():
            if not self.feeding:
                self.feeding = True
                self.timer = 0.0
            self.shooter.set_feeder_speed(0.5)
            self.timer += 0.02  # Assumes 50Hz loop (20ms)
        else:
            self.shooter.stop_feeder()
            
    def end(self, interrupted: bool):
        """Stop everything when command ends"""
        self.shooter.stop_all()
        
    def isFinished(self) -> bool:
        """Finish after feeding for specified duration"""
        return self.feeding and self.timer >= self.shoot_duration
       
        
class AllShoot(Command):
    def __init__(self, shoot_subsystem):
        super().__init__()
        self.shoot_subsystem = shoot_subsystem      
    def initialize(self):
        pass
    def execute(self):
        self.shoot_subsystem.allShoot() 
    def end(self, interrupted):
        self.shoot_subsystem.stop_all()
        pass
    
class Feed(Command):
    def __init__(self, shoot_subsystem):
        super().__init__()
        self.shoot_subsystem = shoot_subsystem      
    def initialize(self):
        pass
    def execute(self):
        self.shoot_subsystem.feed() 
    def end(self, interrupted):
        self.shoot_subsystem.stop_feeder()
        
class Shoot(Command):
    def __init__(self, shoot_subsystem):
        super().__init__()
        self.shoot_subsystem = shoot_subsystem      
    def initialize(self):
        pass
    def execute(self):
        self.shoot_subsystem.shoot() 
    def end(self, interrupted):
        self.shoot_subsystem.stop_shooter()
        

# ============================================================================
# TUNING GUIDE
# ============================================================================

"""
STEP-BY-STEP TUNING PROCESS:

1. TUNE SHOOTER PID/FEEDFORWARD
   - Start with the default values in shooter_subsystem.py
   - Use Phoenix Tuner X to plot velocity vs time
   - Adjust k_v (feedforward) first - should get you 90% there
   - Then adjust k_p if needed for faster response
   - Goal: Smooth ramp up to target velocity with minimal oscillation

2. MEASURE DISTANCE-VELOCITY DATA
   - Place robot at known distances from target (1m, 2m, 3m, etc.)
   - Manually test different velocities at each distance
   - Record which velocity gives consistent makes
   - Update the distance_velocity_map in ShooterTable

3. TEST TOLERANCE
   - Default is 2.0 RPS tolerance
   - If shooter is inconsistent, tighten tolerance
   - If feeder waits too long, loosen tolerance
   - Monitor "Shooter/At Speed" in SmartDashboard

4. TUNE FEEDER SPEED
   - Start with 0.5 (50%) feeder speed
   - Too fast: Note jams or shoots before wheels at speed
   - Too slow: Slow shot cycle time
   - Adjust feeder_speed in shooter_subsystem.py

5. VERIFY WITH SHUFFLEBOARD
   - Create a dashboard to monitor:
     * Left/Right shooter velocities
     * Target velocity
     * Distance to target
     * Calculated velocity
     * At speed indicator
   - Watch for velocity consistency during shots

6. ADVANCED: ADD SPIN
   - Run left/right wheels at slightly different speeds
   - Example: left at 100%, right at 95% for spin
   - Test and measure effect on accuracy
"""

# ============================================================================
# TROUBLESHOOTING
# ============================================================================

"""
COMMON ISSUES:

1. Shooter velocity oscillates
   - Reduce k_p, increase k_d
   - Check for mechanical issues (friction, binding)

2. Shots are inconsistent
   - Tighten is_at_speed() tolerance
   - Verify both motors are actually at same speed
   - Check for Note slippage in feeder

3. Feeder runs but shooter doesn't spin
   - Check motor inversions
   - Verify CAN IDs are correct
   - Check motor controllers are powered

4. Shooter spins but wrong speed
   - Verify Kraken X60 sensor is working (Phoenix Tuner)
   - Re-check feedforward values
   - Ensure no mechanical binding

5. Works at some distances but not others
   - Expand your lookup table with more data points
   - Consider using physics calculator instead
   - Account for battery voltage drop (adjust kV)
"""