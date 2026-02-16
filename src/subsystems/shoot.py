import rev
import wpilib
import commands2
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
import math
import src.constants as constants
from phoenix6 import CANBus, controls, hardware
import wpilib
from wpilib import Timer, XboxController
from phoenix6 import CANBus, controls, hardware
import rev
from phoenix6.controls import DutyCycleOut, VelocityVoltage, PositionVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
import commands2
from wpilib import Timer

class ShootSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        
        self.feedMotor = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
        self.shootMotor1 = hardware.TalonFX(2)
        self.shootMotor2 = hardware.TalonFX(3)
        
        self.control = controls.DutyCycleOut(0)
        self.gadget_controller = commands2.button.CommandXboxController(0)
        
        #self.gadget_controller.x().whileTrue(self.feed.set(1.0))
        #self.shoot1.set_control(self.control.with_output(self.joystick.getLeftY()))
        #self.shoot2.set_control(self.control.with_output(self.joystick.getLeftY()))
        
    def allShoot(self):
        self.feedMotor.set(-0.4)
        self.shootMotor1.set(-0.4)
        self.shootMotor2.set(0.4)
    
    def feed(self):
        self.feedMotor.set(-0.4)
    
    def stopFeed(self):
        self.feedMotor.set(0.0)
    
    def shoot(self):  
        self.shootMotor1.set(-0.4)
        self.shootMotor2.set(0.4)
        
    def stopShoot(self):  
        self.shootMotor1.set(0.0)
        self.shootMotor2.set(0.0)
    
    def stop(self):
        self.feedMotor.set(0.0)
        self.shootMotor1.set(0.0)
        self.shootMotor2.set(0.0)
        
        
class AllShoot(Command):
    def __init__(self, shoot_subsystem):
        super().__init__()

        self.shoot_subsystem = shoot_subsystem      
    #stopped here
    def initialize(self):
        pass
    def execute(self):
        self.shoot_subsystem.allShoot() 
    def end(self, interrupted):
        self.shoot_subsystem.stop()
        pass
    
class Feed(Command):
    def __init__(self, shoot_subsystem):
        super().__init__()

        self.shoot_subsystem = shoot_subsystem      
    #stopped here
    def initialize(self):
        pass
    def execute(self):
        self.shoot_subsystem.feed() 
    def end(self, interrupted):
        self.shoot_subsystem.stopFeed()
        
class Shoot(Command):
    def __init__(self, shoot_subsystem):
        super().__init__()

        self.shoot_subsystem = shoot_subsystem      
    #stopped here
    def initialize(self):
        pass
    def execute(self):
        self.shoot_subsystem.shoot() 
    def end(self, interrupted):
        self.shoot_subsystem.stopShoot()

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