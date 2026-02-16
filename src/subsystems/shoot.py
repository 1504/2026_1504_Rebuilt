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
        
        self.feed = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
        self.shoot1 = hardware.TalonFX(2)
        self.shoot2 = hardware.TalonFX(3)
        
        self.control = controls.DutyCycleOut(0)
        self.gadget_controller = commands2.button.CommandXboxController(0)
        
        #self.gadget_controller.x().whileTrue(self.feed.set(1.0))
        #self.shoot1.set_control(self.control.with_output(self.joystick.getLeftY()))
        #self.shoot2.set_control(self.control.with_output(self.joystick.getLeftY()))
        
    def basicAhhShoot(self):

        self.feed.set(-0.4)
        self.shoot1.set(-0.4)
        self.shoot2.set(0.4)
    
    def stop(self):
        self.feed.set(0.0)
        self.shoot1.set(0.0)
        self.shoot2.set(0.0)
        
        
class basicAhhShoot(Command):
    def __init__(self, shoot_subsystem):
        super().__init__()

        self.shoot_subsystem = shoot_subsystem      
    #stopped here
    def initialize(self):
        pass
    def execute(self):
        self.shoot_subsystem.basicAhhShoot() 
    def end(self, interrupted):
        self.shoot_subsystem.stop()
    