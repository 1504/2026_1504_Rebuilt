# """
# Team 1504 - ShooterSubsystem
# Phoenix6 TalonFX velocity PID for flywheel + SparkMax feeder.
# """

# import commands2
# import wpilib
# from wpilib import SmartDashboard

# # Phoenix6
# from phoenix6.hardware import TalonFX
# from phoenix6.controls import VelocityVoltage, NeutralOut
# from phoenix6.configs import TalonFXConfiguration

# # SparkMax
# import rev
# from rev import SparkMax, SparkMaxConfig

# from src.constants import ShooterConstants
# from phoenix6.signals import InvertedValue

# class ShooterSubsystem(commands2.Subsystem):
#     def __init__(self) -> None:
#         super().__init__()

#         # ── Flywheel motors (TalonFX / Kraken or Falcon) ──────────
#         self._motor1 = TalonFX(ShooterConstants.kShooterMotor1Id)
#         self._motor2 = TalonFX(ShooterConstants.kShooterMotor2Id)

#         cfg1 = TalonFXConfiguration()

#         cfg1.slot0.k_p = ShooterConstants.kShooterP
#         cfg1.slot0.k_i = ShooterConstants.kShooterI
#         cfg1.slot0.k_d = ShooterConstants.kShooterD
#         cfg1.slot0.k_v = ShooterConstants.kShooterKv

#         cfg1.current_limits.supply_current_limit_enable = True
#         cfg1.current_limits.supply_current_limit        = ShooterConstants.kFlywheelCurrentLimit
#         cfg1.current_limits.stator_current_limit_enable = True
#         cfg1.current_limits.stator_current_limit        = ShooterConstants.kFlywheelStatorCurrentLimit

#         self._motor1.configurator.apply(cfg1)
        
#         cfg2 = TalonFXConfiguration()

#         cfg2.slot0.k_p = ShooterConstants.kShooterP
#         cfg2.slot0.k_i = ShooterConstants.kShooterI
#         cfg2.slot0.k_d = ShooterConstants.kShooterD
#         cfg2.slot0.k_v = ShooterConstants.kShooterKv

#         cfg2.current_limits.supply_current_limit_enable = True
#         cfg2.current_limits.supply_current_limit        = ShooterConstants.kFlywheelCurrentLimit
#         cfg2.current_limits.stator_current_limit_enable = True
#         cfg2.current_limits.stator_current_limit        = ShooterConstants.kFlywheelStatorCurrentLimit
#         cfg2.motor_output.inverted
#         cfg2.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
#         self._motor2.configurator.apply(cfg2)

#         # Reused every loop to avoid GC pressure
#         self._velocity_request = VelocityVoltage(0).with_slot(0).with_enable_foc(True)
#         # FIXED: stop_shooter now sends NeutralOut instead of VelocityVoltage(0).
#         # VelocityVoltage(0) keeps the PID active and actively resists being
#         # turned by hand (e.g. during inspection or after disable).  NeutralOut
#         # releases the motor so it coasts freely.
#         self._neutral_request  = NeutralOut()

#         # ── Feeder motor (SparkMax NEO 550) ───────────────────────
#         self._feeder = SparkMax(ShooterConstants.kFeederMotorId, SparkMax.MotorType.kBrushless)
#         self._agitator = SparkMax(ShooterConstants.kAgitatorMotorId, SparkMax.MotorType.kBrushless)
#         feeder_cfg = SparkMaxConfig()
#         feeder_cfg.smartCurrentLimit(ShooterConstants.kFeederCurrentLimit)
#         self._feeder.configure(
#             feeder_cfg,
#             rev.ResetMode.kResetSafeParameters,
#             rev.PersistMode.kPersistParameters,
#         )
#         self._agitator.configure(
#             feeder_cfg,
#             rev.ResetMode.kResetSafeParameters,
#             rev.PersistMode.kPersistParameters,
#         )

#         # ── State ─────────────────────────────────────────────────
#         self._target_rps: float = 0.0

#     # ─────────────────────────────────────────────────────────────
#     # PERIODIC
#     # ─────────────────────────────────────────────────────────────
#     def periodic(self) -> None:
#         m1_vel = self._motor1.get_velocity().value
#         m2_vel = self._motor2.get_velocity().value

#         SmartDashboard.putNumber("Shooter/Motor1 RPS",     m1_vel)
#         SmartDashboard.putNumber("Shooter/Motor2 RPS",     m2_vel)
#         SmartDashboard.putNumber("Shooter/Target RPS",     self._target_rps)
#         SmartDashboard.putBoolean("Shooter/AtSpeed",       self.is_at_speed())
#         SmartDashboard.putNumber("Shooter/FeederCurrent",  self._feeder.getOutputCurrent())
#         SmartDashboard.putNumber("Shooter/Motor1Current",  self._motor1.get_supply_current().value)
#         SmartDashboard.putNumber("Shooter/Motor2Current",  self._motor2.get_supply_current().value)

#     # ─────────────────────────────────────────────────────────────
#     # FLYWHEEL CONTROL
#     # ─────────────────────────────────────────────────────────────
#     def set_velocity_rps(self, rps: float) -> None:
#         self._target_rps = rps
#         self._motor1.set_control(self._velocity_request.with_velocity(rps))
#         self._motor2.set_control(self._velocity_request.with_velocity(rps))

#     def set_velocity_from_distance(self, distance_meters: float) -> None:
#         rps = self._interpolate_rps(distance_meters)
#         self.set_velocity_rps(rps)

#     def stop_shooter(self) -> None:
#         self._target_rps = 0.0
#         self._motor1.set_control(self._neutral_request)
#         self._motor2.set_control(self._neutral_request)

#     def is_at_speed(self) -> bool:
#         if self._target_rps == 0.0:
#             return False
#         m1_err = abs(self._motor1.get_velocity().value - self._target_rps)
#         m2_err = abs(self._motor2.get_velocity().value - self._target_rps)
#         return (
#             m1_err < ShooterConstants.kVelocityToleranceRps
#             and m2_err < ShooterConstants.kVelocityToleranceRps
#         )

#     # ─────────────────────────────────────────────────────────────
#     # FEEDER CONTROL
#     # ─────────────────────────────────────────────────────────────
#     def run_feeder(self, speed: float | None = None) -> None:
#         self._feeder.set(speed if speed is not None else ShooterConstants.kFeederSpeed)
#         self._agitator.set(speed if speed is not None else ShooterConstants.kAgitatorSpeed)
        
#     def stop_feeder(self) -> None:
#         self._feeder.set(0.0)
#         self._agitator.set(0.0)

#     def stop_all(self) -> None:
#         self.stop_shooter()
#         self.stop_feeder()

#     # ─────────────────────────────────────────────────────────────
#     # HELPER: interpolate shooter table
#     # ─────────────────────────────────────────────────────────────
#     def _interpolate_rps(self, distance_meters: float) -> float:
#         table = ShooterConstants.kShooterTable
#         if not table:
#             return 40.0
#         if distance_meters <= table[0][0]:
#             return table[0][1]
#         if distance_meters >= table[-1][0]:
#             return table[-1][1]
#         for i in range(len(table) - 1):
#             d0, v0 = table[i]
#             d1, v1 = table[i + 1]
#             if d0 <= distance_meters <= d1:
#                 t = (distance_meters - d0) / (d1 - d0)
#                 return v0 + t * (v1 - v0)
#         return 40.0