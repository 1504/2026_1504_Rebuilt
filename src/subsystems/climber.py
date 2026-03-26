# """
# Team 1504 - ClimberSubsystem
# Dual SparkMax motors — one leader, one follower (inverted).
# """

# import time

# import commands2
# from wpilib import SmartDashboard
# import rev
# from rev import SparkMax, SparkMaxConfig

# from src.constants import ClimberConstants



# class ClimberSubsystem(commands2.Subsystem):
#     def __init__(self) -> None:
#         super().__init__()

#         # ── Leader motor ──────────────────────────────────────────
#         self._motor = SparkMax(ClimberConstants.kClimberMotorId, SparkMax.MotorType.kBrushless)
#         leader_cfg = SparkMaxConfig()
#         leader_cfg.smartCurrentLimit(ClimberConstants.kCurrentLimit)
#         leader_cfg.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
#         self._motor.configure(
#             leader_cfg,
#             rev.ResetMode.kResetSafeParameters,
#             rev.PersistMode.kPersistParameters,
#         )

#         # ── Follower motor ────────────────────────────────────────
#         # FIXED: kClimberMotor2Id was defined in constants but the motor was
#         # never instantiated — it had no current limit and was completely
#         # uncontrolled.  Now it follows the leader (inverted so both pull in
#         # the same direction given opposing mount orientations).
#         self._motor2 = SparkMax(ClimberConstants.kClimberMotor2Id, SparkMax.MotorType.kBrushless)
#         follower_cfg = SparkMaxConfig()
#         follower_cfg.smartCurrentLimit(ClimberConstants.kCurrentLimit)
#         follower_cfg.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
#         follower_cfg.follow(ClimberConstants.kClimberMotorId, invert=True)
#         self._motor2.configure(
#             follower_cfg,
#             rev.ResetMode.kResetSafeParameters,
#             rev.PersistMode.kPersistParameters,
#         )

#         self._encoder = self._motor.getEncoder()

#     def periodic(self) -> None:
#         SmartDashboard.putNumber("Climber/Position", self._encoder.getPosition())
#         SmartDashboard.putNumber("Climber/LeaderCurrent", self._motor.getOutputCurrent())
#         SmartDashboard.putNumber("Climber/FollowerCurrent", self._motor2.getOutputCurrent())

#     def climb_up(self) -> None:
#         self._motor.set(ClimberConstants.kClimbSpeed)

#     def climb_down(self) -> None:
#         self._motor.set(ClimberConstants.kDescendSpeed)

#     def stop(self) -> None:
#         self._motor.set(0.0)

#     def get_position(self) -> float:
#         return self._encoder.getPosition()
    
#     def levelone(self):
#         self.climb_up()
#         self.start_time = time.time()
#         self.inTime = time.time() + 0.20
#         if time.time() > self.inTime:
#             self.climb_down()
#             if time.time() > self.inTime + 0.20:
#                 self.stop()
    
#     def leveltwo(self):
#         self.climb_up()
#         self.start_time = time.time()
#         self.inTime = time.time() + 0.20
#         if time.time() > self.inTime:
#             self.climb_down()
#             if time.time() > self.inTime + 0.20:
#                 self.stop()