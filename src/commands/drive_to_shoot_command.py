# """
# Team 1504 - DriveToShootCommand
# Limelight TX + pose-derived distance alignment loop.

# Setup geometry:
#   - Intake, Limelight, and shooter all face FORWARD (same side).
#   - Robot drives forward toward the Hub; TX centers the tag; pose gives distance.
#   - isFinished() returns True when both tolerances are held for kSettleCycles loops.
#   - whileTrue() binding means the driver holds a button; releasing cancels and
#     stops the drive. The scheduler restores TeleopDriveCommand automatically.

# Tuning order (do this at practice):
#   1. Set kTargetDistanceM by measuring your ideal shooting spot.
#   2. Tune kAngleP — watch rotation. Oscillating = too high. Sluggish = too low.
#   3. Tune kDistP — watch approach. Overshooting = too high. Creeping = too low.
#   4. Tighten kTxToleranceDeg / kDistToleranceM once motion looks stable.

# ⚠️ rate_limit=False is intentional — slew limiters cause the command to feel
#    mushy and make the settle counter unreliable. The output clamps in constants
#    cap the speed instead.
# """

# import wpilib
# import commands2

# from src.subsystems.drive import DriveSubsystem
# from src.vision.limelight import LimelightVision
# from src.constants import ShootingConstants


# class DriveToShootCommand(commands2.Command):
#     def __init__(self, drive: DriveSubsystem, vision: LimelightVision) -> None:
#         super().__init__()
#         self._drive  = drive
#         self._vision = vision
#         self._settle_count = 0
#         self._timer  = wpilib.Timer()
#         self.addRequirements(drive)

#     def initialize(self) -> None:
#         self._settle_count = 0
#         self._timer.reset()
#         self._timer.start()
#         # Clear stale slew state so motion starts cleanly from zero.
#         self._drive.reset_slew()

#     def execute(self) -> None:
#         tx   = self._vision.get_tx()
#         dist = self._vision.get_distance_to_target()

#         if not self._vision.has_target() or dist is None:
#             # No target — hold still and reset settle counter.
#             # Don't spin or drive blind; wait for the tag to appear.
#             self._drive.drive(0.0, 0.0, 0.0, field_relative=False, rate_limit=False)
#             self._settle_count = 0
#             return

#         # Positive tx = target is to the right → rotate right (positive rot in WPILib)
#         # Negative tx = target is to the left  → rotate left
#         rot = ShootingConstants.kAngleP * tx

#         # Positive dist_err = robot is too far → drive forward (positive x)
#         # Negative dist_err = robot is too close → drive backward
#         dist_err = dist - ShootingConstants.kTargetDistanceM
#         fwd = ShootingConstants.kDistP * dist_err

#         # Clamp to safe speeds
#         rot = max(-ShootingConstants.kMaxAlignSpeed,
#                   min( ShootingConstants.kMaxAlignSpeed, rot))
#         fwd = max(-ShootingConstants.kMaxApproachSpeed,
#                   min( ShootingConstants.kMaxApproachSpeed, fwd))

#         self._drive.drive(fwd, 0.0, rot, field_relative=False, rate_limit=False)

#         # Count consecutive cycles within tolerance for debounce
#         on_target = (
#             abs(tx)       < ShootingConstants.kTxToleranceDeg
#             and abs(dist_err) < ShootingConstants.kDistToleranceM
#         )
#         self._settle_count = (self._settle_count + 1) if on_target else 0

#     def end(self, interrupted: bool) -> None:
#         self._drive.drive(0.0, 0.0, 0.0, field_relative=False, rate_limit=False)
#         self._timer.stop()

#     def isFinished(self) -> bool:
#         # Safety timeout — shoot even if we never fully aligned
#         if self._timer.hasElapsed(ShootingConstants.kAlignTimeoutSec):
#             wpilib.reportWarning(
#                 "[DriveToShootCommand] Alignment timed out — proceeding",
#                 printTrace=False,
#             )
#             return True
#         return self._settle_count >= ShootingConstants.kSettleCycles
