"""
Team 1504 - DriveIO
IO layer for the drive subsystem gyro input.

Why only the gyro here?
  The swerve modules each have their own SwerveModuleIO (see swerve_module_io.py).
  This file covers the gyro so it can be swapped in sim without touching the
  NavX import, which will fail if navx isn't installed in the sim environment.

Real  → NavX AHRS over MXP SPI
Sim   → Integrates chassis angular velocity from module states each loop
"""

from __future__ import annotations
from dataclasses import dataclass
import wpimath.geometry
import wpimath.kinematics


@dataclass
class GyroInputs:
    connected: bool = False
    yaw_deg: float = 0.0
    yaw_rate_dps: float = 0.0
    rotation2d: wpimath.geometry.Rotation2d = None  # type: ignore[assignment]

    def __post_init__(self):
        if self.rotation2d is None:
            self.rotation2d = wpimath.geometry.Rotation2d()


class GyroIO:
    def update_inputs(self, inputs: GyroInputs) -> None:
        pass

    def reset(self) -> None:
        pass


# ─────────────────────────────────────────────────────────────────────────────
# REAL HARDWARE
# ─────────────────────────────────────────────────────────────────────────────
class GyroIONavX(GyroIO):
    def __init__(self) -> None:
        import navx
        self._gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

    def update_inputs(self, inputs: GyroInputs) -> None:
        inputs.connected    = self._gyro.isConnected()
        inputs.yaw_deg      = self._gyro.getAngle()
        inputs.yaw_rate_dps = self._gyro.getRate()
        inputs.rotation2d   = self._gyro.getRotation2d()

    def reset(self) -> None:
        self._gyro.reset()


# ─────────────────────────────────────────────────────────────────────────────
# SIMULATION
# ─────────────────────────────────────────────────────────────────────────────
class GyroIOSim(GyroIO):
    """
    Integrates the omega (turn rate) reported by the swerve kinematics.

    DriveSubsystem calls update_sim_state(chassis_speeds) every loop so this
    class can integrate heading without knowing about individual modules.
    """

    DT = 0.02  # seconds

    def __init__(self) -> None:
        self._yaw_deg: float = 0.0

    def update_sim_state(self, chassis_speeds: wpimath.kinematics.ChassisSpeeds) -> None:
        """Call this from DriveSubsystem.simulationPeriodic() each loop."""
        import math
        omega_rps = chassis_speeds.omega   # rad/s
        self._yaw_deg -= math.degrees(omega_rps * self.DT)  # NavX yaw increases CCW

    def update_inputs(self, inputs: GyroInputs) -> None:
        import math
        inputs.connected  = True
        inputs.yaw_deg    = self._yaw_deg
        inputs.rotation2d = wpimath.geometry.Rotation2d.fromDegrees(self._yaw_deg)

    def reset(self) -> None:
        self._yaw_deg = 0.0
