# physics.py  (project root, next to robot.py)
"""
Team 1504 - Physics simulation
Drives the WPILib sim GUI: moves the robot on the field,
spins the swerve modules, updates the gyro.
"""

import math
import wpilib.simulation
import wpimath.geometry
import wpimath.kinematics
from wpimath.kinematics import ChassisSpeeds

from pyfrc.physics.core import PhysicsInterface


class PhysicsEngine:
    """
    pyfrc calls createPhysics() once, then update() every sim loop.
    The robot's position on the field widget is driven by what you
    return from update().
    """

    def __init__(self, physics_controller: PhysicsInterface, robot) -> None:
        self.physics_controller = physics_controller
        self.robot = robot

    def update_sim(self, now: float, tm_diff: float) -> None:
        if not hasattr(self.robot, "container"):
            return

        drive = self.robot.container.drive
        speeds: ChassisSpeeds = drive.get_chassis_speeds()

        # Integrate speeds into a pose delta
        dx    = speeds.vx    * tm_diff
        dy    = speeds.vy    * tm_diff
        dtheta = speeds.omega * tm_diff

        transform = wpimath.geometry.Transform2d(
            wpimath.geometry.Translation2d(dx, dy),
            wpimath.geometry.Rotation2d(dtheta),
        )
        self.physics_controller.move_robot(transform)
"""
---

## Using the Sim GUI

Once it's open, you need to do three things before the robot does anything:

**1. Enable the robot**
In the "Robot State" panel, click **Teleoperated** then **Enable**.

**2. Attach a joystick**
- In the "Joysticks" panel, drag a joystick type onto port 0 (driver) and port 1 (operator)
- If you don't have a real controller plugged in, use "Keyboard 0" — you can map keys to axes/buttons

**3. Watch the field**
- Open **NetworkTables → SmartDashboard → Field** to see your robot moving
- Or use the **Field2d** widget — right-click the sim window → Add Widget → select Field2d

---

## What Each Sim Class Actually Simulates
```
ShooterIOSim   → FlywheelSim physics (inertia, voltage → RPS)
                 PID + feedforward matches the real TalonFX loop
                 SmartDashboard shows RPS climbing toward target

IntakeIOSim    → Speed pass-through + virtual beam-break sensor
                 Call intake.sim_trigger_fuel() to "inject" a game piece

GyroIOSim      → Integrates chassis omega each loop
                 Pose estimator gets a real heading → robot rotates on field
                 
"""