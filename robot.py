"""
Team 1504 Desperate Penguins - 2026 Robot Code
FRC Game: Rebuilt
"""

import wpilib
import commands2

from src.robot_container import RobotContainer


class Robot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        self.container = RobotContainer()
        self.autonomous_command = None

    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        self.autonomous_command = self.container.get_autonomous_command()
        if self.autonomous_command:
            self.autonomous_command.schedule()

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        if self.autonomous_command:
            self.autonomous_command.cancel()
        self.container.configure_teleop()

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self) -> None:
        pass

    # ─────────────────────────────────────────────────────────────
    # SIMULATION  (only runs when using the sim GUI)
    # ─────────────────────────────────────────────────────────────
    def simulationPeriodic(self) -> None:
        """
        Called every loop in simulation mode.
        Feeds integrated gyro heading, physics model updates, etc.
        """
        self.container.update_sim()


if __name__ == "__main__":
    wpilib.run(Robot)