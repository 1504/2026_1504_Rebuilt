"""
Team 1504 Desperate Penguins - 2026 Robot Code
FRC Game: Rebuilt

Entry point for the robot program.
"""

import wpilib
import commands2

from src.robot_container import RobotContainer


class Robot(commands2.TimedCommandRobot):
    """
    Main robot class. Keeps robotInit lean - all subsystem/command
    setup lives in RobotContainer (6328-style separation of concerns).
    """

    def robotInit(self) -> None:
        self.container = RobotContainer()
        self.autonomous_command = None

    def robotPeriodic(self) -> None:
        # Runs the scheduler every loop (includes subsystem periodics + commands)
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
        # Cancel auto when teleop starts (safety)
        if self.autonomous_command:
            self.autonomous_command.cancel()
        self.container.configure_teleop()

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
