import wpilib
import ntcore
import math
from commands2 import Subsystem

class VisionSubsystem(Subsystem):
    """
    Subsystem for 2026 REBUILT AprilTag tracking.
    Calculates distance, logs to SmartDashboard, and records to DataLog.
    """
    def __init__(self):
        super().__init__()
        
        # 1. Initialize NetworkTables
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.limelight_table = self.inst.getTable("limelight")
        
        # 2. Start Data Logging (For AdvantageScope)
        wpilib.DataLogManager.start()
        
        # 3. Configuration (Inches)
        self.LIMELIGHT_HEIGHT = 18.0   # TBD: Update after build
        self.MOUNT_ANGLE_DEG = -22.0   # 22 degrees DOWN
        
        # 4. 2026 REBUILT Tag Heights (Center of Tag)
        self.TAG_HEIGHT_MAP = {
            1: 27.0, 2: 27.0,   # Tower Rungs
            3: 63.0, 4: 63.0,   # Upper Targets
            5: 12.0, 6: 12.0    # Floor Depot
        }

    def get_distance_to_tag(self) -> float:
        """Calculates horizontal distance. Returns -1.0 if no target."""
        tv = self.limelight_table.getNumber("tv", 0)
        tid = int(self.limelight_table.getNumber("tid", -1))
        ty = self.limelight_table.getNumber("ty", 0.0)

        if tv < 1.0 or tid not in self.TAG_HEIGHT_MAP:
            return -1.0

        target_height = self.TAG_HEIGHT_MAP[tid]
        angle_to_target_rad = math.radians(self.MOUNT_ANGLE_DEG + ty)
        height_diff = target_height - self.LIMELIGHT_HEIGHT
        
        try:
            return abs(height_diff / math.tan(angle_to_target_rad))
        except ZeroDivisionError:
            return -1.0

    def periodic(self):
        """Updates SmartDashboard every 20ms."""
        dist = self.get_distance_to_tag()
        wpilib.SmartDashboard.putNumber("Vision/Distance_In", dist)
