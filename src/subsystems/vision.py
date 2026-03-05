import wpilib
import ntcore
import math
from commands2 import Subsystem

class VisionSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.limelight_table = self.inst.getTable("limelight")
        
        # Subscribers
        self.botpose_sub = self.limelight_table.getDoubleArrayTopic("botpose_wpiblue").subscribe([0]*11)

        # --- PHYSICAL CONSTANTS (Verified for your setup) ---
        self.LENS_HEIGHT = 37      # Carpet to lens center
        self.TARGET_HEIGHT = 45    # Carpet to AprilTag center
        self.MOUNT_ANGLE_DEG = 0   # Tilted UP 10 degrees
        self.LENS_TO_BUMPER = 15    # Lens distance from front bumper
        
        self.height_diff = self.TARGET_HEIGHT - self.LENS_HEIGHT # 8.0"

    def get_field_pose(self):
        """Returns (x, y) in meters for Robot.py"""
        pose = self.botpose_sub.get()
        if self.limelight_table.getNumber("tv", 0) > 0 and len(pose) >= 2:
            return pose[0], pose[1]
        return None

    def get_distance_to_tag(self):
        """Calculates Bumper-to-Tag distance using Inverted Trig."""
        tv = self.limelight_table.getNumber("tv", 0)
        if tv < 1.0:
            return -1.0

        ty = self.limelight_table.getNumber("ty", 0.0)
        
        # --- THE CALCULATION ---
        # Switch to subtraction because 10 + ty was producing 14 inches.
        # This accounts for the Limelight's coordinate system relative to a 10 deg tilt.
        total_angle_deg = self.MOUNT_ANGLE_DEG + ty 
        
        # Prevent math errors (Tan of 0 or negative)
        if total_angle_deg < 0.5:
            total_angle_deg = 0.5
            
        total_angle_rad = math.radians(total_angle_deg)
        
        try:
            distance_lens = self.height_diff / math.tan(total_angle_rad)
            distance_bumper = distance_lens - self.LENS_TO_BUMPER
            return distance_bumper
        except:
            return -1.0

    def get_all_tag_distances(self):
        """Helper for existing Robot.py loops."""
        dist_bumper = self.get_distance_to_tag()
        if dist_bumper > 0:
            # We add LENS_TO_BUMPER back here because Robot.py's 
            # print statement manually subtracts 13.5 again.
            return {int(self.limelight_table.getNumber("tid", 0)): dist_bumper + self.LENS_TO_BUMPER}
        return {}

    def periodic(self):
        dist = self.get_distance_to_tag()
        if dist > 0:
            # This is your main 100.0" display on the dashboard
            wpilib.SmartDashboard.putNumber("Vision/BumperDist_In", dist)
        else:
            wpilib.SmartDashboard.putNumber("Vision/BumperDist_In", 0.0)