import wpilib
import ntcore
import math
import json
from commands2 import Subsystem

class VisionSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.limelight_table = self.inst.getTable("limelight")
        
        # 1. Botpose subscriber
        self.botpose_sub = self.limelight_table.getDoubleArrayTopic("botpose_wpiblue").subscribe([0]*11)
        
        # 2. JSON subscriber for multi-tag distances
        self.json_sub = self.limelight_table.getStringTopic("json").subscribe("")

    def get_field_pose(self):
        """Returns (x, y) in meters."""
        pose = self.botpose_sub.get()
        # If tv (target valid) is 1, we return the pose
        if self.limelight_table.getNumber("tv", 0) > 0 and len(pose) >= 2:
            return pose[0], pose[1]
        return None

    def get_all_tag_distances(self):
        """Parses the JSON dump to find every tag's 3D distance."""
        distances = {}
        json_string = self.json_sub.get()
        
        if not json_string or json_string == "":
            return distances

        try:
            data = json.loads(json_string)
            # Traverse Limelight JSON: Results -> Fiducial
            if "Results" in data and data["Results"] is not None:
                results = data["Results"]
                if "Fiducial" in results:
                    for tag in results["Fiducial"]:
                        tag_id = int(tag.get("fID", -1))
                        # 't6c_ts' is the Camera-Space transform array
                        transform = tag.get("t6c_ts", None)
                        
                        if transform and len(transform) >= 3:
                            # Z-distance (index 2) is the distance from lens to tag
                            z_meters = transform[2]
                            distances[tag_id] = z_meters * 39.3701
        except Exception as e:
            # If there's a parse error, we return what we have (or empty)
            pass
            
        return distances

    def get_distance_to_tag(self) -> float:
        """Helper for the simple log in Robot.py"""
        tags = self.get_all_tag_distances()
        if tags:
            return list(tags.values())[0]
        return -1.0

    def periodic(self):
        # Update Dashboard for easy viewing
        pose = self.get_field_pose()
        if pose:
            wpilib.SmartDashboard.putNumber("Vision/Field_X_In", pose[0] * 39.37)
            wpilib.SmartDashboard.putNumber("Vision/Field_Y_In", pose[1] * 39.37)
        
        tag_dict = self.get_all_tag_distances()
        for tid, dist in tag_dict.items():
            wpilib.SmartDashboard.putNumber(f"Vision/Tag_{tid}_Dist", dist)