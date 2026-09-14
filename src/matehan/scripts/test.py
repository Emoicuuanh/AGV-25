a = {
    "waypoints": [
        {
            "name": "Pose_1",
            "position": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        },
        {
            "name": "Pose_2",
            "position": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        },
    ]
}
a["waypoints"][1]["position"]["position"]["x"] = 1
print(a)
