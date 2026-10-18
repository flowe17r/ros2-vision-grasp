"""HSV ranges for the three cubes in the Gazebo world.

Each entry is a list of (h_min, h_max, s_min, s_max, v_min, v_max) tuples.
OpenCV hue is 0-179, so red needs two segments around the wrap point.
Tune these against /perception/segment_debug before trusting detection.
"""

COLOR_PRESETS = {
    'red': [
        (0, 10, 70, 255, 50, 255),
        (170, 179, 70, 255, 50, 255),
    ],
    'green': [
        (40, 85, 60, 255, 40, 255),
    ],
    'blue': [
        (100, 135, 60, 255, 40, 255),
    ],
}
