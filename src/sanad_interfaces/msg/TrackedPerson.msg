# Single tracked person — output of the detector/tracker node.
# All pixel coordinates are in the RGB camera frame.

std_msgs/Header header

# ByteTrack persistent track ID. Stable across frames even through brief occlusion.
int32 track_id

# YOLO detection confidence [0.0, 1.0]
float32 confidence

# Bounding box in image pixels: [x1, y1, x2, y2] top-left → bottom-right
float32[4] bbox

# Bounding box centroid in pixels
float32 centroid_x
float32 centroid_y

# Median depth sampled from the inner 60% of the bbox, in metres.
# 0.0 means no valid depth reading was available.
float32 depth_m

# True when depth_m is a valid reading (> 0 and within sensor range)
bool depth_valid