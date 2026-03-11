# 2-D human pose landmarks from MediaPipe in pixel coordinates.
#
# All arrays are parallel and have the same length (one entry per landmark).
#   ids[i]        – MediaPipe landmark index (0-32)
#   u[i], v[i]    – pixel coordinates in the source RGB image
#   visibility[i] – MediaPipe visibility score [0, 1]

std_msgs/Header header

uint32   image_width       # source image width  (pixels)
uint32   image_height      # source image height (pixels)

uint32[]  ids
float32[] u
float32[] v
float32[] visibility
