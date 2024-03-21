# VisualizePCD

Use Open3D to visualize the pcd

Filtered points of pcd to visualize only a certain section of the PCD

Make sure when saving the depth image from camera: 
  1. It is of uint16 ( 16-bit)
  2. RGB and depth are aligned using rs.align function from Realsense example aligned (https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py)
