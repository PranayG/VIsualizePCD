# VisualizePCD

Use Open3D to visualize the pcd

Filtered points of pcd to visualize only a certain section of the PCD

Make sure when saving the depth image from camera: 
  1. Depth is s of uint16 ( 16-bits) since they are more accurate than 8-bit. ( OpenCV by default stores 8-bit images )
  2. To store 16 bit images, while storing cv2.imwrite(name, -1) (-1 is store image in 16 bits)
  3. Use VGA size ( 480, 640)
  4. RGB and depth are aligned using rs.align function from Realsense example aligned (https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py)
