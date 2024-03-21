import open3d as o3d
import matplotlib.pyplot as plt
import cv2
import numpy as np

import pickle
# print("Read Redwo")
# color_raw = o3d.io.read_image("../Downloads/test_rgb.png")
# depth_raw = o3d.io.read_image("../Downloads/test_depth.png")

color_raw = o3d.io.read_image("color_image.png")
depth_raw = o3d.io.read_image("depth_image.png")

rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw,)
print(rgbd_image)

plt.subplot(1, 2, 1)
plt.title('Redwood grayscale image')
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.title('Redwood depth image')
plt.imshow(rgbd_image.depth)    
plt.show()


# with open('rs_camera_params.pkl', 'rb') as file: 
    
# # Call load method to deserialze 
#     myvar = pickle.load(file) 
#     print(myvar)



# fx = 659.9139240696829 
# fy = 663.6905028432622
# cx = 646.3796860837847
# cy = 360.59616797397973

# fx = 306
# fy = 306 
# cx = 118
# cy = 211

    
fx = 380.296875 
fy = 380.02658081
cx = 317.82562256
cy = 242.7481842     

intrinsics = o3d.camera.PinholeCameraIntrinsic(int(cx*2) , int(cy*2), fx, fy, cx, cy)
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    intrinsics)

# pcd_clean = o3d.io.read_point_cloud(pcd)

# X Axis
points = np.asarray(pcd.points)
mask_x_1 = points[:,0] > -0.29
mask_x_2 = points[:,0] < 0.29

# Y Axis
mask_y_1 = points[:,1] > -0.15
mask_y_2 = points[:,1] < 0.15

# Z Axis
mask_z_1 = points[:,2] < 0 # Closer to floor     
mask_z_2 = points[:,2] > 0.475 # Clooser to ceiling

mask_x = np.logical_and(mask_x_1, mask_x_2) # Along table's wide
mask_y = np.logical_and(mask_y_1, mask_y_2) # Along table's longitude
mask_z = np.logical_and(mask_z_1, mask_z_2) # Along table's height
mask = np.logical_and(mask_x, mask_y, mask_z)
pcd.points = o3d.utility.Vector3dVector(points[mask])

# Flip it, otherwise the pointcloud will be upside down
# pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
o3d.visualization.draw_geometries([pcd])

