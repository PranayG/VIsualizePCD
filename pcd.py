from visualize import RealSenseCamera
import open3d as o3d
import numpy as np
import cv2

def depth2PointCloud(
    depth_map, fx, fy, cx, cy, min_depth=0.2, max_depth=3.0, depth_scale=1000.0
):
    """
    Converts a depth map to a point cloud.

    Args:
        depth_map (numpy.ndarray): The depth map as a 2D numpy array.
        fx (float): The focal length in the x-axis.
        fy (float): The focal length in the y-axis.
        cx (float): The principal point in the x-axis.
        cy (float): The principal point in the y-axis.
        min_depth (float, optional): Minimum valid depth value (default: 0.2 meters).
        max_depth (float, optional): Maximum valid depth value (default: 3.0 meters).
        depth_scale (float, optional): Scaling factor to convert depth values (default: 1000.0).

    Returns:
        numpy.ndarray: A point cloud represented as a numpy array of shape (N, 3) where N is the number of points.

    Note:
        This function applies median filtering and removes outliers from the depth map
        before converting it to a point cloud.

    Example:
        depth_map = load_depth_map()
        fx = 500.0
        fy = 500.0
        cx = 320.0
        cy = 240.0
        point_cloud = depth2PointCloud(depth_map, fx, fy, cx, cy)
    """
    rows, cols = depth_map.shape
    # rows, cols = np.shape(depth_map)

    # Apply the median filter to the depth image
    depth = cv2.medianBlur(depth_map, ksize=3)
    depth = depth.astype(np.float32) / depth_scale

    # Remove the outliers
    idx = np.where(
        ((depth.reshape(-1) <= min_depth) + (depth.reshape(-1) >= max_depth))
    )[0]

    # Create a mesh grid for the depth map
    mesh_x, mesh_y = np.meshgrid(np.arange(cols), np.arange(rows))
    f = (fx + fy) / 2

    # Calculate 3D coordinates (X, Y, Z) from depth values
    x = (mesh_x - cx) * depth / f
    y = (mesh_y - cy) * depth / f
    z = depth
    point_cloud = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    point_cloud = np.delete(point_cloud, idx, axis=0)

    return point_cloud



depth_raw = o3d.io.read_image("depth779.png")
# depth_raw = o3d.io.read_image("../Downloads/test_depth.png")
# print(depth680)
depth_np = np.array(depth_raw)

fx = 659.9139240696829 
fy = 663.6905028432622
cx = 646.3796860837847
cy = 360.59616797397973

# fx = 306
# fy = 306 
# cx = 118
# cy = 211


point_cloud_data = depth2PointCloud(depth_np ,fx , fy , cx , cy )

# print(len(pcd))

# o3d.visualization.draw_geometries([pcd])

pcd = o3d.geometry.PointCloud()

pcd.points = o3d.utility.Vector3dVector(point_cloud_data)

# Visualize the PointCloud object
o3d.visualization.draw_geometries([pcd])


# print(a)