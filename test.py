import cv2
import numpy as np
import open3d as o3d

depth_file = r"C:\Users\adam\CAD Dataset Generator 2018U\Capture\Depth_0010.png"
proj_file = r"C:\Users\adam\CAD Dataset Generator 2018U\Meta\proj_mat.txt"

proj_mat = np.zeros((4, 4))

i = 0

#unity is column major order
for line in open(proj_file, 'r'):
    if i == 4:
        break
    k = 0
    for elem in line.split("\t"):
        elem = elem.strip().rstrip()
        proj_mat[k,i] = float(elem)
        k += 1
    i += 1

inverse_proj_mat = np.linalg.inv(proj_mat)

image = cv2.imread(depth_file, cv2.IMREAD_UNCHANGED)

depth = image.astype(np.float64) / 65534
depth = 2*depth - 1
depth = -proj_mat[3,2] / (proj_mat[2,2] + depth)

x_range = np.arange(-1, 1, 2. / image.shape[1])
y_range = np.arange(-1, 1, 2. / image.shape[0])

#need to invert the y-axis
pixel_map = np.array([[[x_range[i], -y_range[k]] for i in range(image.shape[1])] for k in range(image.shape[0])])

#z_map = -depth[...,np.newaxis]
z_map = np.ones((pixel_map.shape[0], pixel_map.shape[1], 1)) * -1
w_map = np.ones((pixel_map.shape[0], pixel_map.shape[1], 1))

pixel_map = np.concatenate((pixel_map, z_map, w_map), axis=2)
pixel_map = pixel_map[..., np.newaxis]

ray_map = np.matmul(inverse_proj_mat, pixel_map).squeeze()

ray_map /= ray_map[:,:,3,np.newaxis]
ray_map /= ray_map[:,:,2,np.newaxis]

ray_map = ray_map[:,:,:3]
ray_map *= depth[...,np.newaxis]
ray_map = ray_map.reshape((-1, 3))

origin = np.array([[0, 0, 0]])

pcld = o3d.geometry.PointCloud()
pcld.points = o3d.utility.Vector3dVector(np.vstack((ray_map, origin)))

o3d.visualization.draw_geometries([pcld])
o3d.io.write_point_cloud("test.ply", pcld)