# Manual annotation of pointcloud, print output in csv file

import numpy as np
import open3d as o3d

ply_file_path = "./data/avatar_one_component.ply"
out_file_path = "./data/avatar_one_component.csv"

def load_data(path):
    cloud = o3d.io.read_point_cloud(path)
    mesh = o3d.io.read_triangle_mesh(path)
    mesh.compute_vertex_normals()
    return cloud, mesh

def pick_points(pcd, mesh):
    print("")
    print("1) Please pick at least three correspondences using [shift + left click]")
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    print("")
    print("  Note: use '1', '2', or '3' to change the gradient orientation")
    print("  Note: use [shift + +/-] to change the point size")
    vis = o3d.visualization.VisualizerWithEditing()
    #o3d.visualization.draw([pcd, mesh]) # this seems to be the way, but this is not implemented properly https://github.com/isl-org/Open3D/issues/239#issuecomment-843420133
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()

def get_XYZ(points_id, cloud):
    XYZ = np.zeros([len(points_id), 3])
    for i in range(len(points_id)):
        XYZ[i, :] = cloud.points[points_id[i]]
    return XYZ

def print_to_csv(output_path, matrix):
    np.savetxt(output_path, matrix, delimiter=",")

if __name__ == "__main__":
    cloud, mesh = load_data(ply_file_path)
    annotated_points = pick_points(cloud, mesh)
    XYZ = get_XYZ(annotated_points, cloud)
    print_to_csv(out_file_path, XYZ)
