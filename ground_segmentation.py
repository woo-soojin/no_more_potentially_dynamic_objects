import os
import sys
import open3d as o3d
import numpy as np
# import pypatchworkpp

from tqdm import tqdm
import argparse

cur_dir = os.path.dirname(os.path.abspath(__file__))

parser = argparse.ArgumentParser(description='A simple kitti publisher')
parser.add_argument('--dir', default='./data/KITTI', metavar='DIR', help='path to dataset')
args = parser.parse_args()
DIRECTORY_PATH = os.path.join(args.dir, "origin")

file_names = os.listdir(DIRECTORY_PATH)
file_names = sorted([fn for fn in file_names if ".bin" in fn])
is_vis = False


def read_bin(bin_path):
    scan = np.fromfile(bin_path, dtype=np.float32)
    scan = scan.reshape((-1, 4))

    return scan

if __name__ == "__main__":
    with open(os.path.join(args.dir, "ground_heights.txt"), "w") as f_w:
        for fn_i, file_name in tqdm(enumerate(file_names)):
            input_cloud_filepath = os.path.join(DIRECTORY_PATH, file_name)

            try:
                patchwork_module_path = os.path.join(cur_dir, "../../build/python_wrapper")
                sys.path.insert(0, patchwork_module_path)
                import pypatchworkpp
            except ImportError:
                print("Cannot find pypatchworkpp!")
                exit(1)

            # Patchwork++ initialization
            params = pypatchworkpp.Parameters()
            params.verbose = False

            PatchworkPLUSPLUS = pypatchworkpp.patchworkpp(params)

            # Load point cloud
            pointcloud = read_bin(input_cloud_filepath)

            # Estimate Ground
            PatchworkPLUSPLUS.estimateGround(pointcloud)

            # Get Ground and Nonground
            ground      = PatchworkPLUSPLUS.getGround()
            nonground   = PatchworkPLUSPLUS.getNonground()
            time_taken  = PatchworkPLUSPLUS.getTimeTaken()

            # Get centers and normals for patches
            centers     = PatchworkPLUSPLUS.getCenters()
            normals     = PatchworkPLUSPLUS.getNormals()

            if is_vis:
                print("Origianl Points  #: ", pointcloud.shape[0])
                print("Ground Points    #: ", ground.shape[0])
                print("Nonground Points #: ", nonground.shape[0])
                print("Time Taken : ", time_taken / 1000000, "(sec)")
                print("Press ... \n")
                print("\t H  : help")
                print("\t N  : visualize the surface normals")
                print("\tESC : close the Open3D window")

            # Visualize
            if is_vis:
                vis = o3d.visualization.VisualizerWithKeyCallback()
                vis.create_window(width = 600, height = 400)

            mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()

            ground_o3d = o3d.geometry.PointCloud()
            ground_o3d.points = o3d.utility.Vector3dVector(ground)
            ground_o3d.colors = o3d.utility.Vector3dVector(
                np.array([[0.0, 1.0, 0.0] for _ in range(ground.shape[0])], dtype=float) # RGB
            )

            nonground_o3d = o3d.geometry.PointCloud()
            nonground_o3d.points = o3d.utility.Vector3dVector(nonground)
            nonground_o3d.colors = o3d.utility.Vector3dVector(
                np.array([[1.0, 0.0, 0.0] for _ in range(nonground.shape[0])], dtype=float) #RGB
            )

            centers_o3d = o3d.geometry.PointCloud()
            centers_o3d.points = o3d.utility.Vector3dVector(centers)
            centers_o3d.normals = o3d.utility.Vector3dVector(normals)
            centers_o3d.colors = o3d.utility.Vector3dVector(
                np.array([[1.0, 1.0, 0.0] for _ in range(centers.shape[0])], dtype=float) #RGB
            )
            if is_vis:
                vis.add_geometry(mesh)
                vis.add_geometry(ground_o3d)
                vis.add_geometry(nonground_o3d)
                vis.add_geometry(centers_o3d)
            
            o3d.io.write_point_cloud(".".join(input_cloud_filepath.split(".")[:-1]).replace("origin","pcd")+"_ground.pcd", ground_o3d)
            o3d.io.write_point_cloud(".".join(input_cloud_filepath.split(".")[:-1]).replace("origin","pcd")+"_non_ground.pcd", nonground_o3d)
            added_strings = file_name + " " + str(ground[:,2].mean())
            f_w.write(added_strings)
            if fn_i < len(file_names) - 1:
                f_w.write("\n")

            if is_vis:
                vis.run()
                vis.destroy_window()