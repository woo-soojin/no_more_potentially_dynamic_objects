import os
import numpy as np
import open3d as o3d
import pickle
from scipy.spatial.transform import Rotation
import struct
from tqdm import tqdm
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='A simple kitti publisher')
    parser.add_argument('--dir', default='./data/KITTI', metavar='DIR', help='path to dataset')
    parser.add_argument('--bbox_file_name', default='result_4541', metavar='DIR', help='path to dataset')
    args = parser.parse_args()
    DATASET_DIREC_PATH = args.dir
    BIN_DIREC_PATH = os.path.join(DATASET_DIREC_PATH, "origin")
    bin_file_names_list = sorted(os.listdir(BIN_DIREC_PATH))

    input_pkl_path = os.path.join(DATASET_DIREC_PATH, args.bbox_file_name + ".pkl") # write input pkl path
    start_idx = 0

    ground_heights_path = os.path.join(DATASET_DIREC_PATH, "ground_heights.txt")

    with open(input_pkl_path,"rb") as fr:
        results = pickle.load(fr) # load pickle file
        
    for target_idx, result in tqdm(enumerate(results)):
        list_pcd = []
        intensities = []
        size_float = 4
        bin_file_path = os.path.join(BIN_DIREC_PATH, bin_file_names_list[start_idx+target_idx])
        with open (bin_file_path, "rb") as f:
            byte = f.read(size_float*4)
            while byte:
                x,y,z,intensity = struct.unpack("ffff", byte)
                list_pcd.append([x, y, z])
                byte = f.read(size_float*4)
                intensities.append(intensity)
        np_pcd = np.asarray(list_pcd)
        pcd = o3d.geometry.PointCloud()
        v3d = o3d.utility.Vector3dVector

        pcd.points = v3d(np_pcd)

        boxes_lidar = result["boxes_lidar"]
        scores = result["score"]
        projected_idx = []

        ground_heights_bin_file_names = np.loadtxt(ground_heights_path, usecols=[0], dtype=np.object_)
        ground_heights = np.loadtxt(ground_heights_path, usecols=[1])
        gh = ground_heights[np.where(ground_heights_bin_file_names == bin_file_names_list[target_idx])]

        pcd_points = np.asarray(pcd.points)
        for bl_i, box_lidar in enumerate(boxes_lidar):
            x = box_lidar[0]
            y = box_lidar[1]
            z = box_lidar[2]

            dx = box_lidar[3]
            dy = box_lidar[4]
            dz = box_lidar[5]
            
            theta = box_lidar[6]    # target_idx = 2500

            score = scores[bl_i]
            if score > 0.5:
                car_center = [x,y,z]
                car_extent = [dx,dy,dz]
                yaw = theta
                bbox = o3d.geometry.OrientedBoundingBox()
                bbox.center = car_center
                bbox.R = Rotation.from_euler("xyz", [0,0,yaw]).as_matrix()
                bbox.extent = car_extent
                bbox.color = np.array([0,0,255]).reshape((3,1))


                min_bound = bbox.get_min_bound()
                max_bound = bbox.get_max_bound()

                filtered_idx = np.where((min_bound[0] < pcd_points[:,0])&(pcd_points[:,0] < max_bound[0])&\
                                        (min_bound[1] < pcd_points[:,1])&(pcd_points[:,1] < max_bound[1])&\
                                        (min_bound[2] < pcd_points[:,2])&(pcd_points[:,2] < max_bound[2]))
                if filtered_idx[0].shape[0] > 0: ## if found any points corresponding to the object
                    projected_idx += filtered_idx[0].tolist()

                    filtered_points = pcd_points[filtered_idx]
                    # pcd_points[filtered_idx,2] = filtered_points[:,2].min()
                    pcd_points[filtered_idx,2] = gh

        ## Get data from pcd (x, y, z, intensity, ring, time)
        np_xyz = np.asarray(pcd_points, dtype=np.float32).astype(np.float32)
        np_i = np.asarray(intensities, dtype=np.float32).reshape(-1,1).astype(np.float32)

        ## Stack all data
        points_32 = np.hstack((np_xyz, np_i))

        ## Save bin filest[start_idx+target_idx
        bin_direc_name = bin_file_path.split("/")[-2]
        projected_bin_file_path = bin_file_path.replace(bin_direc_name, "projected")                                 
        points_32.tofile(projected_bin_file_path)
