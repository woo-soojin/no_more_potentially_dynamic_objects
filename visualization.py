import os
import glob
import open3d as o3d
import pickle
import numpy as np

def pkl_visualization():
    visualization_item = list()
    
    pcd_path = "000000.pcd" # TODO change path
    pcd = o3d.io.read_point_cloud(pcd_path)
    visualization_item.append(pcd)

    # read pkl
    pkl_path = "result.pkl" # TODO change path    
    with open(pkl_path, "rb") as fr:
        samples = pickle.load(fr)

    boxes_lidar = samples[0]["boxes_lidar"]
    for box_lidar in boxes_lidar:
        center = np.array([box_lidar[0], box_lidar[1], box_lidar[2]])
        extent = np.array([box_lidar[3], box_lidar[4], box_lidar[5]])
        theta = box_lidar[6]
        rotation = np.array([[np.cos(theta), -np.sin(theta), 0.0],
                            [np.sin(theta), np.cos(theta), 0.0],
                            [0.0, 0.0, 1.0]])
        
        obb = o3d.geometry.OrientedBoundingBox(center=center, extent=extent, R=rotation)
        visualization_item.append(obb)

    o3d.visualization.draw_geometries(visualization_item)

if __name__ == "__main__":
    pkl_visualization()