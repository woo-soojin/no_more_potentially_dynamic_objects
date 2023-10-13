#!/usr/bin/env python

import os

import rospy
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

import argparse
import csv

from cv_bridge import CvBridge
import numpy as np

from tqdm import tqdm

parser = argparse.ArgumentParser(description='A simple kitti publisher')
parser.add_argument('--dir', default='/home/user/Documents/data/KITTI/00', metavar='DIR', help='path to dataset')
parser.add_argument('--velodyne_dir', default='origin', help='path to velodyne bin files')
parser.add_argument('--velodyne_topic_name', default='velodyne_points', help='name of point cloud topic')
parser.add_argument('--velodyne_frame_name', default='velodyne', help='name of lidar frame')
args = parser.parse_args()


def makePointCloud2Msg(points, frame_time, parent_frame, pcd_format):

    ros_dtype = sensor_msgs.PointField.FLOAT32

    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate(pcd_format)]

    # header = std_msgs.Header(frame_id20220616_porter_bin=parent_frame, stamp=rospy.Time.now())
    header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.from_sec(frame_time))
    
    num_field = len(pcd_format)
    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * num_field),
        row_step=(itemsize * num_field * points.shape[0]),
        data=data
    )


if __name__ == '__main__':
    rospy.init_node('KittiPublisher') # don't have blank (space) in the name
    r = rospy.Rate(10)

    bridge = CvBridge()

    # 
    scan_publisher = rospy.Publisher(args.velodyne_topic_name, sensor_msgs.PointCloud2, queue_size=10)

    #
    seqence_dir = args.dir

    scan_dir = os.path.join(seqence_dir, args.velodyne_dir)
    scan_names = os.listdir(scan_dir)
    scan_names_wo_extension = [sn.replace(".bin", "") for sn in scan_names]
    scan_names_wo_extension.sort(key=float)
    
    # parse gt times 
    times = []
    with open(os.path.join(seqence_dir, 'times.txt')) as csvfile:
        times_reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
        for line in times_reader:
            times.append(float(line[0]))
    
    #
    num_frames = scan_names_wo_extension.__len__()
    for frame_idx in tqdm(range(num_frames)):

        # pub velodyne scan 
        scan_path = os.path.join(scan_dir, scan_names_wo_extension[frame_idx] + ".bin")
        xyzi = np.fromfile(scan_path, dtype=np.float32).reshape((-1, 4))
        scan_publisher.publish(makePointCloud2Msg(xyzi, times[frame_idx], args.velodyne_frame_name, ['x', 'y', 'z', 'intensity']))

        #
        r.sleep()
