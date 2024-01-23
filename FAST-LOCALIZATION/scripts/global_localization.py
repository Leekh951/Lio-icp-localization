#!/usr/bin/env python3
# coding=utf8

import psutil
import os
import copy
import _thread as thread  # 여기를 변경함
import time
import csv
import open3d as o3d
import rospy
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf
import tf.transformations

global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None

# def write_to_csv(fitness_score, timestamp, file_path):
#     with open(file_path, mode='a', newline='') as file:
#         writer = csv.writer(file)
#         writer.writerow([timestamp, fitness_score])
# file_path = '/home/lee/fitness_scores.csv'
        
def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def msg_to_array(pc_msg):
    pc_array = ros_numpy.numpify(pc_msg)
    pc = np.zeros([len(pc_array), 3])
    pc[:, 0] = pc_array['x']
    pc[:, 1] = pc_array['y']
    pc[:, 2] = pc_array['z']
    return pc


def registration_at_scale(pc_scan, pc_map, initial, scale):
    
    #이 안에 DBSCAN 알고리즘 코딩  
    result_icp = o3d.pipelines.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
        1.0 * scale, initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200) #이것도 변경 요망 2000?
    )


    return result_icp.transformation, result_icp.fitness


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    data = np.zeros(len(pc), dtype=[
        ('x', np.float64),
        ('y', np.float64),
        ('z', np.float64),
        ('intensity', np.float64),
    ])
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    msg = ros_numpy.msgify(PointCloud2, data)
    msg.header = header
    publisher.publish(msg)


def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

   
    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

   
    if FOV > 3.14:
        
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &  (global_map_in_base_link[:, 1] < FOV_FAR) #& (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
            & (global_map_in_base_link[:, 1] > - FOV_FAR) & (global_map_in_base_link[:, 0] > -FOV_FAR)
        )
    else:
        
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) & (global_map_in_base_link[:, 1] < FOV_FAR) #& (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        & (global_map_in_base_link[:, 1] > - FOV_FAR) & (global_map_in_base_link[:, 0] > -FOV_FAR)
        )
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

  
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])

    return global_map_in_FOV


def global_localization(pose_estimation):
    global global_map, cur_scan, cur_odom, T_map_to_odom
    start_time = time.process_time()
    # print(global_map, cur_scan, T_map_to_odom)
    rospy.loginfo('Global localization by scan-to-map matching......')

    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()

    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom)
 
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                    scale=1)
    toc = time.time()
    rospy.loginfo('Time: {}'.format(toc - tic))
    rospy.loginfo('')
    end_time = time.process_time()
    process_time = end_time - start_time

    print(f"프로세스 실행 시간: {process_time}초")
    # 현재 파이썬 프로세스의 PID(프로세스 ID)를 얻습니다.
    pid = os.getpid()
    py = psutil.Process(pid)

    # 메모리 사용량을 얻습니다. 이는 bytes 단위로 반환됩니다.
    memory_use = py.memory_info()[0] / 2. ** 30  # bytes를 GB로 변환
    print(f'메모리 사용량: {memory_use} GB')
    #map2odom
    if fitness > LOCALIZATION_TH:
        # T_map_to_odom = np.matmul(transformation, pose_estimation)
        T_map_to_odom = transformation

        #map_to_odom
        map_to_odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        map_to_odom.header.stamp = cur_odom.header.stamp
        map_to_odom.header.frame_id = 'map'
        pub_map_to_odom.publish(map_to_odom)
        print('fitness score:{}'.format(fitness))
        #write_to_csv(fitness, rospy.Time.now().to_sec(), file_path)
        return True
    else:
        rospy.logwarn('Not match!!!!')
        rospy.logwarn('{}'.format(transformation))
        rospy.logwarn('fitness score:{}'.format(fitness))
        #write_to_csv(fitness, rospy.Time.now().to_sec(), file_path)
        return False


def voxel_down_sample(pcd, voxel_size):
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down


def initialize_global_map(pc_msg):
    global global_map

    global_map = o3d.geometry.PointCloud()
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
    rospy.loginfo('Global map received.')


def cb_save_cur_odom(odom_msg):
    global cur_odom
    cur_odom = odom_msg

def visualize_point_cloud(pc):
    # Create a visualizer object
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Add the point cloud to the visualizer
    vis.add_geometry(pc)

    # Run the visualizer
    vis.run()
    vis.destroy_window()

def cb_save_cur_scan(pc_msg):
    global cur_scan
    # 여기서 디비스캔해야함
    pc_msg.header.frame_id = 'camera_init'
    pc_msg.header.stamp = rospy.Time().now()  
    #pub_pc_in_map.publish(pc_msg)
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    pc = msg_to_array(pc_msg)

    # 원점에서 각 포인트까지의 거리 계산
    distances = np.linalg.norm(pc[:, :2], axis=1)

    # 반경 3미터 이내이면서 z가 0.2미터 이상인 포인트들만 필터링
    #filter_mask = (pc[:, 0] < 9) & (pc[:, 0] > -9) & (pc[:, 1] < 7) & (pc[:, 1] > -7) & (pc[:, 2] > -2.1)
    filter_mask = (distances <= 10) & (pc[:, 2] > -2.1)
    pc = np.delete(pc, np.where(filter_mask), axis=0)
    pc_msg = pc2.create_cloud_xyz32(pc_msg.header, pc)
    pub_pc_in_map.publish(pc_msg)
    cur_scan = o3d.geometry.PointCloud()
    cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])
    #visualize_point_cloud(cur_scan)


def thread_localization():
    global T_map_to_odom
    while True:
   
        rospy.sleep(1 / FREQ_LOCALIZATION)
     
        global_localization(T_map_to_odom)


if __name__ == '__main__':
    MAP_VOXEL_SIZE = 0.8
    SCAN_VOXEL_SIZE = 0.4

    # Global localization frequency (HZ)
    FREQ_LOCALIZATION = 1

    # The threshold of global localization,
    # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
    LOCALIZATION_TH = 0.50

    # FOV(rad), modify this according to your LiDAR type
    FOV = 2*3.14

    # The farthest distance(meters) within FOV
    FOV_FAR = 90

    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    # publisher
    pub_pp_pc_in_map = rospy.Publisher('/cur_scan_pp', PointCloud2, queue_size=1)
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)

    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)

    rospy.logwarn('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('/map', PointCloud2))

    while not initialized:
        rospy.logwarn('Waiting for initial pose....')

        pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        initial_pose = pose_to_mat(pose_msg)
        if cur_scan:
            initialized = global_localization(initial_pose)
        else:
            rospy.logwarn('First scan not received!!!!!')

    rospy.loginfo('')
    rospy.loginfo('Initialize successfully!!!!!!')
    rospy.loginfo('')
    thread.start_new_thread(thread_localization, ())

    rospy.spin()
