#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np
import time
import os,json
import open3d as o3d
import numba as jit

class LIDAR():
    def __init__(self):
        rospy.init_node('lidar_processing_node')
        rospy.Subscriber("/velodyne_points", PointCloud2, self.lidar_callback)
        self.bbox_pub = rospy.Publisher("bbox_info", PoseArray, queue_size=10)
        self.marker_pub = rospy.Publisher('bbox_marker_array', MarkerArray, queue_size=10)
        self.pointcloud_pub = rospy.Publisher("clustering_cloud", PointCloud2, queue_size=10)
        self.markerArray = MarkerArray() 
        self.pcd_info = PCD()
        self.n_clusters = 0
        self.cluster_coords = None
        self.is_object = False
        
        rospy.spin()
    
    def numpy_to_pointcloud2(self, points):
        """Convert a numpy array to a sensor_msgs.PointCloud2 message."""
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]
        
        headers = Header()
        # header.frame_id = 'laser'  # Change this to your frame ID
        headers = header
        return pc2.create_cloud(headers, fields, points)    

    @jit    
    def lidar_callback(self, data):
        global header
        header = data.header
        t1 = time.time()
        # last_marker_ids = set()
        gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(gen)).T.astype(np.float32)  # shape should be (3, N)
        # self.lidar_callback(points)   
        
        #numpy to point cloud data
        self.pcd_info.point_np2pcd(points)
        self.pcd_info.get_origin_point(points)
        #voxelize
        self.pcd_info.Voxelize()
        
        height,width = 10, 40
        #ROI filtering
        self.pcd_info.ROI_filtering()
        # numpy to PointCloud2
        pointcloud2_msg = self.numpy_to_pointcloud2(self.pcd_info.pcd.points)
        
        # Publish
        self.pointcloud_pub.publish(pointcloud2_msg)
        # DBscan clustering
        
        if self.pcd_info.pcd.points :
      
            self.n_clusters, self.cluster_coords, self.cluster_sizes = self.pcd_info.o3d_DBscan()
            self.is_object = True
            self.markerArray.markers = []

            pose_array = PoseArray()
            for center in self.cluster_coords:  # 이 부분은 bounding box 중심점들을 반복하는 코드에 따라 다릅니다.
                pose = Pose()
                # pose_array.header.frame_id = "laser"
                pose_array.header = header
                pose.position.x = center[0]
                pose.position.y = center[1]
                pose.position.z = center[2]
                pose.orientation.x = 0  # 이 값들은 bounding box의 회전에 따라 설정해야 함
                pose.orientation.y = 0
                pose.orientation.z = 0
                pose.orientation.w = 1
                pose_array.poses.append(pose)
                
            self.bbox_pub.publish(pose_array) 
            for idx, center in enumerate(self.cluster_coords):
                
                marker = Marker()
                bbox_size = self.cluster_sizes[idx]
                # if (bbox_size[0] < 5 and bbox_size[1] < 5):
                    # sort_id = self.sorted_indices[idx]
                # marker.header.frame_id = "laser"
                marker.header = header
                marker.type = marker.CUBE
                marker.action = marker.ADD
                marker.scale.x = bbox_size[0]  # x축 크기
                marker.scale.y = bbox_size[1]  # y축 크기
                marker.scale.z = bbox_size[2]  # z축 크기
                marker.color.a = 0.5
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = center[0]
                marker.pose.position.y = center[1]
                marker.pose.position.z = center[2]
                marker.id = idx
                marker.lifetime = rospy.Duration(0.3)
                self.markerArray.markers.append(marker)
            # print('object ', idx, ', center : ', center, ', size : ', bbox_size)

        else : 
            self.n_clusters, self.cluster_coords = 0, None
            self.is_object = False
        self.marker_pub.publish(self.markerArray) 
        # last_marker_ids = current_marker_ids    
            # time.sleep(1)
        t2 = time.time()
        print(f'Time to cluster outliers using DBSCAN {t2 - t1}')    
  

   

class PCD:
    def __init__(self):
        self.origin_pcd = o3d.geometry.PointCloud()
        self.pcd =  o3d.geometry.PointCloud()
        self.pcd_np = None
        self.pcd_center = []
    
    def get_origin_point(self,points_np):
        self.pcd_np = points_np.T        
        self.origin_pcd.points = o3d.utility.Vector3dVector(self.pcd_np)

    def point_np2pcd(self, points_np):
        self.pcd_np = points_np.T        
        self.pcd.points = o3d.utility.Vector3dVector(self.pcd_np)
        
               # point_cloud2_msg = create_point_cloud2(rotated_points, colors)
    def Voxelize(self):
        self.pcd = self.pcd.voxel_down_sample(voxel_size=0.15)
        #self.pcd, inliers = self.pcd.remove_radius_outlier(nb_points=20, radius=0.3)
        plane_model, road_inliers = self.pcd.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=100)
        self.pcd = self.pcd.select_by_index(road_inliers, invert=True)
        self.pcd_np = np.asarray(self.pcd.points)
    
    # def Display_pcd(self, pcd):
    #     o3d.visualization.draw_geometries([pcd])
        
    # def Write_pcd(self, file_name):
    #     output_file = file_name
    #     with open(output_file, 'wt', newline='\r\n', encoding='UTF-8') as csvfile:
    #         for line in self.pcd_np:
    #             csvfile.write(str(line) + '\n')
                
    # def channel_filtering(self, channel_select):
    #     channel_list = np.array([[-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15]])
    #     channel_idx = np.where(channel_list == channel_select)
    #     channel_idx = channel_idx[1][0]
        
        # self.pcd_np = self.pcd_np.T[channel_idx::16,:]
        # self.point_np2pcd(self.pcd_np)
       
    def ROI_filtering(self):
        #point shape (3,)
        points = self.pcd_np.T
        # 센서 프레임에서 1m 정사각형 영역 내부의 포인트를 삭제
        mask_x = np.logical_and(points[0, :] > -4, points[0, :] < 2)
        mask_y = np.logical_and(points[1, :] > -1.5, points[1, :] < 1.5)
        mask_z = np.logical_and(points[2, :] > -2.5, points[2, :] < 2.5)
        
        mask_inner_square = np.logical_and(mask_x, mask_y, mask_z)
        points = np.delete(points, np.where(mask_inner_square), axis=1)

        mask_y1 = np.logical_or(points[1, :] < -6, points[1, :] > 6)
        mask_z1 = points[2, :] < -1.4
        
        mask_out_square = np.logical_and( mask_y1, mask_z1)
        points = np.delete(points, np.where(mask_out_square), axis=1)
        
        # 센서 프레임에서 반경 1미터 이내의 포인트를 삭제
        # distances = np.linalg.norm(points, axis=0)
        # points = np.delete(points, np.where(distances < 2.4), axis=1)
        points = np.delete(points,np.where(points[2,:]<-2),axis=1)
        points = np.delete(points,np.where(points[2,:]>0.0),axis=1)
        
        points = np.delete(points,np.where(points[1,:]>12),axis=1)
        points = np.delete(points,np.where(points[1,:]<-12),axis=1)
        
    
        points = np.delete(points,np.where(points[0,:]>30),axis=1)
        points = np.delete(points,np.where(points[0,:]<-20),axis=1)
        ####
  
        ####
        self.pcd_np = points.T
        self.point_np2pcd(points)
        
    
    def get_pcd_center(self,idx):       # point_cloud2_msg = create_point_cloud2(rotated_points, colors)
        # np2pcd
        pcd =  o3d.geometry.PointCloud()
        pcd_center = o3d.geometry.PointCloud()
        pcd_bbox_center = o3d.geometry.PointCloud()
        
        pcd_np = self.pcd_np[idx,:]   
        pcd_np = np.squeeze(pcd_np)  
        pcd.points = o3d.utility.Vector3dVector(pcd_np)  
        pcd_center_np = pcd.get_center() 
        pcd_center_np = pcd_center_np[np.newaxis]
        pcd_center.points = o3d.utility.Vector3dVector(pcd_center_np) 
        
        pcd_bbox = pcd.get_axis_aligned_bounding_box()  
        pcd_bbox_center_np = pcd_bbox.get_center()
        pcd_center_np = pcd.get_center() 
        min_bound = pcd_bbox.min_bound
        max_bound = pcd_bbox.max_bound
        pcd_bbox_extent = max_bound - min_bound

          
        pcd_bbox.color = (1, 0, 0)

        #o3d.visualization.draw_geometries([pcd, pcd_bbox_center, pcd_bbox,pcd_center])

        return pcd_bbox_center_np, pcd_bbox, pcd_bbox_extent
    
    def o3d_DBscan(self):
        """DBscan을 통해 roi영역의 pointcloud를 clustering하여 객체 검출

        Returns:
            n_clusters_(int) : clustering 개수
            pcd_center_sorted(np.array) : clustering 의 중점 좌표 
        """
        # create model and prediction
        if (self.pcd is not None):
            self.labels = self.pcd.cluster_dbscan(eps=0.5, min_points=6)
 
        n_clusters_ = len(set(self.labels)) - (1 if -1 in self.labels else 0)
        n_noise_ = list(self.labels).count(-1)
        self.labels = np.asarray(self.labels)
        
        center_point = []
        pcd_center = []
        pcd_bbox = []
        bbox_sizes = []
        filtered_pcd_center = []
        filtered_bbox_sizes = []
        for label in range(len(set(self.labels[self.labels!=-1]))):
            idx = np.where(self.labels==label)

            pc, pb, bbox_size = self.get_pcd_center(idx)
            pcd_center.append(pc)
            pcd_bbox.append(pb)
            bbox_sizes.append(bbox_size)
        for i in range(len(pcd_center)):
            bbox_size = bbox_sizes[i]
            # Check if cluster dimensions are within desired ranges
            if (
                bbox_size[0] <= 10
                and bbox_size[1] <= 10
                and bbox_size[2] <= 10
                and 1/3 <= bbox_size[0]/bbox_size[1] <= 3
                and bbox_size[1]/bbox_size[0] <= 4
                and 1/10 <= bbox_size[2]/bbox_size[1] <= 10
            ):
                # You can add additional filtering criteria here, e.g., cluster location
                # For example, check if the cluster is within a specific region of interest (ROI)
                if (
                    -30 <= pcd_center[i][0] <= 30
                    and -12 <= pcd_center[i][1] <= 12
                    and -2 <= pcd_center[i][2] <= 0
                ):
                    filtered_pcd_center.append(pcd_center[i])
                    filtered_bbox_sizes.append(bbox_size)
    
        pcd_center = filtered_pcd_center
        bbox_sizes = filtered_bbox_sizes
        center_points_np = np.array(center_point)
        pcd_center_np = np.array(pcd_center)
        if pcd_center_np.size == 0:
            return 0, np.array([]), np.array([])  # Return default values for empty array
        # # 1차원 배열의 경우 2차원으로 변경
        if len(pcd_center_np.shape) == 1:
            pcd_center_np = pcd_center_np[:, np.newaxis]
            
        center_points_np = np.squeeze(center_points_np)
        self.bbox_sizes = np.array(bbox_sizes)
        assert pcd_center_np.shape[1] == 3, f"Unexpected shape: {pcd_center_np.shape}"

        pcd_center_sorted = pcd_center_np
        # (n,3)
        print(pcd_center_np.shape)
        ## sort center points 매우 중요
        if pcd_center_np.shape[0] == 1:
            
            pcd_center_sorted = pcd_center_np
            bbox_sizes_sorted = self.bbox_sizes
        else:
            # center points를 y축 기준으로 정렬하면서, 해당하는 index를 가져옵니다.
            # sorted_indices = np.argsort(pcd_center_np[:, 1])
            # print(sorted_indices)
            pcd_center_sorted = pcd_center_np[pcd_center_np[:, 1].argsort()]
            
            # 위에서 얻은 index를 기준으로 bbox_sizes도 정렬합니다.
            # bbox_sizes_sorted = self.bbox_sizes[sorted_indices]
            bbox_sizes_sorted = self.bbox_sizes[pcd_center_np[:, 1].argsort()]
        # 이후 코드에서 bbox_sizes 대신 bbox_sizes_sorted를 사용하십시오.

        # ...

        return n_clusters_, pcd_center_sorted, bbox_sizes_sorted #, sorted_indices

def main():
    lidar = LIDAR()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
 
        lidar.lidar_callback()

        rate.sleep()    
if __name__ == '__main__':
    main()
