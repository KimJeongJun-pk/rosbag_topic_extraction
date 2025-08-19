import os
import rosbag
import cv2
import numpy as np
import open3d as o3d
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import rospy

output_folder = '/home/kjj/rosbag/rosbag_250804/img_pcd_250804'
camera_folder = os.path.join(output_folder, 'camera_images')
lidar_folder = os.path.join(output_folder, 'lidar_points')

if not os.path.exists(camera_folder):
   os.makedirs(camera_folder)
if not os.path.exists(lidar_folder):
    os.makedirs(lidar_folder)

bag_file = '/home/kjj/rosbag/rosbag_250804/rosbag_250804.bag'

bridge = CvBridge()

bag = rosbag.Bag(bag_file)

camera_topic = '/camera/color/image_raw/time_stamp'
lidar_topic = '/ilidar/points/time_stamp'

camera_images = []
lidar_points = []

camera_timestamps = []
lidar_timestamps = []

for topic, msg, t in bag.read_messages(topics=[camera_topic, lidar_topic]):
    if topic == camera_topic:
        try :
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
            camera_images.append(cv_image)
            camera_timestamps.append(t.to_sec())
            
        except Exception as e:
            print(f"image conversion failed : {e}")

    if topic == lidar_topic:
        try:
            points = pc2.read_points(msg, skip_nans = True, field_names=("x","y","z"))
            lidar_points.append(list(points))
            lidar_timestamps.append(t.to_sec())
            
        except Exception as e:
            print(f"Point cloud processing failed : {e}")
            
bag.close()

synced_camera_images = []
synced_lidar_points = []

for i, cam_time in enumerate(camera_timestamps):
    for j, lidar_time in enumerate(lidar_timestamps):
        if abs(cam_time - lidar_time) <= 0.01:
            synced_camera_images.append(camera_images[i])
            synced_lidar_points.append(lidar_points[j])
            break

for idx, (img, point_cloud) in enumerate(zip(synced_camera_images, synced_lidar_points)):
    
    image_filename = os.path.join(camera_folder, f'image_{idx:04d}.png')
    cv2.imwrite(image_filename, img)
    
    lidar_filename = os.path.join(lidar_folder, f'lidar_{idx:04d}.pcd')
    
    cloud = o3d.geometry.PointCloud()
    points_array = np.array(point_cloud)
    cloud.points = o3d.utility.Vector3dVector(points_array)
    
    o3d.io.write_point_cloud(lidar_filename, cloud)

print(f"카메라 이미지가 {camera_folder}에 저장되었습니다.")
print(f"라이다 포인트클라우드가 {lidar_folder}에 저장되었습니다.")
