#!/usr/bin/env python3

import cv_bridge
import numpy as np
import ros_numpy
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import PointCloud2, Image
from ur5_control.srv import FilterWorkspace, FilterWorkspaceRequest

def filter_incoming_cloud(req: FilterWorkspaceRequest):
    """Filter workspace to exclude red block. 
    Checks for red in image captured and removed it from pointcloud.

    Args:
        req (FilterWorkspaceRequest):
            - pointcloud_topic (std_msgs.msg.String): topic name for pointcloud
            - image_topic (std_msgs.msg.String): topic name for image

    Returns:
        None
    """
    # Retrieve the pointcloud and image published
    unfiltered: PointCloud2 = rospy.wait_for_message(req.pointcloud_topic.data, PointCloud2, timeout=rospy.Duration(secs=5))
    image: Image = rospy.wait_for_message(req.image_topic.data, Image, timeout=rospy.Duration(secs=5))
    
    # Convert pointcloud and image to workable numpy arrays
    image_array = bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")
    depth_map = ros_numpy.point_cloud2.pointcloud2_to_array(unfiltered)
    depth_map = np.copy(depth_map)
    
    # Find the regions in red in image and remove from pointcloud
    reds = image_array[:,:,0] > 85
    depth_map[reds] = (np.nan,np.nan,np.nan,np.nan)
    
    # Repackage depth_map to pointcloud and publish to octomap
    out_ros_cloud = ros_numpy.point_cloud2.array_to_pointcloud2(depth_map,stamp = rospy.Time.now(), frame_id=unfiltered.header.frame_id)
    publisher.publish(out_ros_cloud)
    
    return Empty()

if __name__ == "__main__":
    try:
        rospy.init_node('publish_octomap_cloud', anonymous=False)
        bridge = cv_bridge.CvBridge()
        service = rospy.Service('filter_workspace', FilterWorkspace, filter_incoming_cloud)
        publisher = rospy.Publisher('/octomap', PointCloud2, queue_size=10, latch=True)
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass