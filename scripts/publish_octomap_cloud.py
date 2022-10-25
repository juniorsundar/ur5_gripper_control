#!/usr/bin/env python3

import ros_numpy
import rospy
import numpy as np
from ur5_control.srv import FilterWorkspace, FilterWorkspaceRequest, FilterWorkspaceResponse
from sensor_msgs.msg import PointCloud2, Image
import rospy
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import cv_bridge

def filter_incoming_cloud(req: FilterWorkspaceRequest):
    """Filter workspace to exclude red block.

    Args:
        req (FilterWorkspaceRequest): _description_

    Returns:
        _type_: _description_
    """
    unfiltered: PointCloud2 = rospy.wait_for_message(req.pointcloud_topic.data, PointCloud2, timeout=rospy.Duration(secs=5))
    image: Image = rospy.wait_for_message(req.image_topic.data, Image, timeout=rospy.Duration(secs=5))
        
    image_array = bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")

    depth_map = ros_numpy.point_cloud2.pointcloud2_to_array(unfiltered)
    depth_map = np.copy(depth_map)
    reds = image_array[:,:,0] > 100
    depth_map[reds] = (np.nan,np.nan,np.nan,np.nan)
    
    
    out_ros_cloud = ros_numpy.point_cloud2.array_to_pointcloud2(depth_map,stamp = rospy.Time.now(), frame_id=unfiltered.header.frame_id)
    
    
    publisher.publish(out_ros_cloud)
    return FilterWorkspaceResponse(filtered = out_ros_cloud)

if __name__ == "__main__":
    try:
        rospy.init_node('publish_octomap_cloud', anonymous=False)
        bridge = cv_bridge.CvBridge()
        service = rospy.Service('filter_workspace', FilterWorkspace, filter_incoming_cloud)
        publisher = rospy.Publisher('/octomap', PointCloud2, queue_size=10, latch=True)
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass