import rospy
import tf
import tf.transformations
import geometry_msgs.msg
import numpy as np

def normalize(input):
    return np.array(input / np.linalg.norm(input))

def sendTransformHomogeneous(broadcaster, transform, time, child, parent):
    broadcaster.sendTransform(transform[0:3, 3],
                              tf.transformations.quaternion_from_matrix(transform),
                              time,
                              child,
                              parent)
    print(transform)


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster_robot')

    broadcaster = tf.TransformBroadcaster()

    time_current = rospy.Time.now()

    transform_base_to_reg_marker = np.array([[1,0,0,0],
                                             [0,1,0,0],
                                             [0,0,1,0],
                                             [0,0,0,1]])

    transform_reg_marker_to_target = np.array([[1,0,0,1],
                                               [0,1,0,0],
                                               [0,0,1,0],
                                               [0,0,0,1]])

    transform_reg_marker_to_tip = np.array([[1,0,0,0],
                                            [0,1,0,1],
                                            [0,0,1,0],
                                            [0,0,0,1]])

    transform_base_to_pose = np.array([[1,0,0,1],
                                       [0,1,0,0],
                                       [0,0,1,0],
                                       [0,0,0,1]])

    sendTransformHomogeneous(broadcaster, np.eye(4), time_current, "base", "world")
    sendTransformHomogeneous(broadcaster, transform_base_to_pose, time_current, "pose", "base")
    sendTransformHomogeneous(broadcaster, transform_base_to_reg_marker, time_current, "reg_marker", "base")
    sendTransformHomogeneous(broadcaster, transform_reg_marker_to_target, time_current, "target", "reg_marker")
    sendTransformHomogeneous(broadcaster, transform_reg_marker_to_tip, time_current, "tip", "reg_marker")
    rospy.sleep(1)
    rospy.spin()


