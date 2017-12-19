import rospy
import tf
import tf.transformations
import geometry_msgs.msg
import numpy as np

def normalize(input):
    return np.array(input / np.linalg.norm(input))

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster_offset_tip')

    data_optitrack_needle_offset = np.genfromtxt('/media/jgschornak/Data/2017-11-29 Needle Tip Tracking/Needle Tip Offset Take 2017-11-29 02.16.21 PM.csv',
                                          delimiter=',', skip_header=7)
    broadcaster = tf.TransformBroadcaster()

    # Find mean needle tip offset
    transforms_needle_base_to_needle_tip = []
    for row in range(1700, 1800):
        position_marker_a = np.array(data_optitrack_needle_offset[row, 17:20]).reshape((1, 3))
        position_marker_b = np.array(data_optitrack_needle_offset[row, 26:29]).reshape((1, 3))
        position_marker_c = np.array(data_optitrack_needle_offset[row, 20:23]).reshape((1, 3))

        position_marker_tip = np.array(data_optitrack_needle_offset[row, 14:17]).reshape((1, 3))

        needle_axis_x = normalize(position_marker_a - position_marker_b).reshape((3, 1))
        needle_axis_y = normalize(position_marker_c - position_marker_b).reshape((3, 1))
        needle_axis_z = normalize(np.cross(needle_axis_x.ravel(), needle_axis_y.ravel())).reshape((3, 1))
        needle_axis_y = normalize(np.cross(needle_axis_z.ravel(), needle_axis_x.ravel())).reshape((3, 1))

        rotation = np.concatenate((needle_axis_x, needle_axis_y, needle_axis_z), axis=1)
        transform_optitrack_to_needle_base = np.concatenate((np.concatenate((rotation, position_marker_b.reshape((3, 1))), axis=1),
                                      np.array([0, 0, 0, 1]).reshape((1, 4))), axis=0)

        transform_optitrack_to_needle_tip = np.concatenate((np.concatenate((rotation, position_marker_tip.reshape((3, 1))), axis=1),
                                           np.array([0, 0, 0, 1]).reshape((1, 4))), axis=0)

        transform_needle_base_to_needle_tip = np.dot(np.linalg.inv(transform_optitrack_to_needle_base), transform_optitrack_to_needle_tip)
        transforms_needle_base_to_needle_tip.append(transform_needle_base_to_needle_tip)
        print('\n')
        print(row)
        print(transform_needle_base_to_needle_tip)

        time = rospy.Time.now()
        broadcaster.sendTransform(transform_optitrack_to_needle_base[0:3,3].reshape((3, 1)), tf.transformations.quaternion_from_matrix(transform_optitrack_to_needle_base), time, "needle_body", "world")
        broadcaster.sendTransform(transform_optitrack_to_needle_tip[0:3,3].reshape((3, 1)), tf.transformations.quaternion_from_matrix(transform_optitrack_to_needle_tip), time, "needle_tip", "world")
        broadcaster.sendTransform(transform_needle_base_to_needle_tip[0:3,3].reshape((3, 1)), tf.transformations.quaternion_from_matrix(np.eye(4)), time, "needle_tip_local", "world")

        rospy.sleep(0.25)
    transform_mean = np.mean(np.array(transforms_needle_base_to_needle_tip), axis=0)
    # print(transforms_needle_base_to_needle_tip)
    print(transform_mean)
    np.savez_compressed("transform_body_to_tip.npz",transform_body_to_tip=transform_mean)

