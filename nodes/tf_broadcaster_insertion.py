import rospy
import tf
import tf.transformations
import geometry_msgs.msg
import numpy as np
import matplotlib.pyplot as plt


def normalize(input):
    return np.array(input / np.linalg.norm(input))

def process_optitrack_axes(data_optitrack,
                           transform_needle_base_to_needle_tip, transform_camera_to_optitrack,
                           col_start_marker_a, col_end_marker_a,
                           col_start_marker_b, col_end_marker_b,
                           col_start_marker_c, col_end_marker_c):
    transforms_optitrack_to_needle_base = []
    transforms_optitrack_to_needle_tip = []
    transforms_camera_to_needle_tip = []
    for row in range(0, len(data_optitrack)):
        position_marker_a = np.array(data_optitrack[row, col_start_marker_a:col_end_marker_a]).reshape((1, 3))
        position_marker_b = np.array(data_optitrack[row, col_start_marker_b:col_end_marker_b]).reshape((1, 3))
        position_marker_c = np.array(data_optitrack[row, col_start_marker_c:col_end_marker_c]).reshape((1, 3))

        needle_axis_x = normalize(position_marker_a - position_marker_b).reshape((3, 1))
        needle_axis_y = normalize(position_marker_c - position_marker_b).reshape((3, 1))
        needle_axis_z = normalize(np.cross(needle_axis_x.ravel(), needle_axis_y.ravel())).reshape((3, 1))
        needle_axis_y = normalize(np.cross(needle_axis_z.ravel(), needle_axis_x.ravel())).reshape((3, 1))

        rotation = np.concatenate((needle_axis_x, needle_axis_y, needle_axis_z), axis=1)
        transform_optitrack_to_needle_base = np.concatenate(
            (np.concatenate((rotation, position_marker_b.reshape((3, 1))), axis=1),
             np.array([0, 0, 0, 1]).reshape((1, 4))), axis=0)

        transform_optitrack_to_needle_tip = np.dot(transform_optitrack_to_needle_base, transform_needle_base_to_needle_tip)
        transform_camera_to_needle_tip = np.dot(transform_camera_to_optitrack, transform_optitrack_to_needle_tip)

        transforms_optitrack_to_needle_base.append(transform_optitrack_to_needle_base)
        transforms_optitrack_to_needle_tip.append(transform_optitrack_to_needle_tip)
        transforms_camera_to_needle_tip.append(transform_camera_to_needle_tip)

    return np.array(transforms_optitrack_to_needle_base),\
           np.array(transforms_optitrack_to_needle_tip),\
           np.array(transforms_camera_to_needle_tip)

def process_camera_axes(data_camera):
    transforms_camera_to_needle_tip_measured = []
    for row in range(0,len(data_camera)):
        transform_camera_to_needle_tip_measured = np.concatenate(
            (np.concatenate((np.eye(3), data_camera[row,1:].reshape((3, 1))), axis=1),
             np.array([0, 0, 0, 1]).reshape((1, 4))), axis=0)
        transforms_camera_to_needle_tip_measured.append(transform_camera_to_needle_tip_measured)
    return np.array(transforms_camera_to_needle_tip_measured)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster_insertion')

    # data_optitrack_needle_insertion = np.genfromtxt('/media/jgschornak/Data/2017-11-29 Needle Tip Tracking/Trial 2 Take 2017-11-29 02.36.07 PM.csv',
    #                                       delimiter=',', skip_header=7)
    # data_optitrack_needle_insertion = np.genfromtxt('/media/jgschornak/Data/2017-11-29 Needle Tip Tracking/Trial 3 Take 2017-11-29 02.44.31 PM.csv',
    #                                       delimiter=',', skip_header=7)
    # data_optitrack_needle_insertion = np.genfromtxt('/media/jgschornak/Data/2017-11-29 Needle Tip Tracking/Trial 4 Take 2017-11-29 02.46.11 PM.csv',
    #                                       delimiter=',', skip_header=7)
    data_optitrack_needle_insertion = np.genfromtxt('/media/jgschornak/Data/2017-12-11 Needle Tip Tracking/Trial 1 Take 2017-12-11 06.59.31 PM.csv',
                                          delimiter=',', skip_header=7)

    broadcaster = tf.TransformBroadcaster()
    transform_needle_base_to_needle_tip = np.load("transform_body_to_tip.npz")['transform_body_to_tip']
    print("NB to NT:")
    print(transform_needle_base_to_needle_tip)

    trajectory_camera = np.genfromtxt('/media/jgschornak/Data/2017-12-11 Needle Tip Tracking/Trial 1 trajectory.csv', delimiter=',')

    data_registration = np.load('transform_camera_to_world.npz')
    transform_camera_to_optitrack = data_registration['transform_camera_to_world']

    print("Transform_camera_to_optitrack")
    print(transform_camera_to_optitrack)
    print("transform_needle_base_to_needle_tip")
    print(transform_needle_base_to_needle_tip)

    transforms_optitrack_to_needle_base,\
    transforms_optitrack_to_needle_tip,\
    transforms_camera_to_needle_tip = process_optitrack_axes(data_optitrack_needle_insertion,
                                                             transform_needle_base_to_needle_tip,
                                                             transform_camera_to_optitrack,
                                                             8, 11,
                                                             2, 5,
                                                             5, 8)

    transforms_camera_to_needle_tip_measured = process_camera_axes(trajectory_camera)


    for row in range(0, min(len(transforms_camera_to_needle_tip), len(transforms_camera_to_needle_tip_measured))):
        if not np.isnan(transforms_camera_to_needle_tip[row]).any() and not np.isnan(transforms_camera_to_needle_tip_measured[row]).any():
            time = rospy.Time.now()
            print("Optitrack")
            print(transforms_camera_to_needle_tip[row])
            print("Camera")
            print(transforms_camera_to_needle_tip_measured[row])
            broadcaster.sendTransform(transforms_camera_to_needle_tip[row][0:3,3].reshape((3, 1)), tf.transformations.quaternion_from_matrix(transforms_camera_to_needle_tip[row]), time, "needle_tip", "world")
            broadcaster.sendTransform(transforms_camera_to_needle_tip_measured[row][0:3,3].reshape((3, 1)), tf.transformations.quaternion_from_matrix(np.eye(4)), time, "needle_tip_meas", "world")
            rospy.sleep(0.1)
    print("\n\n\n")



