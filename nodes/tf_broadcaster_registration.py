import rospy
import tf
import tf.transformations
import geometry_msgs.msg
import numpy as np

def normalize(input):
    return np.array(input / np.linalg.norm(input))

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster_registration')

    data_optitrack_registration_checkerboard = np.genfromtxt(
        '/media/jgschornak/Data/2017-12-11 Needle Tip Tracking/Take 2017-12-11 06.55.26 PM.csv', delimiter=',',
        skip_header=7)
    broadcaster = tf.TransformBroadcaster()

    axis_x = np.array([1, 0, 0]).reshape((1, 3))
    axis_y = np.array([0, 1, 0]).reshape((1, 3))
    axis_z = np.array([0, 0, 1]).reshape((1, 3))

    measurements_optitrack = []

    for row in range(0, data_optitrack_registration_checkerboard.shape[0]):
        time = data_optitrack_registration_checkerboard[row, 1]

        position_marker_a = np.array(data_optitrack_registration_checkerboard[row, 8:11]).reshape((1, 3))
        position_marker_b = np.array(data_optitrack_registration_checkerboard[row, 2:5]).reshape((1, 3))
        position_marker_c = np.array(data_optitrack_registration_checkerboard[row, 5:8]).reshape((1, 3))

        checkerboard_axis_x = normalize(position_marker_a - position_marker_b).reshape((3, 1))
        checkerboard_axis_y = normalize(position_marker_c - position_marker_b).reshape((3, 1))
        checkerboard_axis_z = normalize(
            np.cross(checkerboard_axis_x.ravel(), checkerboard_axis_y.ravel())).reshape((3, 1))

        rotation = np.concatenate((checkerboard_axis_x, checkerboard_axis_y, checkerboard_axis_z), axis=1)
        homogeneous = np.concatenate((np.concatenate((rotation, position_marker_b.reshape((3, 1))), axis=1),
                                      np.array([0, 0, 0, 1]).reshape((1, 4))), axis=0)
        measurements_optitrack.append((time, homogeneous))
        # print(homogeneous)

    data_checkerboard = np.load('/media/jgschornak/Data/2017-12-11 Needle Tip Tracking/transforms_registration.npz')
    transforms_checkerboard = data_checkerboard['transforms']
    times_checkerboard = data_checkerboard['times']

    measurements_checkerboard = []
    for index in range(0, transforms_checkerboard.shape[0]):
        measurements_checkerboard.append((times_checkerboard[index], transforms_checkerboard[index]))
        # print(transforms_checkerboard[index])

    transforms_camera_to_optitrack = []
    for measurement_optitrack_to_checkerboard in measurements_optitrack:
        transform_optitrack_to_checkerboard = measurement_optitrack_to_checkerboard[1]
        # for each optitrack measurement, find the checkerboard measurement taken at nearly the same time
        transform_camera_to_checkerboard = min(measurements_checkerboard,
                                               key=lambda tup: abs(tup[0] - measurement_optitrack_to_checkerboard[0]))[1]
        # print(measurement_checkerboard[1])
        transform_checkerboard_to_camera = np.linalg.inv(np.array(transform_camera_to_checkerboard))
        # print(transform_checkerboard_inverse)
        transform_optitrack_to_camera = np.dot(transform_optitrack_to_checkerboard, transform_checkerboard_to_camera)
        # pose_camera = np.dot(measurement_optitrack[1], measurement_checkerboard[1])
        transform_camera_to_optitrack = np.linalg.inv(transform_optitrack_to_camera)




        # rotation_optitrack = tf.transformations.quaternion_from_matrix(transform_world_to_checkerboard_optitrack)
        # position_optitrack = transform_world_to_checkerboard_optitrack[0:3, 3]
        #
        # rotation_camera = tf.transformations.quaternion_from_matrix(transform_world_to_camera)
        # position_camera = transform_world_to_camera[0:3, 3]

        if not np.isnan(transform_optitrack_to_checkerboard).any()\
                and not np.isnan(transform_optitrack_to_camera).any():
            # print("Opt to Che")
            # print(transform_optitrack_to_checkerboard)
            print("Cam to Opt")
            print(transform_camera_to_optitrack)
            print("\n")

            transforms_camera_to_optitrack.append(transform_camera_to_optitrack)

            time_current = rospy.Time.now()
            broadcaster.sendTransform(transform_optitrack_to_checkerboard[0:3, 3],
                                      tf.transformations.quaternion_from_matrix(transform_optitrack_to_checkerboard),
                                      time_current, "checkerboard_origin", "world")
            broadcaster.sendTransform(transform_optitrack_to_camera[0:3, 3],
                                      tf.transformations.quaternion_from_matrix(transform_optitrack_to_camera),
                                      time_current, "stereo_origin", "world")
            rospy.sleep(0.01)

    print(np.mean(transforms_camera_to_optitrack, axis=0))

    np.savez_compressed("transform_camera_to_optitrack.npz", transform_camera_to_optitrack=np.mean(transforms_camera_to_optitrack, axis=0))
