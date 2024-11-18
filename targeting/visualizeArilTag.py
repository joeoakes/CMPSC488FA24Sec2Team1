import collections
import numpy
import cv2

# code get from https://github.com/Tinker-Twins/AprilTag/blob/main/scripts/apriltag_video.py

def _draw_pose_box(overlay, camera_params, tag_size, pose, z_sign=1):
    opoints = (
        numpy.array(
            [
                -1,
                -1,
                0,
                1,
                -1,
                0,
                1,
                1,
                0,
                -1,
                1,
                0,
                -1,
                -1,
                -2 * z_sign,
                1,
                -1,
                -2 * z_sign,
                1,
                1,
                -2 * z_sign,
                -1,
                1,
                -2 * z_sign,
            ]
        ).reshape(-1, 1, 3)
        * 0.5
        * tag_size
    )

    edges = numpy.array(
        [0, 1, 1, 2, 2, 3, 3, 0, 0, 4, 1, 5, 2, 6, 3, 7, 4, 5, 5, 6, 6, 7, 7, 4]
    ).reshape(-1, 2)

    fx, fy, cx, cy = camera_params

    K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    tvec = pose[:3, 3]

    dcoeffs = numpy.zeros(5)

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

    ipoints = numpy.round(ipoints).astype(int)

    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)


######################################################################


def _draw_pose_axes(overlay, camera_params, tag_size, pose, center):

    fx, fy, cx, cy = camera_params
    K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    tvec = pose[:3, 3]

    dcoeffs = numpy.zeros(5)

    opoints = (
        numpy.float32([[1, 0, 0], [0, -1, 0], [0, 0, -1]]).reshape(-1, 3) * tag_size
    )

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)
    ipoints = numpy.round(ipoints).astype(int)

    center = numpy.round(center).astype(int)
    center = tuple(center.ravel())

    cv2.line(overlay, center, tuple(ipoints[0].ravel()), (0, 0, 255), 2)
    cv2.line(overlay, center, tuple(ipoints[1].ravel()), (0, 255, 0), 2)
    cv2.line(overlay, center, tuple(ipoints[2].ravel()), (255, 0, 0), 2)


def _annotate_detection(overlay, detection, center):

    text = str(detection.tag_id)
    font = cv2.FONT_HERSHEY_SIMPLEX
    tag_size_px = numpy.sqrt(
        (detection.corners[1][0] - detection.corners[0][0]) ** 2
        + (detection.corners[1][1] - detection.corners[0][1]) ** 2
    )
    font_size = tag_size_px / 22
    text_size = cv2.getTextSize(text, font, font_size, 2)[0]
    tag_center = [detection.center[0], detection.center[1]]
    text_x = int(tag_center[0] - text_size[0] / 2)
    text_y = int(tag_center[1] + text_size[1] / 2)
    cv2.putText(overlay, text, (text_x, text_y), font, font_size, (0, 255, 255), 2)


def visualize(
    overlay,
    detection_results,
    detector,
    camera_params=(3156.71852, 3129.52243, 359.097908, 239.736909),
    tag_size=0.0762,
    vizualization=3,
    verbose=0,
    annotation=0,
):
    """
    Detect AprilTags from image.

    Args:   image [image]: Input image to run detection algorithm on
            detector [detector]: AprilTag Detector object
            camera_params [_camera_params]: Intrinsic parameters for camera (fx, fy, cx, cy)
            tag_size [float]: Physical size of tag in user defined units (m or mm recommended)
            vizualization [int]: 0 - Highlight
                                 1 - Highlight + Boxes
                                 2 - Highlight + Axes
                                 3 - Highlight + Boxes + Axes
            verbose [int]: 0 - Silent
                           1 - Number of detections
                           2 - Detection data
                           3 - Detection and pose data
            annotation [bool]: Render annotated text on detection window
    """

    num_detections = len(detection_results)

    if verbose == 1 or verbose == 2 or verbose == 3:
        print("Detected {} tags\n".format(num_detections))

    numpy.set_printoptions(suppress=True, formatter={"float_kind": "{:0.4f}".format})

    for i, detection in enumerate(detection_results):
        if verbose == 2 or verbose == 3:
            print("Detection {} of {}:".format(i + 1, num_detections))
            print()
            print(detection.tostring(indent=2))

        pose, e0, e1 = detector.detection_pose(detection, camera_params, tag_size)

        if vizualization == 1:
            _draw_pose_box(overlay, camera_params, tag_size, pose)
        elif vizualization == 2:
            _draw_pose_axes(overlay, camera_params, tag_size, pose, detection.center)
        elif vizualization == 3:
            _draw_pose_box(overlay, camera_params, tag_size, pose)
            _draw_pose_axes(overlay, camera_params, tag_size, pose, detection.center)

        if annotation == True:
            _annotate_detection(overlay, detection, tag_size)

        if verbose == 3:
            print(
                detection.tostring(
                    collections.OrderedDict(
                        [("Pose", pose), ("InitError", e0), ("FinalError", e1)]
                    ),
                    indent=2,
                )
            )

            print()

    return overlay
