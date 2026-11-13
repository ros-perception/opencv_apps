# a keypoint of a skeleton, e.g. a joint of a person estimated by dnn_pose_estimation
string name        # name of the keypoint, e.g. nose, left_shoulder
float64 x          # position in the image
float64 y
float64 visibility # probability that the keypoint is visible (not occluded) in the image, 0 to 1
float64 presence   # probability that the keypoint is within the image, 0 to 1
