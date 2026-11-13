# a skeleton (the keypoints of a pose), e.g. a person estimated by dnn_pose_estimation
string label          # class of the skeleton, e.g. person
int32 id              # id of the skeleton, the same while it is tracked over the images, or -1
float64 score         # confidence of the pose
Rect rect             # bounding box of the keypoints, x-y is center point
Keypoint[] keypoints  # keypoints in the order of the model, e.g. the 33 landmarks of MediaPipe
