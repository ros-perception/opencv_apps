# an object detected by dnn_object_detection or people_detect, or a code or a marker detected by qrcode_detection,
# barcode_detection and aruco_detection
string label        # class name of the object, QR_CODE, type of the barcode (e.g. EAN_13), or dictionary of the marker (e.g. DICT_4X4_50)
string data         # decoded text of the code, or empty
int32 id            # class id of the object, or id of the marker, or -1
float64 score       # confidence of the detection, 1.0 if not given
Rect rect           # bounding box, x-y is center point
Point2D[] corners   # corners of the code or the marker, empty for the objects
