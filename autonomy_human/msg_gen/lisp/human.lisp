; Auto-generated. Do not edit!


(cl:in-package autonomy_human-msg)


;//! \htmlinclude human.msg.html

(cl:defclass <human> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (numFaces
    :reader numFaces
    :initarg :numFaces
    :type cl:integer
    :initform 0)
   (faceROI
    :reader faceROI
    :initarg :faceROI
    :type sensor_msgs-msg:RegionOfInterest
    :initform (cl:make-instance 'sensor_msgs-msg:RegionOfInterest))
   (faceScore
    :reader faceScore
    :initarg :faceScore
    :type cl:integer
    :initform 0)
   (flowScore
    :reader flowScore
    :initarg :flowScore
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass human (<human>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <human>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'human)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autonomy_human-msg:<human> is deprecated: use autonomy_human-msg:human instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <human>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomy_human-msg:header-val is deprecated.  Use autonomy_human-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'numFaces-val :lambda-list '(m))
(cl:defmethod numFaces-val ((m <human>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomy_human-msg:numFaces-val is deprecated.  Use autonomy_human-msg:numFaces instead.")
  (numFaces m))

(cl:ensure-generic-function 'faceROI-val :lambda-list '(m))
(cl:defmethod faceROI-val ((m <human>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomy_human-msg:faceROI-val is deprecated.  Use autonomy_human-msg:faceROI instead.")
  (faceROI m))

(cl:ensure-generic-function 'faceScore-val :lambda-list '(m))
(cl:defmethod faceScore-val ((m <human>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomy_human-msg:faceScore-val is deprecated.  Use autonomy_human-msg:faceScore instead.")
  (faceScore m))

(cl:ensure-generic-function 'flowScore-val :lambda-list '(m))
(cl:defmethod flowScore-val ((m <human>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomy_human-msg:flowScore-val is deprecated.  Use autonomy_human-msg:flowScore instead.")
  (flowScore m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <human>) ostream)
  "Serializes a message object of type '<human>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numFaces)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'numFaces)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'numFaces)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'numFaces)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'faceROI) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'faceScore)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'faceScore)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'faceScore)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'faceScore)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'flowScore))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <human>) istream)
  "Deserializes a message object of type '<human>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numFaces)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'numFaces)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'numFaces)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'numFaces)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'faceROI) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'faceScore)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'faceScore)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'faceScore)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'faceScore)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'flowScore) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'flowScore)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<human>)))
  "Returns string type for a message object of type '<human>"
  "autonomy_human/human")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'human)))
  "Returns string type for a message object of type 'human"
  "autonomy_human/human")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<human>)))
  "Returns md5sum for a message object of type '<human>"
  "b1101ef84b709757da9ca210281612a1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'human)))
  "Returns md5sum for a message object of type 'human"
  "b1101ef84b709757da9ca210281612a1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<human>)))
  "Returns full string definition for message of type '<human>"
  (cl:format cl:nil "Header header~%~%uint32 numFaces~%sensor_msgs/RegionOfInterest faceROI~%uint32 faceScore~%float32[2] flowScore~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'human)))
  "Returns full string definition for message of type 'human"
  (cl:format cl:nil "Header header~%~%uint32 numFaces~%sensor_msgs/RegionOfInterest faceROI~%uint32 faceScore~%float32[2] flowScore~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <human>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'faceROI))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'flowScore) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <human>))
  "Converts a ROS message object to a list"
  (cl:list 'human
    (cl:cons ':header (header msg))
    (cl:cons ':numFaces (numFaces msg))
    (cl:cons ':faceROI (faceROI msg))
    (cl:cons ':faceScore (faceScore msg))
    (cl:cons ':flowScore (flowScore msg))
))
