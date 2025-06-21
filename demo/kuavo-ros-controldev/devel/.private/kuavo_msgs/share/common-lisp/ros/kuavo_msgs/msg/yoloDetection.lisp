; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-msg)


;//! \htmlinclude yoloDetection.msg.html

(cl:defclass <yoloDetection> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type (cl:vector kuavo_msgs-msg:yoloOutputData)
   :initform (cl:make-array 0 :element-type 'kuavo_msgs-msg:yoloOutputData :initial-element (cl:make-instance 'kuavo_msgs-msg:yoloOutputData))))
)

(cl:defclass yoloDetection (<yoloDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <yoloDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'yoloDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-msg:<yoloDetection> is deprecated: use kuavo_msgs-msg:yoloDetection instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <yoloDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:header-val is deprecated.  Use kuavo_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <yoloDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:data-val is deprecated.  Use kuavo_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <yoloDetection>) ostream)
  "Serializes a message object of type '<yoloDetection>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <yoloDetection>) istream)
  "Deserializes a message object of type '<yoloDetection>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kuavo_msgs-msg:yoloOutputData))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<yoloDetection>)))
  "Returns string type for a message object of type '<yoloDetection>"
  "kuavo_msgs/yoloDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'yoloDetection)))
  "Returns string type for a message object of type 'yoloDetection"
  "kuavo_msgs/yoloDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<yoloDetection>)))
  "Returns md5sum for a message object of type '<yoloDetection>"
  "442f2c4723f0369e79b97b3e674354d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'yoloDetection)))
  "Returns md5sum for a message object of type 'yoloDetection"
  "442f2c4723f0369e79b97b3e674354d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<yoloDetection>)))
  "Returns full string definition for message of type '<yoloDetection>"
  (cl:format cl:nil "std_msgs/Header header~%yoloOutputData[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kuavo_msgs/yoloOutputData~%string class_name~%int32 class_id~%float32 confidence~%float32 x_pos~%float32 y_pos~%float32 height~%float32 width~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'yoloDetection)))
  "Returns full string definition for message of type 'yoloDetection"
  (cl:format cl:nil "std_msgs/Header header~%yoloOutputData[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kuavo_msgs/yoloOutputData~%string class_name~%int32 class_id~%float32 confidence~%float32 x_pos~%float32 y_pos~%float32 height~%float32 width~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <yoloDetection>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <yoloDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'yoloDetection
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
