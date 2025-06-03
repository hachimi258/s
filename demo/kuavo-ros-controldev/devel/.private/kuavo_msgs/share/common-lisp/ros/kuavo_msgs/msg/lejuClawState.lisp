; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-msg)


;//! \htmlinclude lejuClawState.msg.html

(cl:defclass <lejuClawState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (data
    :reader data
    :initarg :data
    :type kuavo_msgs-msg:endEffectorData
    :initform (cl:make-instance 'kuavo_msgs-msg:endEffectorData)))
)

(cl:defclass lejuClawState (<lejuClawState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lejuClawState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lejuClawState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-msg:<lejuClawState> is deprecated: use kuavo_msgs-msg:lejuClawState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <lejuClawState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:header-val is deprecated.  Use kuavo_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <lejuClawState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:state-val is deprecated.  Use kuavo_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <lejuClawState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:data-val is deprecated.  Use kuavo_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<lejuClawState>)))
    "Constants for message type '<lejuClawState>"
  '((:KERROR . -1)
    (:KUNKNOWN . 0)
    (:KMOVING . 1)
    (:KREACHED . 2)
    (:KGRABBED . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'lejuClawState)))
    "Constants for message type 'lejuClawState"
  '((:KERROR . -1)
    (:KUNKNOWN . 0)
    (:KMOVING . 1)
    (:KREACHED . 2)
    (:KGRABBED . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lejuClawState>) ostream)
  "Serializes a message object of type '<lejuClawState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'state))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lejuClawState>) istream)
  "Deserializes a message object of type '<lejuClawState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'state) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'state)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lejuClawState>)))
  "Returns string type for a message object of type '<lejuClawState>"
  "kuavo_msgs/lejuClawState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lejuClawState)))
  "Returns string type for a message object of type 'lejuClawState"
  "kuavo_msgs/lejuClawState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lejuClawState>)))
  "Returns md5sum for a message object of type '<lejuClawState>"
  "71c0eb8f4803a00f5667de51a2f70aac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lejuClawState)))
  "Returns md5sum for a message object of type 'lejuClawState"
  "71c0eb8f4803a00f5667de51a2f70aac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lejuClawState>)))
  "Returns full string definition for message of type '<lejuClawState>"
  (cl:format cl:nil "std_msgs/Header header~%~%int8 kError = -1                ~%int8 kUnknown = 0              ~%int8 kMoving = 1              ~%int8 kReached = 2            ~%int8 kGrabbed = 3         ~%~%int8[] state # 0:; left; 1: right~%kuavo_msgs/endEffectorData data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kuavo_msgs/endEffectorData~%string[] name  ~%float64[] position~%float64[] velocity  ~%float64[] effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lejuClawState)))
  "Returns full string definition for message of type 'lejuClawState"
  (cl:format cl:nil "std_msgs/Header header~%~%int8 kError = -1                ~%int8 kUnknown = 0              ~%int8 kMoving = 1              ~%int8 kReached = 2            ~%int8 kGrabbed = 3         ~%~%int8[] state # 0:; left; 1: right~%kuavo_msgs/endEffectorData data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kuavo_msgs/endEffectorData~%string[] name  ~%float64[] position~%float64[] velocity  ~%float64[] effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lejuClawState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lejuClawState>))
  "Converts a ROS message object to a list"
  (cl:list 'lejuClawState
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':data (data msg))
))
