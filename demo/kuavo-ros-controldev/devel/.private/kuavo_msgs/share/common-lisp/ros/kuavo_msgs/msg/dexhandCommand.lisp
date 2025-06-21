; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-msg)


;//! \htmlinclude dexhandCommand.msg.html

(cl:defclass <dexhandCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass dexhandCommand (<dexhandCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <dexhandCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'dexhandCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-msg:<dexhandCommand> is deprecated: use kuavo_msgs-msg:dexhandCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <dexhandCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:header-val is deprecated.  Use kuavo_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <dexhandCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:control_mode-val is deprecated.  Use kuavo_msgs-msg:control_mode instead.")
  (control_mode m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <dexhandCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:data-val is deprecated.  Use kuavo_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<dexhandCommand>)))
    "Constants for message type '<dexhandCommand>"
  '((:POSITION_CONTROL . 0)
    (:VELOCITY_CONTROL . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'dexhandCommand)))
    "Constants for message type 'dexhandCommand"
  '((:POSITION_CONTROL . 0)
    (:VELOCITY_CONTROL . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <dexhandCommand>) ostream)
  "Serializes a message object of type '<dexhandCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'control_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <dexhandCommand>) istream)
  "Deserializes a message object of type '<dexhandCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'control_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<dexhandCommand>)))
  "Returns string type for a message object of type '<dexhandCommand>"
  "kuavo_msgs/dexhandCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'dexhandCommand)))
  "Returns string type for a message object of type 'dexhandCommand"
  "kuavo_msgs/dexhandCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<dexhandCommand>)))
  "Returns md5sum for a message object of type '<dexhandCommand>"
  "ab54699609aa0f9682d32ee67eae87dc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'dexhandCommand)))
  "Returns md5sum for a message object of type 'dexhandCommand"
  "ab54699609aa0f9682d32ee67eae87dc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<dexhandCommand>)))
  "Returns full string definition for message of type '<dexhandCommand>"
  (cl:format cl:nil "# Control modes~%int8 POSITION_CONTROL = 0  # Position control mode~%int8 VELOCITY_CONTROL = 1  # Velocity control mode~%~%# Message header~%std_msgs/Header header~%~%# Control mode to be used~%int8 control_mode~%~%# Data array~%# 数据数组，单手时长度必须为6，双手长度必须为12~%# - 位置控制模式下，每个元素的数据的范围为[0, 100], 0 为完全打开，100 为完全关闭~%# - 速度控制模式下，每个元素的数据的范围为[-100, 100] 负数表示打开，正数表示关闭~%int16[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'dexhandCommand)))
  "Returns full string definition for message of type 'dexhandCommand"
  (cl:format cl:nil "# Control modes~%int8 POSITION_CONTROL = 0  # Position control mode~%int8 VELOCITY_CONTROL = 1  # Velocity control mode~%~%# Message header~%std_msgs/Header header~%~%# Control mode to be used~%int8 control_mode~%~%# Data array~%# 数据数组，单手时长度必须为6，双手长度必须为12~%# - 位置控制模式下，每个元素的数据的范围为[0, 100], 0 为完全打开，100 为完全关闭~%# - 速度控制模式下，每个元素的数据的范围为[-100, 100] 负数表示打开，正数表示关闭~%int16[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <dexhandCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <dexhandCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'dexhandCommand
    (cl:cons ':header (header msg))
    (cl:cons ':control_mode (control_mode msg))
    (cl:cons ':data (data msg))
))
