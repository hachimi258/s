; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-msg)


;//! \htmlinclude switchGaitByName.msg.html

(cl:defclass <switchGaitByName> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (gait_name
    :reader gait_name
    :initarg :gait_name
    :type cl:string
    :initform ""))
)

(cl:defclass switchGaitByName (<switchGaitByName>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <switchGaitByName>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'switchGaitByName)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-msg:<switchGaitByName> is deprecated: use kuavo_msgs-msg:switchGaitByName instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <switchGaitByName>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:header-val is deprecated.  Use kuavo_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'gait_name-val :lambda-list '(m))
(cl:defmethod gait_name-val ((m <switchGaitByName>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:gait_name-val is deprecated.  Use kuavo_msgs-msg:gait_name instead.")
  (gait_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <switchGaitByName>) ostream)
  "Serializes a message object of type '<switchGaitByName>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'gait_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'gait_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <switchGaitByName>) istream)
  "Deserializes a message object of type '<switchGaitByName>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gait_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'gait_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<switchGaitByName>)))
  "Returns string type for a message object of type '<switchGaitByName>"
  "kuavo_msgs/switchGaitByName")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'switchGaitByName)))
  "Returns string type for a message object of type 'switchGaitByName"
  "kuavo_msgs/switchGaitByName")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<switchGaitByName>)))
  "Returns md5sum for a message object of type '<switchGaitByName>"
  "f98f610ddbe99ec14bb5241e5221ca96")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'switchGaitByName)))
  "Returns md5sum for a message object of type 'switchGaitByName"
  "f98f610ddbe99ec14bb5241e5221ca96")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<switchGaitByName>)))
  "Returns full string definition for message of type '<switchGaitByName>"
  (cl:format cl:nil "std_msgs/Header header~%string  gait_name~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'switchGaitByName)))
  "Returns full string definition for message of type 'switchGaitByName"
  (cl:format cl:nil "std_msgs/Header header~%string  gait_name~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <switchGaitByName>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'gait_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <switchGaitByName>))
  "Converts a ROS message object to a list"
  (cl:list 'switchGaitByName
    (cl:cons ':header (header msg))
    (cl:cons ':gait_name (gait_name msg))
))
