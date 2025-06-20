; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-msg)


;//! \htmlinclude lejuClawCommand.msg.html

(cl:defclass <lejuClawCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type kuavo_msgs-msg:endEffectorData
    :initform (cl:make-instance 'kuavo_msgs-msg:endEffectorData)))
)

(cl:defclass lejuClawCommand (<lejuClawCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lejuClawCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lejuClawCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-msg:<lejuClawCommand> is deprecated: use kuavo_msgs-msg:lejuClawCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <lejuClawCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:header-val is deprecated.  Use kuavo_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <lejuClawCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:data-val is deprecated.  Use kuavo_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lejuClawCommand>) ostream)
  "Serializes a message object of type '<lejuClawCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lejuClawCommand>) istream)
  "Deserializes a message object of type '<lejuClawCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lejuClawCommand>)))
  "Returns string type for a message object of type '<lejuClawCommand>"
  "kuavo_msgs/lejuClawCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lejuClawCommand)))
  "Returns string type for a message object of type 'lejuClawCommand"
  "kuavo_msgs/lejuClawCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lejuClawCommand>)))
  "Returns md5sum for a message object of type '<lejuClawCommand>"
  "57c40e80f90e7a289ae0b2488552e043")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lejuClawCommand)))
  "Returns md5sum for a message object of type 'lejuClawCommand"
  "57c40e80f90e7a289ae0b2488552e043")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lejuClawCommand>)))
  "Returns full string definition for message of type '<lejuClawCommand>"
  (cl:format cl:nil "# kuavo_msgs/endEffectorData:~%# string[] name  ~%# float64[] position~%# float64[] velocity  ~%# float64[] effort~%# ~%# ** For the Topic Notes **~%# ~%# name     : 'left_claw' , 'right_claw'~%# position : 0 ~~ 100, the percentage of the claw's opening angle~%#            0: closed, 100: open   ~%# velocity : 0 ~~ 100, if size is 0, will use default `50.0`.~%# effort   : torque/current, better 1A ~~ 2A, if size is 0, will use default `1.0`.~%# ~%# ** Example **~%# data:~%#   - name: ['left_claw', 'right_claw']~%#     position: [20.0, 20.0]~%#     velocity: [50.0, 50.0]~%#     effort: [1.0, 1.0]~%~%std_msgs/Header header~%kuavo_msgs/endEffectorData data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kuavo_msgs/endEffectorData~%string[] name  ~%float64[] position~%float64[] velocity  ~%float64[] effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lejuClawCommand)))
  "Returns full string definition for message of type 'lejuClawCommand"
  (cl:format cl:nil "# kuavo_msgs/endEffectorData:~%# string[] name  ~%# float64[] position~%# float64[] velocity  ~%# float64[] effort~%# ~%# ** For the Topic Notes **~%# ~%# name     : 'left_claw' , 'right_claw'~%# position : 0 ~~ 100, the percentage of the claw's opening angle~%#            0: closed, 100: open   ~%# velocity : 0 ~~ 100, if size is 0, will use default `50.0`.~%# effort   : torque/current, better 1A ~~ 2A, if size is 0, will use default `1.0`.~%# ~%# ** Example **~%# data:~%#   - name: ['left_claw', 'right_claw']~%#     position: [20.0, 20.0]~%#     velocity: [50.0, 50.0]~%#     effort: [1.0, 1.0]~%~%std_msgs/Header header~%kuavo_msgs/endEffectorData data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kuavo_msgs/endEffectorData~%string[] name  ~%float64[] position~%float64[] velocity  ~%float64[] effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lejuClawCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lejuClawCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'lejuClawCommand
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
