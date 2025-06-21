; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-msg)


;//! \htmlinclude dexhandTouchState.msg.html

(cl:defclass <dexhandTouchState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left_hand
    :reader left_hand
    :initarg :left_hand
    :type (cl:vector kuavo_msgs-msg:touchSensorStatus)
   :initform (cl:make-array 5 :element-type 'kuavo_msgs-msg:touchSensorStatus :initial-element (cl:make-instance 'kuavo_msgs-msg:touchSensorStatus)))
   (right_hand
    :reader right_hand
    :initarg :right_hand
    :type (cl:vector kuavo_msgs-msg:touchSensorStatus)
   :initform (cl:make-array 5 :element-type 'kuavo_msgs-msg:touchSensorStatus :initial-element (cl:make-instance 'kuavo_msgs-msg:touchSensorStatus))))
)

(cl:defclass dexhandTouchState (<dexhandTouchState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <dexhandTouchState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'dexhandTouchState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-msg:<dexhandTouchState> is deprecated: use kuavo_msgs-msg:dexhandTouchState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <dexhandTouchState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:header-val is deprecated.  Use kuavo_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left_hand-val :lambda-list '(m))
(cl:defmethod left_hand-val ((m <dexhandTouchState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:left_hand-val is deprecated.  Use kuavo_msgs-msg:left_hand instead.")
  (left_hand m))

(cl:ensure-generic-function 'right_hand-val :lambda-list '(m))
(cl:defmethod right_hand-val ((m <dexhandTouchState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:right_hand-val is deprecated.  Use kuavo_msgs-msg:right_hand instead.")
  (right_hand m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <dexhandTouchState>) ostream)
  "Serializes a message object of type '<dexhandTouchState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'left_hand))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'right_hand))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <dexhandTouchState>) istream)
  "Deserializes a message object of type '<dexhandTouchState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'left_hand) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'left_hand)))
    (cl:dotimes (i 5)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kuavo_msgs-msg:touchSensorStatus))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'right_hand) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'right_hand)))
    (cl:dotimes (i 5)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kuavo_msgs-msg:touchSensorStatus))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<dexhandTouchState>)))
  "Returns string type for a message object of type '<dexhandTouchState>"
  "kuavo_msgs/dexhandTouchState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'dexhandTouchState)))
  "Returns string type for a message object of type 'dexhandTouchState"
  "kuavo_msgs/dexhandTouchState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<dexhandTouchState>)))
  "Returns md5sum for a message object of type '<dexhandTouchState>"
  "ce777577e1167705dca90d1f63037a05")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'dexhandTouchState)))
  "Returns md5sum for a message object of type 'dexhandTouchState"
  "ce777577e1167705dca90d1f63037a05")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<dexhandTouchState>)))
  "Returns full string definition for message of type '<dexhandTouchState>"
  (cl:format cl:nil "std_msgs/Header header~%kuavo_msgs/touchSensorStatus[5] left_hand~%kuavo_msgs/touchSensorStatus[5] right_hand~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kuavo_msgs/touchSensorStatus~%uint16 normal_force1  # 法向力1~%uint16 normal_force2  # 法向力2~%uint16 normal_force3  # 法向力3~%uint16 tangential_force1  # 切向力1~%uint16 tangential_force2  # 切向力2~%uint16 tangential_force3  # 切向力3~%uint16 tangential_direction1  # 切向力方向1~%uint16 tangential_direction2  # 切向力方向2~%uint16 tangential_direction3  # 切向力方向3~%uint32 self_proximity1  # 自电容接近传感器1~%uint32 self_proximity2  # 自电容接近传感器2~%uint32 mutual_proximity  # 互电容接近传感器~%uint16 status  # 传感器状态~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'dexhandTouchState)))
  "Returns full string definition for message of type 'dexhandTouchState"
  (cl:format cl:nil "std_msgs/Header header~%kuavo_msgs/touchSensorStatus[5] left_hand~%kuavo_msgs/touchSensorStatus[5] right_hand~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kuavo_msgs/touchSensorStatus~%uint16 normal_force1  # 法向力1~%uint16 normal_force2  # 法向力2~%uint16 normal_force3  # 法向力3~%uint16 tangential_force1  # 切向力1~%uint16 tangential_force2  # 切向力2~%uint16 tangential_force3  # 切向力3~%uint16 tangential_direction1  # 切向力方向1~%uint16 tangential_direction2  # 切向力方向2~%uint16 tangential_direction3  # 切向力方向3~%uint32 self_proximity1  # 自电容接近传感器1~%uint32 self_proximity2  # 自电容接近传感器2~%uint32 mutual_proximity  # 互电容接近传感器~%uint16 status  # 传感器状态~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <dexhandTouchState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'left_hand) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'right_hand) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <dexhandTouchState>))
  "Converts a ROS message object to a list"
  (cl:list 'dexhandTouchState
    (cl:cons ':header (header msg))
    (cl:cons ':left_hand (left_hand msg))
    (cl:cons ':right_hand (right_hand msg))
))
