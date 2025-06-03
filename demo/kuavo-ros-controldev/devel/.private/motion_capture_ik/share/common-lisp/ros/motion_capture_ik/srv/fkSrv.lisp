; Auto-generated. Do not edit!


(cl:in-package motion_capture_ik-srv)


;//! \htmlinclude fkSrv-request.msg.html

(cl:defclass <fkSrv-request> (roslisp-msg-protocol:ros-message)
  ((q
    :reader q
    :initarg :q
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass fkSrv-request (<fkSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <fkSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'fkSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_capture_ik-srv:<fkSrv-request> is deprecated: use motion_capture_ik-srv:fkSrv-request instead.")))

(cl:ensure-generic-function 'q-val :lambda-list '(m))
(cl:defmethod q-val ((m <fkSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_capture_ik-srv:q-val is deprecated.  Use motion_capture_ik-srv:q instead.")
  (q m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <fkSrv-request>) ostream)
  "Serializes a message object of type '<fkSrv-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'q))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'q))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <fkSrv-request>) istream)
  "Deserializes a message object of type '<fkSrv-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'q) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'q)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<fkSrv-request>)))
  "Returns string type for a service object of type '<fkSrv-request>"
  "motion_capture_ik/fkSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'fkSrv-request)))
  "Returns string type for a service object of type 'fkSrv-request"
  "motion_capture_ik/fkSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<fkSrv-request>)))
  "Returns md5sum for a message object of type '<fkSrv-request>"
  "b89cc987a02b6d1c2a1588d5659bf064")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'fkSrv-request)))
  "Returns md5sum for a message object of type 'fkSrv-request"
  "b89cc987a02b6d1c2a1588d5659bf064")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<fkSrv-request>)))
  "Returns full string definition for message of type '<fkSrv-request>"
  (cl:format cl:nil "float64[] q # 广义关节角度，如果是虚拟关节ik，则前4维度为躯干虚拟关节的角度，后14维度为手臂关节的角度~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'fkSrv-request)))
  "Returns full string definition for message of type 'fkSrv-request"
  (cl:format cl:nil "float64[] q # 广义关节角度，如果是虚拟关节ik，则前4维度为躯干虚拟关节的角度，后14维度为手臂关节的角度~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <fkSrv-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'q) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <fkSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'fkSrv-request
    (cl:cons ':q (q msg))
))
;//! \htmlinclude fkSrv-response.msg.html

(cl:defclass <fkSrv-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (hand_poses
    :reader hand_poses
    :initarg :hand_poses
    :type motion_capture_ik-msg:twoArmHandPose
    :initform (cl:make-instance 'motion_capture_ik-msg:twoArmHandPose)))
)

(cl:defclass fkSrv-response (<fkSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <fkSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'fkSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_capture_ik-srv:<fkSrv-response> is deprecated: use motion_capture_ik-srv:fkSrv-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <fkSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_capture_ik-srv:success-val is deprecated.  Use motion_capture_ik-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'hand_poses-val :lambda-list '(m))
(cl:defmethod hand_poses-val ((m <fkSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_capture_ik-srv:hand_poses-val is deprecated.  Use motion_capture_ik-srv:hand_poses instead.")
  (hand_poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <fkSrv-response>) ostream)
  "Serializes a message object of type '<fkSrv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'hand_poses) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <fkSrv-response>) istream)
  "Deserializes a message object of type '<fkSrv-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'hand_poses) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<fkSrv-response>)))
  "Returns string type for a service object of type '<fkSrv-response>"
  "motion_capture_ik/fkSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'fkSrv-response)))
  "Returns string type for a service object of type 'fkSrv-response"
  "motion_capture_ik/fkSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<fkSrv-response>)))
  "Returns md5sum for a message object of type '<fkSrv-response>"
  "b89cc987a02b6d1c2a1588d5659bf064")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'fkSrv-response)))
  "Returns md5sum for a message object of type 'fkSrv-response"
  "b89cc987a02b6d1c2a1588d5659bf064")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<fkSrv-response>)))
  "Returns full string definition for message of type '<fkSrv-response>"
  (cl:format cl:nil "bool success~%twoArmHandPose  hand_poses~%~%================================================================================~%MSG: motion_capture_ik/twoArmHandPose~%Header header~%armHandPose  left_pose~%armHandPose  right_pose~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: motion_capture_ik/armHandPose~%float64[3] pos_xyz~%float64[4] quat_xyzw~%~%float64[3] elbow_pos_xyz~%~%float64[7] joint_angles~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'fkSrv-response)))
  "Returns full string definition for message of type 'fkSrv-response"
  (cl:format cl:nil "bool success~%twoArmHandPose  hand_poses~%~%================================================================================~%MSG: motion_capture_ik/twoArmHandPose~%Header header~%armHandPose  left_pose~%armHandPose  right_pose~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: motion_capture_ik/armHandPose~%float64[3] pos_xyz~%float64[4] quat_xyzw~%~%float64[3] elbow_pos_xyz~%~%float64[7] joint_angles~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <fkSrv-response>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'hand_poses))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <fkSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'fkSrv-response
    (cl:cons ':success (success msg))
    (cl:cons ':hand_poses (hand_poses msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'fkSrv)))
  'fkSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'fkSrv)))
  'fkSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'fkSrv)))
  "Returns string type for a service object of type '<fkSrv>"
  "motion_capture_ik/fkSrv")