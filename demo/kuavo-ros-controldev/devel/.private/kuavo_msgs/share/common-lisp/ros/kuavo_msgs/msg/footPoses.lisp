; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-msg)


;//! \htmlinclude footPoses.msg.html

(cl:defclass <footPoses> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector kuavo_msgs-msg:footPose)
   :initform (cl:make-array 0 :element-type 'kuavo_msgs-msg:footPose :initial-element (cl:make-instance 'kuavo_msgs-msg:footPose))))
)

(cl:defclass footPoses (<footPoses>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <footPoses>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'footPoses)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-msg:<footPoses> is deprecated: use kuavo_msgs-msg:footPoses instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <footPoses>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:data-val is deprecated.  Use kuavo_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <footPoses>) ostream)
  "Serializes a message object of type '<footPoses>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <footPoses>) istream)
  "Deserializes a message object of type '<footPoses>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kuavo_msgs-msg:footPose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<footPoses>)))
  "Returns string type for a message object of type '<footPoses>"
  "kuavo_msgs/footPoses")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'footPoses)))
  "Returns string type for a message object of type 'footPoses"
  "kuavo_msgs/footPoses")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<footPoses>)))
  "Returns md5sum for a message object of type '<footPoses>"
  "2fad89e6e8a2f3c5d0ec891cc7c76c35")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'footPoses)))
  "Returns md5sum for a message object of type 'footPoses"
  "2fad89e6e8a2f3c5d0ec891cc7c76c35")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<footPoses>)))
  "Returns full string definition for message of type '<footPoses>"
  (cl:format cl:nil "footPose[] data~%~%================================================================================~%MSG: kuavo_msgs/footPose~%float64[4] footPose # x, y, z, yaw~%float64[4] torsoPose # x, y, z, yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'footPoses)))
  "Returns full string definition for message of type 'footPoses"
  (cl:format cl:nil "footPose[] data~%~%================================================================================~%MSG: kuavo_msgs/footPose~%float64[4] footPose # x, y, z, yaw~%float64[4] torsoPose # x, y, z, yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <footPoses>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <footPoses>))
  "Converts a ROS message object to a list"
  (cl:list 'footPoses
    (cl:cons ':data (data msg))
))
