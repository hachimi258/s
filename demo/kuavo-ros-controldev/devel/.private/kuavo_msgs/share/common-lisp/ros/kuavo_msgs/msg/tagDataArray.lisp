; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-msg)


;//! \htmlinclude tagDataArray.msg.html

(cl:defclass <tagDataArray> (roslisp-msg-protocol:ros-message)
  ((tag_ids
    :reader tag_ids
    :initarg :tag_ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (tag_poses
    :reader tag_poses
    :initarg :tag_poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass tagDataArray (<tagDataArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tagDataArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tagDataArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-msg:<tagDataArray> is deprecated: use kuavo_msgs-msg:tagDataArray instead.")))

(cl:ensure-generic-function 'tag_ids-val :lambda-list '(m))
(cl:defmethod tag_ids-val ((m <tagDataArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:tag_ids-val is deprecated.  Use kuavo_msgs-msg:tag_ids instead.")
  (tag_ids m))

(cl:ensure-generic-function 'tag_poses-val :lambda-list '(m))
(cl:defmethod tag_poses-val ((m <tagDataArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:tag_poses-val is deprecated.  Use kuavo_msgs-msg:tag_poses instead.")
  (tag_poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tagDataArray>) ostream)
  "Serializes a message object of type '<tagDataArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tag_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'tag_ids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tag_poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tag_poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tagDataArray>) istream)
  "Deserializes a message object of type '<tagDataArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tag_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tag_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tag_poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tag_poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tagDataArray>)))
  "Returns string type for a message object of type '<tagDataArray>"
  "kuavo_msgs/tagDataArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tagDataArray)))
  "Returns string type for a message object of type 'tagDataArray"
  "kuavo_msgs/tagDataArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tagDataArray>)))
  "Returns md5sum for a message object of type '<tagDataArray>"
  "454da6edf551b421dda595d3272ef7ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tagDataArray)))
  "Returns md5sum for a message object of type 'tagDataArray"
  "454da6edf551b421dda595d3272ef7ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tagDataArray>)))
  "Returns full string definition for message of type '<tagDataArray>"
  (cl:format cl:nil "int32[] tag_ids~%geometry_msgs/Pose[] tag_poses~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tagDataArray)))
  "Returns full string definition for message of type 'tagDataArray"
  (cl:format cl:nil "int32[] tag_ids~%geometry_msgs/Pose[] tag_poses~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tagDataArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tag_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tag_poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tagDataArray>))
  "Converts a ROS message object to a list"
  (cl:list 'tagDataArray
    (cl:cons ':tag_ids (tag_ids msg))
    (cl:cons ':tag_poses (tag_poses msg))
))
