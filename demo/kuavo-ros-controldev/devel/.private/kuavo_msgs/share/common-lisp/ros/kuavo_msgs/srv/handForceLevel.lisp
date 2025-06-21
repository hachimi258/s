; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-srv)


;//! \htmlinclude handForceLevel-request.msg.html

(cl:defclass <handForceLevel-request> (roslisp-msg-protocol:ros-message)
  ((force_level
    :reader force_level
    :initarg :force_level
    :type cl:fixnum
    :initform 0)
   (hand_side
    :reader hand_side
    :initarg :hand_side
    :type cl:fixnum
    :initform 0))
)

(cl:defclass handForceLevel-request (<handForceLevel-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <handForceLevel-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'handForceLevel-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<handForceLevel-request> is deprecated: use kuavo_msgs-srv:handForceLevel-request instead.")))

(cl:ensure-generic-function 'force_level-val :lambda-list '(m))
(cl:defmethod force_level-val ((m <handForceLevel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:force_level-val is deprecated.  Use kuavo_msgs-srv:force_level instead.")
  (force_level m))

(cl:ensure-generic-function 'hand_side-val :lambda-list '(m))
(cl:defmethod hand_side-val ((m <handForceLevel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:hand_side-val is deprecated.  Use kuavo_msgs-srv:hand_side instead.")
  (hand_side m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<handForceLevel-request>)))
    "Constants for message type '<handForceLevel-request>"
  '((:SMALL . 0)
    (:NORMAL . 1)
    (:FULL . 2)
    (:LEFT_HAND . 0)
    (:RIGHT_HAND . 1)
    (:BOTH_HANDS . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'handForceLevel-request)))
    "Constants for message type 'handForceLevel-request"
  '((:SMALL . 0)
    (:NORMAL . 1)
    (:FULL . 2)
    (:LEFT_HAND . 0)
    (:RIGHT_HAND . 1)
    (:BOTH_HANDS . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <handForceLevel-request>) ostream)
  "Serializes a message object of type '<handForceLevel-request>"
  (cl:let* ((signed (cl:slot-value msg 'force_level)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'hand_side)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <handForceLevel-request>) istream)
  "Deserializes a message object of type '<handForceLevel-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'force_level) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hand_side) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<handForceLevel-request>)))
  "Returns string type for a service object of type '<handForceLevel-request>"
  "kuavo_msgs/handForceLevelRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'handForceLevel-request)))
  "Returns string type for a service object of type 'handForceLevel-request"
  "kuavo_msgs/handForceLevelRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<handForceLevel-request>)))
  "Returns md5sum for a message object of type '<handForceLevel-request>"
  "0f6c4fd291557ab445334f0487fbaf78")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'handForceLevel-request)))
  "Returns md5sum for a message object of type 'handForceLevel-request"
  "0f6c4fd291557ab445334f0487fbaf78")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<handForceLevel-request>)))
  "Returns full string definition for message of type '<handForceLevel-request>"
  (cl:format cl:nil "# This service sets the force level for the Dexhand.~%# It is used to control the force applied by the Dexhand.~%#~%# Request:~%# int8 SMALL = 0          # Small force level.~%# int8 NORMAL = 1         # Normal force level.~%# int8 FULL = 2           # Full force level.~%# int8 force_level        # The desired force level to set.~%#~%# Response:~%# bool success            # Indicates whether the request was successful.~%# string message          # A message indicating the result of the request.~%~%int8 SMALL = 0~%int8 NORMAL = 1~%int8 FULL = 2~%~%int8 LEFT_HAND=0~%int8 RIGHT_HAND=1~%int8 BOTH_HANDS=2~%~%int8 force_level~%int8 hand_side~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'handForceLevel-request)))
  "Returns full string definition for message of type 'handForceLevel-request"
  (cl:format cl:nil "# This service sets the force level for the Dexhand.~%# It is used to control the force applied by the Dexhand.~%#~%# Request:~%# int8 SMALL = 0          # Small force level.~%# int8 NORMAL = 1         # Normal force level.~%# int8 FULL = 2           # Full force level.~%# int8 force_level        # The desired force level to set.~%#~%# Response:~%# bool success            # Indicates whether the request was successful.~%# string message          # A message indicating the result of the request.~%~%int8 SMALL = 0~%int8 NORMAL = 1~%int8 FULL = 2~%~%int8 LEFT_HAND=0~%int8 RIGHT_HAND=1~%int8 BOTH_HANDS=2~%~%int8 force_level~%int8 hand_side~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <handForceLevel-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <handForceLevel-request>))
  "Converts a ROS message object to a list"
  (cl:list 'handForceLevel-request
    (cl:cons ':force_level (force_level msg))
    (cl:cons ':hand_side (hand_side msg))
))
;//! \htmlinclude handForceLevel-response.msg.html

(cl:defclass <handForceLevel-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (force_level
    :reader force_level
    :initarg :force_level
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass handForceLevel-response (<handForceLevel-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <handForceLevel-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'handForceLevel-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<handForceLevel-response> is deprecated: use kuavo_msgs-srv:handForceLevel-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <handForceLevel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:success-val is deprecated.  Use kuavo_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <handForceLevel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:message-val is deprecated.  Use kuavo_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'force_level-val :lambda-list '(m))
(cl:defmethod force_level-val ((m <handForceLevel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:force_level-val is deprecated.  Use kuavo_msgs-srv:force_level instead.")
  (force_level m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <handForceLevel-response>) ostream)
  "Serializes a message object of type '<handForceLevel-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'force_level))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'force_level))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <handForceLevel-response>) istream)
  "Deserializes a message object of type '<handForceLevel-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'force_level) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'force_level)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<handForceLevel-response>)))
  "Returns string type for a service object of type '<handForceLevel-response>"
  "kuavo_msgs/handForceLevelResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'handForceLevel-response)))
  "Returns string type for a service object of type 'handForceLevel-response"
  "kuavo_msgs/handForceLevelResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<handForceLevel-response>)))
  "Returns md5sum for a message object of type '<handForceLevel-response>"
  "0f6c4fd291557ab445334f0487fbaf78")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'handForceLevel-response)))
  "Returns md5sum for a message object of type 'handForceLevel-response"
  "0f6c4fd291557ab445334f0487fbaf78")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<handForceLevel-response>)))
  "Returns full string definition for message of type '<handForceLevel-response>"
  (cl:format cl:nil "bool success~%string message~%int8[] force_level~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'handForceLevel-response)))
  "Returns full string definition for message of type 'handForceLevel-response"
  (cl:format cl:nil "bool success~%string message~%int8[] force_level~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <handForceLevel-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'force_level) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <handForceLevel-response>))
  "Converts a ROS message object to a list"
  (cl:list 'handForceLevel-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':force_level (force_level msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'handForceLevel)))
  'handForceLevel-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'handForceLevel)))
  'handForceLevel-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'handForceLevel)))
  "Returns string type for a service object of type '<handForceLevel>"
  "kuavo_msgs/handForceLevel")