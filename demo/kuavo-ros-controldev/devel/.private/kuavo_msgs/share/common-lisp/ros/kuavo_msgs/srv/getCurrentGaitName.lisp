; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-srv)


;//! \htmlinclude getCurrentGaitName-request.msg.html

(cl:defclass <getCurrentGaitName-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getCurrentGaitName-request (<getCurrentGaitName-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getCurrentGaitName-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getCurrentGaitName-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<getCurrentGaitName-request> is deprecated: use kuavo_msgs-srv:getCurrentGaitName-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getCurrentGaitName-request>) ostream)
  "Serializes a message object of type '<getCurrentGaitName-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getCurrentGaitName-request>) istream)
  "Deserializes a message object of type '<getCurrentGaitName-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getCurrentGaitName-request>)))
  "Returns string type for a service object of type '<getCurrentGaitName-request>"
  "kuavo_msgs/getCurrentGaitNameRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getCurrentGaitName-request)))
  "Returns string type for a service object of type 'getCurrentGaitName-request"
  "kuavo_msgs/getCurrentGaitNameRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getCurrentGaitName-request>)))
  "Returns md5sum for a message object of type '<getCurrentGaitName-request>"
  "657f58245742fcfc53dd6ab5bfa3063e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getCurrentGaitName-request)))
  "Returns md5sum for a message object of type 'getCurrentGaitName-request"
  "657f58245742fcfc53dd6ab5bfa3063e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getCurrentGaitName-request>)))
  "Returns full string definition for message of type '<getCurrentGaitName-request>"
  (cl:format cl:nil "# Request: Empty message, no fields needed~%~%# Response:~%# success: Whether the service call was successful~%# gait_name: Name of the current gait being executed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getCurrentGaitName-request)))
  "Returns full string definition for message of type 'getCurrentGaitName-request"
  (cl:format cl:nil "# Request: Empty message, no fields needed~%~%# Response:~%# success: Whether the service call was successful~%# gait_name: Name of the current gait being executed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getCurrentGaitName-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getCurrentGaitName-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getCurrentGaitName-request
))
;//! \htmlinclude getCurrentGaitName-response.msg.html

(cl:defclass <getCurrentGaitName-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (gait_name
    :reader gait_name
    :initarg :gait_name
    :type cl:string
    :initform ""))
)

(cl:defclass getCurrentGaitName-response (<getCurrentGaitName-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getCurrentGaitName-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getCurrentGaitName-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<getCurrentGaitName-response> is deprecated: use kuavo_msgs-srv:getCurrentGaitName-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <getCurrentGaitName-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:success-val is deprecated.  Use kuavo_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'gait_name-val :lambda-list '(m))
(cl:defmethod gait_name-val ((m <getCurrentGaitName-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:gait_name-val is deprecated.  Use kuavo_msgs-srv:gait_name instead.")
  (gait_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getCurrentGaitName-response>) ostream)
  "Serializes a message object of type '<getCurrentGaitName-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'gait_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'gait_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getCurrentGaitName-response>) istream)
  "Deserializes a message object of type '<getCurrentGaitName-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getCurrentGaitName-response>)))
  "Returns string type for a service object of type '<getCurrentGaitName-response>"
  "kuavo_msgs/getCurrentGaitNameResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getCurrentGaitName-response)))
  "Returns string type for a service object of type 'getCurrentGaitName-response"
  "kuavo_msgs/getCurrentGaitNameResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getCurrentGaitName-response>)))
  "Returns md5sum for a message object of type '<getCurrentGaitName-response>"
  "657f58245742fcfc53dd6ab5bfa3063e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getCurrentGaitName-response)))
  "Returns md5sum for a message object of type 'getCurrentGaitName-response"
  "657f58245742fcfc53dd6ab5bfa3063e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getCurrentGaitName-response>)))
  "Returns full string definition for message of type '<getCurrentGaitName-response>"
  (cl:format cl:nil "bool success~%string gait_name~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getCurrentGaitName-response)))
  "Returns full string definition for message of type 'getCurrentGaitName-response"
  (cl:format cl:nil "bool success~%string gait_name~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getCurrentGaitName-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'gait_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getCurrentGaitName-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getCurrentGaitName-response
    (cl:cons ':success (success msg))
    (cl:cons ':gait_name (gait_name msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getCurrentGaitName)))
  'getCurrentGaitName-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getCurrentGaitName)))
  'getCurrentGaitName-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getCurrentGaitName)))
  "Returns string type for a service object of type '<getCurrentGaitName>"
  "kuavo_msgs/getCurrentGaitName")