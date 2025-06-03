; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-srv)


;//! \htmlinclude gestureExecuteState-request.msg.html

(cl:defclass <gestureExecuteState-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass gestureExecuteState-request (<gestureExecuteState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gestureExecuteState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gestureExecuteState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<gestureExecuteState-request> is deprecated: use kuavo_msgs-srv:gestureExecuteState-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gestureExecuteState-request>) ostream)
  "Serializes a message object of type '<gestureExecuteState-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gestureExecuteState-request>) istream)
  "Deserializes a message object of type '<gestureExecuteState-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gestureExecuteState-request>)))
  "Returns string type for a service object of type '<gestureExecuteState-request>"
  "kuavo_msgs/gestureExecuteStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gestureExecuteState-request)))
  "Returns string type for a service object of type 'gestureExecuteState-request"
  "kuavo_msgs/gestureExecuteStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gestureExecuteState-request>)))
  "Returns md5sum for a message object of type '<gestureExecuteState-request>"
  "340cb7a19883494e0e422f2964927053")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gestureExecuteState-request)))
  "Returns md5sum for a message object of type 'gestureExecuteState-request"
  "340cb7a19883494e0e422f2964927053")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gestureExecuteState-request>)))
  "Returns full string definition for message of type '<gestureExecuteState-request>"
  (cl:format cl:nil "# This service checks the status of a task.~%# It is used to determine whether a task is currently executing and provides an additional message.~%~%# Request:~%# (No request fields for this service)~%~%# Response:~%# bool is_executing  # Indicates whether the task is currently executing.~%# string message     # A message providing additional information (e.g., status details or error messages).~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gestureExecuteState-request)))
  "Returns full string definition for message of type 'gestureExecuteState-request"
  (cl:format cl:nil "# This service checks the status of a task.~%# It is used to determine whether a task is currently executing and provides an additional message.~%~%# Request:~%# (No request fields for this service)~%~%# Response:~%# bool is_executing  # Indicates whether the task is currently executing.~%# string message     # A message providing additional information (e.g., status details or error messages).~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gestureExecuteState-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gestureExecuteState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gestureExecuteState-request
))
;//! \htmlinclude gestureExecuteState-response.msg.html

(cl:defclass <gestureExecuteState-response> (roslisp-msg-protocol:ros-message)
  ((is_executing
    :reader is_executing
    :initarg :is_executing
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass gestureExecuteState-response (<gestureExecuteState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gestureExecuteState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gestureExecuteState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<gestureExecuteState-response> is deprecated: use kuavo_msgs-srv:gestureExecuteState-response instead.")))

(cl:ensure-generic-function 'is_executing-val :lambda-list '(m))
(cl:defmethod is_executing-val ((m <gestureExecuteState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:is_executing-val is deprecated.  Use kuavo_msgs-srv:is_executing instead.")
  (is_executing m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <gestureExecuteState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:message-val is deprecated.  Use kuavo_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gestureExecuteState-response>) ostream)
  "Serializes a message object of type '<gestureExecuteState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_executing) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gestureExecuteState-response>) istream)
  "Deserializes a message object of type '<gestureExecuteState-response>"
    (cl:setf (cl:slot-value msg 'is_executing) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gestureExecuteState-response>)))
  "Returns string type for a service object of type '<gestureExecuteState-response>"
  "kuavo_msgs/gestureExecuteStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gestureExecuteState-response)))
  "Returns string type for a service object of type 'gestureExecuteState-response"
  "kuavo_msgs/gestureExecuteStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gestureExecuteState-response>)))
  "Returns md5sum for a message object of type '<gestureExecuteState-response>"
  "340cb7a19883494e0e422f2964927053")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gestureExecuteState-response)))
  "Returns md5sum for a message object of type 'gestureExecuteState-response"
  "340cb7a19883494e0e422f2964927053")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gestureExecuteState-response>)))
  "Returns full string definition for message of type '<gestureExecuteState-response>"
  (cl:format cl:nil "bool is_executing~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gestureExecuteState-response)))
  "Returns full string definition for message of type 'gestureExecuteState-response"
  (cl:format cl:nil "bool is_executing~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gestureExecuteState-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gestureExecuteState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gestureExecuteState-response
    (cl:cons ':is_executing (is_executing msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gestureExecuteState)))
  'gestureExecuteState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gestureExecuteState)))
  'gestureExecuteState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gestureExecuteState)))
  "Returns string type for a service object of type '<gestureExecuteState>"
  "kuavo_msgs/gestureExecuteState")