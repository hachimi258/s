; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-srv)


;//! \htmlinclude setTagId-request.msg.html

(cl:defclass <setTagId-request> (roslisp-msg-protocol:ros-message)
  ((tag_id
    :reader tag_id
    :initarg :tag_id
    :type cl:integer
    :initform 0))
)

(cl:defclass setTagId-request (<setTagId-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setTagId-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setTagId-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<setTagId-request> is deprecated: use kuavo_msgs-srv:setTagId-request instead.")))

(cl:ensure-generic-function 'tag_id-val :lambda-list '(m))
(cl:defmethod tag_id-val ((m <setTagId-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:tag_id-val is deprecated.  Use kuavo_msgs-srv:tag_id instead.")
  (tag_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setTagId-request>) ostream)
  "Serializes a message object of type '<setTagId-request>"
  (cl:let* ((signed (cl:slot-value msg 'tag_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setTagId-request>) istream)
  "Deserializes a message object of type '<setTagId-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tag_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setTagId-request>)))
  "Returns string type for a service object of type '<setTagId-request>"
  "kuavo_msgs/setTagIdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setTagId-request)))
  "Returns string type for a service object of type 'setTagId-request"
  "kuavo_msgs/setTagIdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setTagId-request>)))
  "Returns md5sum for a message object of type '<setTagId-request>"
  "f6b2c1ef4eedb7525fd3a42b8e5cd61f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setTagId-request)))
  "Returns md5sum for a message object of type 'setTagId-request"
  "f6b2c1ef4eedb7525fd3a42b8e5cd61f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setTagId-request>)))
  "Returns full string definition for message of type '<setTagId-request>"
  (cl:format cl:nil "int32 tag_id ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setTagId-request)))
  "Returns full string definition for message of type 'setTagId-request"
  (cl:format cl:nil "int32 tag_id ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setTagId-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setTagId-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setTagId-request
    (cl:cons ':tag_id (tag_id msg))
))
;//! \htmlinclude setTagId-response.msg.html

(cl:defclass <setTagId-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (tag_id
    :reader tag_id
    :initarg :tag_id
    :type cl:integer
    :initform 0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass setTagId-response (<setTagId-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setTagId-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setTagId-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<setTagId-response> is deprecated: use kuavo_msgs-srv:setTagId-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <setTagId-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:success-val is deprecated.  Use kuavo_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'tag_id-val :lambda-list '(m))
(cl:defmethod tag_id-val ((m <setTagId-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:tag_id-val is deprecated.  Use kuavo_msgs-srv:tag_id instead.")
  (tag_id m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <setTagId-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:message-val is deprecated.  Use kuavo_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setTagId-response>) ostream)
  "Serializes a message object of type '<setTagId-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'tag_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setTagId-response>) istream)
  "Deserializes a message object of type '<setTagId-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tag_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setTagId-response>)))
  "Returns string type for a service object of type '<setTagId-response>"
  "kuavo_msgs/setTagIdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setTagId-response)))
  "Returns string type for a service object of type 'setTagId-response"
  "kuavo_msgs/setTagIdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setTagId-response>)))
  "Returns md5sum for a message object of type '<setTagId-response>"
  "f6b2c1ef4eedb7525fd3a42b8e5cd61f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setTagId-response)))
  "Returns md5sum for a message object of type 'setTagId-response"
  "f6b2c1ef4eedb7525fd3a42b8e5cd61f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setTagId-response>)))
  "Returns full string definition for message of type '<setTagId-response>"
  (cl:format cl:nil "bool success~%int32 tag_id ~%string message~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setTagId-response)))
  "Returns full string definition for message of type 'setTagId-response"
  (cl:format cl:nil "bool success~%int32 tag_id ~%string message~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setTagId-response>))
  (cl:+ 0
     1
     4
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setTagId-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setTagId-response
    (cl:cons ':success (success msg))
    (cl:cons ':tag_id (tag_id msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setTagId)))
  'setTagId-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setTagId)))
  'setTagId-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setTagId)))
  "Returns string type for a service object of type '<setTagId>"
  "kuavo_msgs/setTagId")