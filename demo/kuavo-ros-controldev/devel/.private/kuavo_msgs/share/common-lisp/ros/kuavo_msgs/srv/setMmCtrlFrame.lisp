; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-srv)


;//! \htmlinclude setMmCtrlFrame-request.msg.html

(cl:defclass <setMmCtrlFrame-request> (roslisp-msg-protocol:ros-message)
  ((frame
    :reader frame
    :initarg :frame
    :type cl:integer
    :initform 0))
)

(cl:defclass setMmCtrlFrame-request (<setMmCtrlFrame-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setMmCtrlFrame-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setMmCtrlFrame-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<setMmCtrlFrame-request> is deprecated: use kuavo_msgs-srv:setMmCtrlFrame-request instead.")))

(cl:ensure-generic-function 'frame-val :lambda-list '(m))
(cl:defmethod frame-val ((m <setMmCtrlFrame-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:frame-val is deprecated.  Use kuavo_msgs-srv:frame instead.")
  (frame m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setMmCtrlFrame-request>) ostream)
  "Serializes a message object of type '<setMmCtrlFrame-request>"
  (cl:let* ((signed (cl:slot-value msg 'frame)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setMmCtrlFrame-request>) istream)
  "Deserializes a message object of type '<setMmCtrlFrame-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setMmCtrlFrame-request>)))
  "Returns string type for a service object of type '<setMmCtrlFrame-request>"
  "kuavo_msgs/setMmCtrlFrameRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setMmCtrlFrame-request)))
  "Returns string type for a service object of type 'setMmCtrlFrame-request"
  "kuavo_msgs/setMmCtrlFrameRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setMmCtrlFrame-request>)))
  "Returns md5sum for a message object of type '<setMmCtrlFrame-request>"
  "696a444d2580aa682923215a4a34937c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setMmCtrlFrame-request)))
  "Returns md5sum for a message object of type 'setMmCtrlFrame-request"
  "696a444d2580aa682923215a4a34937c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setMmCtrlFrame-request>)))
  "Returns full string definition for message of type '<setMmCtrlFrame-request>"
  (cl:format cl:nil "int32 frame~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setMmCtrlFrame-request)))
  "Returns full string definition for message of type 'setMmCtrlFrame-request"
  (cl:format cl:nil "int32 frame~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setMmCtrlFrame-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setMmCtrlFrame-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setMmCtrlFrame-request
    (cl:cons ':frame (frame msg))
))
;//! \htmlinclude setMmCtrlFrame-response.msg.html

(cl:defclass <setMmCtrlFrame-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil)
   (currentFrame
    :reader currentFrame
    :initarg :currentFrame
    :type cl:integer
    :initform 0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass setMmCtrlFrame-response (<setMmCtrlFrame-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setMmCtrlFrame-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setMmCtrlFrame-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<setMmCtrlFrame-response> is deprecated: use kuavo_msgs-srv:setMmCtrlFrame-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <setMmCtrlFrame-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:result-val is deprecated.  Use kuavo_msgs-srv:result instead.")
  (result m))

(cl:ensure-generic-function 'currentFrame-val :lambda-list '(m))
(cl:defmethod currentFrame-val ((m <setMmCtrlFrame-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:currentFrame-val is deprecated.  Use kuavo_msgs-srv:currentFrame instead.")
  (currentFrame m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <setMmCtrlFrame-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:message-val is deprecated.  Use kuavo_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setMmCtrlFrame-response>) ostream)
  "Serializes a message object of type '<setMmCtrlFrame-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'currentFrame)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setMmCtrlFrame-response>) istream)
  "Deserializes a message object of type '<setMmCtrlFrame-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'currentFrame) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setMmCtrlFrame-response>)))
  "Returns string type for a service object of type '<setMmCtrlFrame-response>"
  "kuavo_msgs/setMmCtrlFrameResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setMmCtrlFrame-response)))
  "Returns string type for a service object of type 'setMmCtrlFrame-response"
  "kuavo_msgs/setMmCtrlFrameResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setMmCtrlFrame-response>)))
  "Returns md5sum for a message object of type '<setMmCtrlFrame-response>"
  "696a444d2580aa682923215a4a34937c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setMmCtrlFrame-response)))
  "Returns md5sum for a message object of type 'setMmCtrlFrame-response"
  "696a444d2580aa682923215a4a34937c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setMmCtrlFrame-response>)))
  "Returns full string definition for message of type '<setMmCtrlFrame-response>"
  (cl:format cl:nil "bool result~%int32 currentFrame ~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setMmCtrlFrame-response)))
  "Returns full string definition for message of type 'setMmCtrlFrame-response"
  (cl:format cl:nil "bool result~%int32 currentFrame ~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setMmCtrlFrame-response>))
  (cl:+ 0
     1
     4
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setMmCtrlFrame-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setMmCtrlFrame-response
    (cl:cons ':result (result msg))
    (cl:cons ':currentFrame (currentFrame msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setMmCtrlFrame)))
  'setMmCtrlFrame-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setMmCtrlFrame)))
  'setMmCtrlFrame-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setMmCtrlFrame)))
  "Returns string type for a service object of type '<setMmCtrlFrame>"
  "kuavo_msgs/setMmCtrlFrame")