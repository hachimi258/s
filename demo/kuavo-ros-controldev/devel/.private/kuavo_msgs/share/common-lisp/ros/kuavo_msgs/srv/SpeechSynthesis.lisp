; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-srv)


;//! \htmlinclude SpeechSynthesis-request.msg.html

(cl:defclass <SpeechSynthesis-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:string
    :initform "")
   (volume
    :reader volume
    :initarg :volume
    :type cl:float
    :initform 0.0))
)

(cl:defclass SpeechSynthesis-request (<SpeechSynthesis-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeechSynthesis-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeechSynthesis-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<SpeechSynthesis-request> is deprecated: use kuavo_msgs-srv:SpeechSynthesis-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SpeechSynthesis-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:data-val is deprecated.  Use kuavo_msgs-srv:data instead.")
  (data m))

(cl:ensure-generic-function 'volume-val :lambda-list '(m))
(cl:defmethod volume-val ((m <SpeechSynthesis-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:volume-val is deprecated.  Use kuavo_msgs-srv:volume instead.")
  (volume m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeechSynthesis-request>) ostream)
  "Serializes a message object of type '<SpeechSynthesis-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'volume))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeechSynthesis-request>) istream)
  "Deserializes a message object of type '<SpeechSynthesis-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'volume) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeechSynthesis-request>)))
  "Returns string type for a service object of type '<SpeechSynthesis-request>"
  "kuavo_msgs/SpeechSynthesisRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeechSynthesis-request)))
  "Returns string type for a service object of type 'SpeechSynthesis-request"
  "kuavo_msgs/SpeechSynthesisRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeechSynthesis-request>)))
  "Returns md5sum for a message object of type '<SpeechSynthesis-request>"
  "c2d03ac0ff2ddd1cb86a5148ad0f9d5e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeechSynthesis-request)))
  "Returns md5sum for a message object of type 'SpeechSynthesis-request"
  "c2d03ac0ff2ddd1cb86a5148ad0f9d5e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeechSynthesis-request>)))
  "Returns full string definition for message of type '<SpeechSynthesis-request>"
  (cl:format cl:nil "# 请求部分~%string data~%float32 volume~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeechSynthesis-request)))
  "Returns full string definition for message of type 'SpeechSynthesis-request"
  (cl:format cl:nil "# 请求部分~%string data~%float32 volume~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeechSynthesis-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'data))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeechSynthesis-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeechSynthesis-request
    (cl:cons ':data (data msg))
    (cl:cons ':volume (volume msg))
))
;//! \htmlinclude SpeechSynthesis-response.msg.html

(cl:defclass <SpeechSynthesis-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SpeechSynthesis-response (<SpeechSynthesis-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeechSynthesis-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeechSynthesis-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<SpeechSynthesis-response> is deprecated: use kuavo_msgs-srv:SpeechSynthesis-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SpeechSynthesis-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:success-val is deprecated.  Use kuavo_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeechSynthesis-response>) ostream)
  "Serializes a message object of type '<SpeechSynthesis-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeechSynthesis-response>) istream)
  "Deserializes a message object of type '<SpeechSynthesis-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeechSynthesis-response>)))
  "Returns string type for a service object of type '<SpeechSynthesis-response>"
  "kuavo_msgs/SpeechSynthesisResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeechSynthesis-response)))
  "Returns string type for a service object of type 'SpeechSynthesis-response"
  "kuavo_msgs/SpeechSynthesisResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeechSynthesis-response>)))
  "Returns md5sum for a message object of type '<SpeechSynthesis-response>"
  "c2d03ac0ff2ddd1cb86a5148ad0f9d5e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeechSynthesis-response)))
  "Returns md5sum for a message object of type 'SpeechSynthesis-response"
  "c2d03ac0ff2ddd1cb86a5148ad0f9d5e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeechSynthesis-response>)))
  "Returns full string definition for message of type '<SpeechSynthesis-response>"
  (cl:format cl:nil "# 响应部分~%bool success   ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeechSynthesis-response)))
  "Returns full string definition for message of type 'SpeechSynthesis-response"
  (cl:format cl:nil "# 响应部分~%bool success   ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeechSynthesis-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeechSynthesis-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeechSynthesis-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SpeechSynthesis)))
  'SpeechSynthesis-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SpeechSynthesis)))
  'SpeechSynthesis-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeechSynthesis)))
  "Returns string type for a service object of type '<SpeechSynthesis>"
  "kuavo_msgs/SpeechSynthesis")