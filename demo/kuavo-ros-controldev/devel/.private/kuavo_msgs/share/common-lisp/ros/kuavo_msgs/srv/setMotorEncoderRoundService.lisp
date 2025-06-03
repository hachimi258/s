; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-srv)


;//! \htmlinclude setMotorEncoderRoundService-request.msg.html

(cl:defclass <setMotorEncoderRoundService-request> (roslisp-msg-protocol:ros-message)
  ((motor_id
    :reader motor_id
    :initarg :motor_id
    :type cl:integer
    :initform 0)
   (direction
    :reader direction
    :initarg :direction
    :type cl:integer
    :initform 0)
   (save_offset
    :reader save_offset
    :initarg :save_offset
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setMotorEncoderRoundService-request (<setMotorEncoderRoundService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setMotorEncoderRoundService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setMotorEncoderRoundService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<setMotorEncoderRoundService-request> is deprecated: use kuavo_msgs-srv:setMotorEncoderRoundService-request instead.")))

(cl:ensure-generic-function 'motor_id-val :lambda-list '(m))
(cl:defmethod motor_id-val ((m <setMotorEncoderRoundService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:motor_id-val is deprecated.  Use kuavo_msgs-srv:motor_id instead.")
  (motor_id m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <setMotorEncoderRoundService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:direction-val is deprecated.  Use kuavo_msgs-srv:direction instead.")
  (direction m))

(cl:ensure-generic-function 'save_offset-val :lambda-list '(m))
(cl:defmethod save_offset-val ((m <setMotorEncoderRoundService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:save_offset-val is deprecated.  Use kuavo_msgs-srv:save_offset instead.")
  (save_offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setMotorEncoderRoundService-request>) ostream)
  "Serializes a message object of type '<setMotorEncoderRoundService-request>"
  (cl:let* ((signed (cl:slot-value msg 'motor_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'direction)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'save_offset) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setMotorEncoderRoundService-request>) istream)
  "Deserializes a message object of type '<setMotorEncoderRoundService-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'save_offset) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setMotorEncoderRoundService-request>)))
  "Returns string type for a service object of type '<setMotorEncoderRoundService-request>"
  "kuavo_msgs/setMotorEncoderRoundServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setMotorEncoderRoundService-request)))
  "Returns string type for a service object of type 'setMotorEncoderRoundService-request"
  "kuavo_msgs/setMotorEncoderRoundServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setMotorEncoderRoundService-request>)))
  "Returns md5sum for a message object of type '<setMotorEncoderRoundService-request>"
  "c8e2df10122ff9a180b403f54a3f8831")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setMotorEncoderRoundService-request)))
  "Returns md5sum for a message object of type 'setMotorEncoderRoundService-request"
  "c8e2df10122ff9a180b403f54a3f8831")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setMotorEncoderRoundService-request>)))
  "Returns full string definition for message of type '<setMotorEncoderRoundService-request>"
  (cl:format cl:nil "# 设置电机编码器圈数，用于单编码器版本的校准~%# 传入参数~%int32 motor_id     # 电机 ID~%int32 direction    # 方向，1 表示顺时针+1圈，-1 表示逆时针-1圈~%bool save_offset   # 是否保存偏移量到文件~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setMotorEncoderRoundService-request)))
  "Returns full string definition for message of type 'setMotorEncoderRoundService-request"
  (cl:format cl:nil "# 设置电机编码器圈数，用于单编码器版本的校准~%# 传入参数~%int32 motor_id     # 电机 ID~%int32 direction    # 方向，1 表示顺时针+1圈，-1 表示逆时针-1圈~%bool save_offset   # 是否保存偏移量到文件~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setMotorEncoderRoundService-request>))
  (cl:+ 0
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setMotorEncoderRoundService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setMotorEncoderRoundService-request
    (cl:cons ':motor_id (motor_id msg))
    (cl:cons ':direction (direction msg))
    (cl:cons ':save_offset (save_offset msg))
))
;//! \htmlinclude setMotorEncoderRoundService-response.msg.html

(cl:defclass <setMotorEncoderRoundService-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass setMotorEncoderRoundService-response (<setMotorEncoderRoundService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setMotorEncoderRoundService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setMotorEncoderRoundService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<setMotorEncoderRoundService-response> is deprecated: use kuavo_msgs-srv:setMotorEncoderRoundService-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <setMotorEncoderRoundService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:success-val is deprecated.  Use kuavo_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <setMotorEncoderRoundService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:message-val is deprecated.  Use kuavo_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setMotorEncoderRoundService-response>) ostream)
  "Serializes a message object of type '<setMotorEncoderRoundService-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setMotorEncoderRoundService-response>) istream)
  "Deserializes a message object of type '<setMotorEncoderRoundService-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setMotorEncoderRoundService-response>)))
  "Returns string type for a service object of type '<setMotorEncoderRoundService-response>"
  "kuavo_msgs/setMotorEncoderRoundServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setMotorEncoderRoundService-response)))
  "Returns string type for a service object of type 'setMotorEncoderRoundService-response"
  "kuavo_msgs/setMotorEncoderRoundServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setMotorEncoderRoundService-response>)))
  "Returns md5sum for a message object of type '<setMotorEncoderRoundService-response>"
  "c8e2df10122ff9a180b403f54a3f8831")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setMotorEncoderRoundService-response)))
  "Returns md5sum for a message object of type 'setMotorEncoderRoundService-response"
  "c8e2df10122ff9a180b403f54a3f8831")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setMotorEncoderRoundService-response>)))
  "Returns full string definition for message of type '<setMotorEncoderRoundService-response>"
  (cl:format cl:nil "# 返回结果~%bool success       # 成功与否~%string message     # 返回信息~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setMotorEncoderRoundService-response)))
  "Returns full string definition for message of type 'setMotorEncoderRoundService-response"
  (cl:format cl:nil "# 返回结果~%bool success       # 成功与否~%string message     # 返回信息~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setMotorEncoderRoundService-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setMotorEncoderRoundService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setMotorEncoderRoundService-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setMotorEncoderRoundService)))
  'setMotorEncoderRoundService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setMotorEncoderRoundService)))
  'setMotorEncoderRoundService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setMotorEncoderRoundService)))
  "Returns string type for a service object of type '<setMotorEncoderRoundService>"
  "kuavo_msgs/setMotorEncoderRoundService")