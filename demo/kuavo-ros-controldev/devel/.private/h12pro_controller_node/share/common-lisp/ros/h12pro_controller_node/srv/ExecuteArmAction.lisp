; Auto-generated. Do not edit!


(cl:in-package h12pro_controller_node-srv)


;//! \htmlinclude ExecuteArmAction-request.msg.html

(cl:defclass <ExecuteArmAction-request> (roslisp-msg-protocol:ros-message)
  ((action_name
    :reader action_name
    :initarg :action_name
    :type cl:string
    :initform ""))
)

(cl:defclass ExecuteArmAction-request (<ExecuteArmAction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteArmAction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteArmAction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name h12pro_controller_node-srv:<ExecuteArmAction-request> is deprecated: use h12pro_controller_node-srv:ExecuteArmAction-request instead.")))

(cl:ensure-generic-function 'action_name-val :lambda-list '(m))
(cl:defmethod action_name-val ((m <ExecuteArmAction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader h12pro_controller_node-srv:action_name-val is deprecated.  Use h12pro_controller_node-srv:action_name instead.")
  (action_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteArmAction-request>) ostream)
  "Serializes a message object of type '<ExecuteArmAction-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'action_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'action_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteArmAction-request>) istream)
  "Deserializes a message object of type '<ExecuteArmAction-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'action_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteArmAction-request>)))
  "Returns string type for a service object of type '<ExecuteArmAction-request>"
  "h12pro_controller_node/ExecuteArmActionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteArmAction-request)))
  "Returns string type for a service object of type 'ExecuteArmAction-request"
  "h12pro_controller_node/ExecuteArmActionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteArmAction-request>)))
  "Returns md5sum for a message object of type '<ExecuteArmAction-request>"
  "a29ecc17b1498502f1af3d6bb5a90a24")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteArmAction-request)))
  "Returns md5sum for a message object of type 'ExecuteArmAction-request"
  "a29ecc17b1498502f1af3d6bb5a90a24")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteArmAction-request>)))
  "Returns full string definition for message of type '<ExecuteArmAction-request>"
  (cl:format cl:nil "string action_name  # 要执行的动作名称，例如 \"welcome\"~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteArmAction-request)))
  "Returns full string definition for message of type 'ExecuteArmAction-request"
  (cl:format cl:nil "string action_name  # 要执行的动作名称，例如 \"welcome\"~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteArmAction-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'action_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteArmAction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteArmAction-request
    (cl:cons ':action_name (action_name msg))
))
;//! \htmlinclude ExecuteArmAction-response.msg.html

(cl:defclass <ExecuteArmAction-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass ExecuteArmAction-response (<ExecuteArmAction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteArmAction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteArmAction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name h12pro_controller_node-srv:<ExecuteArmAction-response> is deprecated: use h12pro_controller_node-srv:ExecuteArmAction-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ExecuteArmAction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader h12pro_controller_node-srv:success-val is deprecated.  Use h12pro_controller_node-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ExecuteArmAction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader h12pro_controller_node-srv:message-val is deprecated.  Use h12pro_controller_node-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteArmAction-response>) ostream)
  "Serializes a message object of type '<ExecuteArmAction-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteArmAction-response>) istream)
  "Deserializes a message object of type '<ExecuteArmAction-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteArmAction-response>)))
  "Returns string type for a service object of type '<ExecuteArmAction-response>"
  "h12pro_controller_node/ExecuteArmActionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteArmAction-response)))
  "Returns string type for a service object of type 'ExecuteArmAction-response"
  "h12pro_controller_node/ExecuteArmActionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteArmAction-response>)))
  "Returns md5sum for a message object of type '<ExecuteArmAction-response>"
  "a29ecc17b1498502f1af3d6bb5a90a24")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteArmAction-response)))
  "Returns md5sum for a message object of type 'ExecuteArmAction-response"
  "a29ecc17b1498502f1af3d6bb5a90a24")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteArmAction-response>)))
  "Returns full string definition for message of type '<ExecuteArmAction-response>"
  (cl:format cl:nil "bool success         # 执行是否成功~%string message       # 返回的消息~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteArmAction-response)))
  "Returns full string definition for message of type 'ExecuteArmAction-response"
  (cl:format cl:nil "bool success         # 执行是否成功~%string message       # 返回的消息~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteArmAction-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteArmAction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteArmAction-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ExecuteArmAction)))
  'ExecuteArmAction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ExecuteArmAction)))
  'ExecuteArmAction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteArmAction)))
  "Returns string type for a service object of type '<ExecuteArmAction>"
  "h12pro_controller_node/ExecuteArmAction")