; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-srv)


;//! \htmlinclude controlLejuClaw-request.msg.html

(cl:defclass <controlLejuClaw-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type kuavo_msgs-msg:endEffectorData
    :initform (cl:make-instance 'kuavo_msgs-msg:endEffectorData)))
)

(cl:defclass controlLejuClaw-request (<controlLejuClaw-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <controlLejuClaw-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'controlLejuClaw-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<controlLejuClaw-request> is deprecated: use kuavo_msgs-srv:controlLejuClaw-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <controlLejuClaw-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:data-val is deprecated.  Use kuavo_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <controlLejuClaw-request>) ostream)
  "Serializes a message object of type '<controlLejuClaw-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <controlLejuClaw-request>) istream)
  "Deserializes a message object of type '<controlLejuClaw-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<controlLejuClaw-request>)))
  "Returns string type for a service object of type '<controlLejuClaw-request>"
  "kuavo_msgs/controlLejuClawRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'controlLejuClaw-request)))
  "Returns string type for a service object of type 'controlLejuClaw-request"
  "kuavo_msgs/controlLejuClawRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<controlLejuClaw-request>)))
  "Returns md5sum for a message object of type '<controlLejuClaw-request>"
  "674277f611b34c602b5afcc4b45849d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'controlLejuClaw-request)))
  "Returns md5sum for a message object of type 'controlLejuClaw-request"
  "674277f611b34c602b5afcc4b45849d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<controlLejuClaw-request>)))
  "Returns full string definition for message of type '<controlLejuClaw-request>"
  (cl:format cl:nil "~%# kuavo_msgs/endEffectorData:~%# string[] name  ~%# float64[] position~%# float64[] velocity  ~%# float64[] effort~%# ~%# ** For the Service Notes **~%# ~%# name     : 'left_claw' , 'right_claw'~%# position : 0 ~~ 100, the percentage of the claw's opening angle~%#            0: closed, 100: open   ~%# velocity : 0 ~~ 100, if size is 0, will use default `50.0`.~%# effort   : torque/current, better 1A ~~ 2A, if size is 0, will use default `1.0`.~%# ~%# ** Example **~%# Request:~%# data:~%#   - name: ['left_claw', 'right_claw']~%#     position: [20.0, 20.0]~%#     velocity: [50.0, 50.0]~%#     effort: [1.0, 1.0]~%#~%# Response:~%# success: True/False, call service success or not.~%# message: 'success'~%kuavo_msgs/endEffectorData data~%~%================================================================================~%MSG: kuavo_msgs/endEffectorData~%string[] name  ~%float64[] position~%float64[] velocity  ~%float64[] effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'controlLejuClaw-request)))
  "Returns full string definition for message of type 'controlLejuClaw-request"
  (cl:format cl:nil "~%# kuavo_msgs/endEffectorData:~%# string[] name  ~%# float64[] position~%# float64[] velocity  ~%# float64[] effort~%# ~%# ** For the Service Notes **~%# ~%# name     : 'left_claw' , 'right_claw'~%# position : 0 ~~ 100, the percentage of the claw's opening angle~%#            0: closed, 100: open   ~%# velocity : 0 ~~ 100, if size is 0, will use default `50.0`.~%# effort   : torque/current, better 1A ~~ 2A, if size is 0, will use default `1.0`.~%# ~%# ** Example **~%# Request:~%# data:~%#   - name: ['left_claw', 'right_claw']~%#     position: [20.0, 20.0]~%#     velocity: [50.0, 50.0]~%#     effort: [1.0, 1.0]~%#~%# Response:~%# success: True/False, call service success or not.~%# message: 'success'~%kuavo_msgs/endEffectorData data~%~%================================================================================~%MSG: kuavo_msgs/endEffectorData~%string[] name  ~%float64[] position~%float64[] velocity  ~%float64[] effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <controlLejuClaw-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <controlLejuClaw-request>))
  "Converts a ROS message object to a list"
  (cl:list 'controlLejuClaw-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude controlLejuClaw-response.msg.html

(cl:defclass <controlLejuClaw-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass controlLejuClaw-response (<controlLejuClaw-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <controlLejuClaw-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'controlLejuClaw-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<controlLejuClaw-response> is deprecated: use kuavo_msgs-srv:controlLejuClaw-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <controlLejuClaw-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:success-val is deprecated.  Use kuavo_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <controlLejuClaw-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:message-val is deprecated.  Use kuavo_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <controlLejuClaw-response>) ostream)
  "Serializes a message object of type '<controlLejuClaw-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <controlLejuClaw-response>) istream)
  "Deserializes a message object of type '<controlLejuClaw-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<controlLejuClaw-response>)))
  "Returns string type for a service object of type '<controlLejuClaw-response>"
  "kuavo_msgs/controlLejuClawResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'controlLejuClaw-response)))
  "Returns string type for a service object of type 'controlLejuClaw-response"
  "kuavo_msgs/controlLejuClawResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<controlLejuClaw-response>)))
  "Returns md5sum for a message object of type '<controlLejuClaw-response>"
  "674277f611b34c602b5afcc4b45849d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'controlLejuClaw-response)))
  "Returns md5sum for a message object of type 'controlLejuClaw-response"
  "674277f611b34c602b5afcc4b45849d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<controlLejuClaw-response>)))
  "Returns full string definition for message of type '<controlLejuClaw-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'controlLejuClaw-response)))
  "Returns full string definition for message of type 'controlLejuClaw-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <controlLejuClaw-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <controlLejuClaw-response>))
  "Converts a ROS message object to a list"
  (cl:list 'controlLejuClaw-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'controlLejuClaw)))
  'controlLejuClaw-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'controlLejuClaw)))
  'controlLejuClaw-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'controlLejuClaw)))
  "Returns string type for a service object of type '<controlLejuClaw>"
  "kuavo_msgs/controlLejuClaw")