; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-srv)


;//! \htmlinclude enableHandTouchSensor-request.msg.html

(cl:defclass <enableHandTouchSensor-request> (roslisp-msg-protocol:ros-message)
  ((mask
    :reader mask
    :initarg :mask
    :type cl:fixnum
    :initform 0))
)

(cl:defclass enableHandTouchSensor-request (<enableHandTouchSensor-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <enableHandTouchSensor-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'enableHandTouchSensor-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<enableHandTouchSensor-request> is deprecated: use kuavo_msgs-srv:enableHandTouchSensor-request instead.")))

(cl:ensure-generic-function 'mask-val :lambda-list '(m))
(cl:defmethod mask-val ((m <enableHandTouchSensor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:mask-val is deprecated.  Use kuavo_msgs-srv:mask instead.")
  (mask m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<enableHandTouchSensor-request>)))
    "Constants for message type '<enableHandTouchSensor-request>"
  '((:THUMB_SENSOR . 1)
    (:INDEX_SENSOR . 2)
    (:MIDDLE_SENSOR . 4)
    (:RING_SENSOR . 8)
    (:PINKY_SENSOR . 16))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'enableHandTouchSensor-request)))
    "Constants for message type 'enableHandTouchSensor-request"
  '((:THUMB_SENSOR . 1)
    (:INDEX_SENSOR . 2)
    (:MIDDLE_SENSOR . 4)
    (:RING_SENSOR . 8)
    (:PINKY_SENSOR . 16))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <enableHandTouchSensor-request>) ostream)
  "Serializes a message object of type '<enableHandTouchSensor-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mask)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <enableHandTouchSensor-request>) istream)
  "Deserializes a message object of type '<enableHandTouchSensor-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mask)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<enableHandTouchSensor-request>)))
  "Returns string type for a service object of type '<enableHandTouchSensor-request>"
  "kuavo_msgs/enableHandTouchSensorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'enableHandTouchSensor-request)))
  "Returns string type for a service object of type 'enableHandTouchSensor-request"
  "kuavo_msgs/enableHandTouchSensorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<enableHandTouchSensor-request>)))
  "Returns md5sum for a message object of type '<enableHandTouchSensor-request>"
  "710f3d70d245856e41b01f7ffef21580")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'enableHandTouchSensor-request)))
  "Returns md5sum for a message object of type 'enableHandTouchSensor-request"
  "710f3d70d245856e41b01f7ffef21580")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<enableHandTouchSensor-request>)))
  "Returns full string definition for message of type '<enableHandTouchSensor-request>"
  (cl:format cl:nil "# Bit masks for enabling individual touch sensors on the robotic hand~%# Each sensor can be enabled by setting the corresponding bit in the mask~%~%# Examples: 0b00000011 enables thumb and index sensors, 0b00000000 disables all sensors~%#  ``` ~%#    mask_value = THUMB_SENSOR | INDEX_SENSOR~%#    req = enableHandTouchSensorRequest()~%#    req.mask = mask_value~%#  ``` ~%~%# Thumb finger touch sensor (bit 0)~%uint8 THUMB_SENSOR = 1~%~%# Index finger touch sensor (bit 1)~%uint8 INDEX_SENSOR = 2~%~%# Middle finger touch sensor (bit 2)~%uint8 MIDDLE_SENSOR = 4~%~%# Ring finger touch sensor (bit 3)~%uint8 RING_SENSOR = 8~%~%# Pinky finger touch sensor (bit 4)~%uint8 PINKY_SENSOR = 16~%~%# Bitmask indicating which sensors to enable~%# Multiple sensors can be enabled by combining masks with bitwise OR~%uint8 mask~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'enableHandTouchSensor-request)))
  "Returns full string definition for message of type 'enableHandTouchSensor-request"
  (cl:format cl:nil "# Bit masks for enabling individual touch sensors on the robotic hand~%# Each sensor can be enabled by setting the corresponding bit in the mask~%~%# Examples: 0b00000011 enables thumb and index sensors, 0b00000000 disables all sensors~%#  ``` ~%#    mask_value = THUMB_SENSOR | INDEX_SENSOR~%#    req = enableHandTouchSensorRequest()~%#    req.mask = mask_value~%#  ``` ~%~%# Thumb finger touch sensor (bit 0)~%uint8 THUMB_SENSOR = 1~%~%# Index finger touch sensor (bit 1)~%uint8 INDEX_SENSOR = 2~%~%# Middle finger touch sensor (bit 2)~%uint8 MIDDLE_SENSOR = 4~%~%# Ring finger touch sensor (bit 3)~%uint8 RING_SENSOR = 8~%~%# Pinky finger touch sensor (bit 4)~%uint8 PINKY_SENSOR = 16~%~%# Bitmask indicating which sensors to enable~%# Multiple sensors can be enabled by combining masks with bitwise OR~%uint8 mask~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <enableHandTouchSensor-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <enableHandTouchSensor-request>))
  "Converts a ROS message object to a list"
  (cl:list 'enableHandTouchSensor-request
    (cl:cons ':mask (mask msg))
))
;//! \htmlinclude enableHandTouchSensor-response.msg.html

(cl:defclass <enableHandTouchSensor-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass enableHandTouchSensor-response (<enableHandTouchSensor-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <enableHandTouchSensor-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'enableHandTouchSensor-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<enableHandTouchSensor-response> is deprecated: use kuavo_msgs-srv:enableHandTouchSensor-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <enableHandTouchSensor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:success-val is deprecated.  Use kuavo_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <enableHandTouchSensor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:message-val is deprecated.  Use kuavo_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <enableHandTouchSensor-response>) ostream)
  "Serializes a message object of type '<enableHandTouchSensor-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <enableHandTouchSensor-response>) istream)
  "Deserializes a message object of type '<enableHandTouchSensor-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<enableHandTouchSensor-response>)))
  "Returns string type for a service object of type '<enableHandTouchSensor-response>"
  "kuavo_msgs/enableHandTouchSensorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'enableHandTouchSensor-response)))
  "Returns string type for a service object of type 'enableHandTouchSensor-response"
  "kuavo_msgs/enableHandTouchSensorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<enableHandTouchSensor-response>)))
  "Returns md5sum for a message object of type '<enableHandTouchSensor-response>"
  "710f3d70d245856e41b01f7ffef21580")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'enableHandTouchSensor-response)))
  "Returns md5sum for a message object of type 'enableHandTouchSensor-response"
  "710f3d70d245856e41b01f7ffef21580")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<enableHandTouchSensor-response>)))
  "Returns full string definition for message of type '<enableHandTouchSensor-response>"
  (cl:format cl:nil "# Whether the operation was successful~%bool success~%~%# Additional status or error message~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'enableHandTouchSensor-response)))
  "Returns full string definition for message of type 'enableHandTouchSensor-response"
  (cl:format cl:nil "# Whether the operation was successful~%bool success~%~%# Additional status or error message~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <enableHandTouchSensor-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <enableHandTouchSensor-response>))
  "Converts a ROS message object to a list"
  (cl:list 'enableHandTouchSensor-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'enableHandTouchSensor)))
  'enableHandTouchSensor-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'enableHandTouchSensor)))
  'enableHandTouchSensor-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'enableHandTouchSensor)))
  "Returns string type for a service object of type '<enableHandTouchSensor>"
  "kuavo_msgs/enableHandTouchSensor")