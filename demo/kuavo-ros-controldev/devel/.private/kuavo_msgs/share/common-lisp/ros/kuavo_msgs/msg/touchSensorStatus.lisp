; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-msg)


;//! \htmlinclude touchSensorStatus.msg.html

(cl:defclass <touchSensorStatus> (roslisp-msg-protocol:ros-message)
  ((normal_force1
    :reader normal_force1
    :initarg :normal_force1
    :type cl:fixnum
    :initform 0)
   (normal_force2
    :reader normal_force2
    :initarg :normal_force2
    :type cl:fixnum
    :initform 0)
   (normal_force3
    :reader normal_force3
    :initarg :normal_force3
    :type cl:fixnum
    :initform 0)
   (tangential_force1
    :reader tangential_force1
    :initarg :tangential_force1
    :type cl:fixnum
    :initform 0)
   (tangential_force2
    :reader tangential_force2
    :initarg :tangential_force2
    :type cl:fixnum
    :initform 0)
   (tangential_force3
    :reader tangential_force3
    :initarg :tangential_force3
    :type cl:fixnum
    :initform 0)
   (tangential_direction1
    :reader tangential_direction1
    :initarg :tangential_direction1
    :type cl:fixnum
    :initform 0)
   (tangential_direction2
    :reader tangential_direction2
    :initarg :tangential_direction2
    :type cl:fixnum
    :initform 0)
   (tangential_direction3
    :reader tangential_direction3
    :initarg :tangential_direction3
    :type cl:fixnum
    :initform 0)
   (self_proximity1
    :reader self_proximity1
    :initarg :self_proximity1
    :type cl:integer
    :initform 0)
   (self_proximity2
    :reader self_proximity2
    :initarg :self_proximity2
    :type cl:integer
    :initform 0)
   (mutual_proximity
    :reader mutual_proximity
    :initarg :mutual_proximity
    :type cl:integer
    :initform 0)
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass touchSensorStatus (<touchSensorStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <touchSensorStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'touchSensorStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-msg:<touchSensorStatus> is deprecated: use kuavo_msgs-msg:touchSensorStatus instead.")))

(cl:ensure-generic-function 'normal_force1-val :lambda-list '(m))
(cl:defmethod normal_force1-val ((m <touchSensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:normal_force1-val is deprecated.  Use kuavo_msgs-msg:normal_force1 instead.")
  (normal_force1 m))

(cl:ensure-generic-function 'normal_force2-val :lambda-list '(m))
(cl:defmethod normal_force2-val ((m <touchSensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:normal_force2-val is deprecated.  Use kuavo_msgs-msg:normal_force2 instead.")
  (normal_force2 m))

(cl:ensure-generic-function 'normal_force3-val :lambda-list '(m))
(cl:defmethod normal_force3-val ((m <touchSensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:normal_force3-val is deprecated.  Use kuavo_msgs-msg:normal_force3 instead.")
  (normal_force3 m))

(cl:ensure-generic-function 'tangential_force1-val :lambda-list '(m))
(cl:defmethod tangential_force1-val ((m <touchSensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:tangential_force1-val is deprecated.  Use kuavo_msgs-msg:tangential_force1 instead.")
  (tangential_force1 m))

(cl:ensure-generic-function 'tangential_force2-val :lambda-list '(m))
(cl:defmethod tangential_force2-val ((m <touchSensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:tangential_force2-val is deprecated.  Use kuavo_msgs-msg:tangential_force2 instead.")
  (tangential_force2 m))

(cl:ensure-generic-function 'tangential_force3-val :lambda-list '(m))
(cl:defmethod tangential_force3-val ((m <touchSensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:tangential_force3-val is deprecated.  Use kuavo_msgs-msg:tangential_force3 instead.")
  (tangential_force3 m))

(cl:ensure-generic-function 'tangential_direction1-val :lambda-list '(m))
(cl:defmethod tangential_direction1-val ((m <touchSensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:tangential_direction1-val is deprecated.  Use kuavo_msgs-msg:tangential_direction1 instead.")
  (tangential_direction1 m))

(cl:ensure-generic-function 'tangential_direction2-val :lambda-list '(m))
(cl:defmethod tangential_direction2-val ((m <touchSensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:tangential_direction2-val is deprecated.  Use kuavo_msgs-msg:tangential_direction2 instead.")
  (tangential_direction2 m))

(cl:ensure-generic-function 'tangential_direction3-val :lambda-list '(m))
(cl:defmethod tangential_direction3-val ((m <touchSensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:tangential_direction3-val is deprecated.  Use kuavo_msgs-msg:tangential_direction3 instead.")
  (tangential_direction3 m))

(cl:ensure-generic-function 'self_proximity1-val :lambda-list '(m))
(cl:defmethod self_proximity1-val ((m <touchSensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:self_proximity1-val is deprecated.  Use kuavo_msgs-msg:self_proximity1 instead.")
  (self_proximity1 m))

(cl:ensure-generic-function 'self_proximity2-val :lambda-list '(m))
(cl:defmethod self_proximity2-val ((m <touchSensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:self_proximity2-val is deprecated.  Use kuavo_msgs-msg:self_proximity2 instead.")
  (self_proximity2 m))

(cl:ensure-generic-function 'mutual_proximity-val :lambda-list '(m))
(cl:defmethod mutual_proximity-val ((m <touchSensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:mutual_proximity-val is deprecated.  Use kuavo_msgs-msg:mutual_proximity instead.")
  (mutual_proximity m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <touchSensorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:status-val is deprecated.  Use kuavo_msgs-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <touchSensorStatus>) ostream)
  "Serializes a message object of type '<touchSensorStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'normal_force1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'normal_force1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'normal_force2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'normal_force2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'normal_force3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'normal_force3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tangential_force1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tangential_force1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tangential_force2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tangential_force2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tangential_force3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tangential_force3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tangential_direction1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tangential_direction1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tangential_direction2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tangential_direction2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tangential_direction3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tangential_direction3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'self_proximity1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'self_proximity1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'self_proximity1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'self_proximity1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'self_proximity2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'self_proximity2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'self_proximity2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'self_proximity2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mutual_proximity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mutual_proximity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mutual_proximity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mutual_proximity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <touchSensorStatus>) istream)
  "Deserializes a message object of type '<touchSensorStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'normal_force1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'normal_force1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'normal_force2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'normal_force2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'normal_force3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'normal_force3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tangential_force1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tangential_force1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tangential_force2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tangential_force2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tangential_force3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tangential_force3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tangential_direction1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tangential_direction1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tangential_direction2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tangential_direction2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tangential_direction3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tangential_direction3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'self_proximity1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'self_proximity1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'self_proximity1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'self_proximity1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'self_proximity2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'self_proximity2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'self_proximity2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'self_proximity2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mutual_proximity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mutual_proximity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mutual_proximity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mutual_proximity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<touchSensorStatus>)))
  "Returns string type for a message object of type '<touchSensorStatus>"
  "kuavo_msgs/touchSensorStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'touchSensorStatus)))
  "Returns string type for a message object of type 'touchSensorStatus"
  "kuavo_msgs/touchSensorStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<touchSensorStatus>)))
  "Returns md5sum for a message object of type '<touchSensorStatus>"
  "08cd59dc396363cba4d4f01df99ec86c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'touchSensorStatus)))
  "Returns md5sum for a message object of type 'touchSensorStatus"
  "08cd59dc396363cba4d4f01df99ec86c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<touchSensorStatus>)))
  "Returns full string definition for message of type '<touchSensorStatus>"
  (cl:format cl:nil "uint16 normal_force1  # 法向力1~%uint16 normal_force2  # 法向力2~%uint16 normal_force3  # 法向力3~%uint16 tangential_force1  # 切向力1~%uint16 tangential_force2  # 切向力2~%uint16 tangential_force3  # 切向力3~%uint16 tangential_direction1  # 切向力方向1~%uint16 tangential_direction2  # 切向力方向2~%uint16 tangential_direction3  # 切向力方向3~%uint32 self_proximity1  # 自电容接近传感器1~%uint32 self_proximity2  # 自电容接近传感器2~%uint32 mutual_proximity  # 互电容接近传感器~%uint16 status  # 传感器状态~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'touchSensorStatus)))
  "Returns full string definition for message of type 'touchSensorStatus"
  (cl:format cl:nil "uint16 normal_force1  # 法向力1~%uint16 normal_force2  # 法向力2~%uint16 normal_force3  # 法向力3~%uint16 tangential_force1  # 切向力1~%uint16 tangential_force2  # 切向力2~%uint16 tangential_force3  # 切向力3~%uint16 tangential_direction1  # 切向力方向1~%uint16 tangential_direction2  # 切向力方向2~%uint16 tangential_direction3  # 切向力方向3~%uint32 self_proximity1  # 自电容接近传感器1~%uint32 self_proximity2  # 自电容接近传感器2~%uint32 mutual_proximity  # 互电容接近传感器~%uint16 status  # 传感器状态~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <touchSensorStatus>))
  (cl:+ 0
     2
     2
     2
     2
     2
     2
     2
     2
     2
     4
     4
     4
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <touchSensorStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'touchSensorStatus
    (cl:cons ':normal_force1 (normal_force1 msg))
    (cl:cons ':normal_force2 (normal_force2 msg))
    (cl:cons ':normal_force3 (normal_force3 msg))
    (cl:cons ':tangential_force1 (tangential_force1 msg))
    (cl:cons ':tangential_force2 (tangential_force2 msg))
    (cl:cons ':tangential_force3 (tangential_force3 msg))
    (cl:cons ':tangential_direction1 (tangential_direction1 msg))
    (cl:cons ':tangential_direction2 (tangential_direction2 msg))
    (cl:cons ':tangential_direction3 (tangential_direction3 msg))
    (cl:cons ':self_proximity1 (self_proximity1 msg))
    (cl:cons ':self_proximity2 (self_proximity2 msg))
    (cl:cons ':mutual_proximity (mutual_proximity msg))
    (cl:cons ':status (status msg))
))
