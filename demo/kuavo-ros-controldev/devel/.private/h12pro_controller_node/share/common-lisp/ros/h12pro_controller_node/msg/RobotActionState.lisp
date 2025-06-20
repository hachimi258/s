; Auto-generated. Do not edit!


(cl:in-package h12pro_controller_node-msg)


;//! \htmlinclude RobotActionState.msg.html

(cl:defclass <RobotActionState> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RobotActionState (<RobotActionState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotActionState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotActionState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name h12pro_controller_node-msg:<RobotActionState> is deprecated: use h12pro_controller_node-msg:RobotActionState instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <RobotActionState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader h12pro_controller_node-msg:state-val is deprecated.  Use h12pro_controller_node-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotActionState>) ostream)
  "Serializes a message object of type '<RobotActionState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotActionState>) istream)
  "Deserializes a message object of type '<RobotActionState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotActionState>)))
  "Returns string type for a message object of type '<RobotActionState>"
  "h12pro_controller_node/RobotActionState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotActionState)))
  "Returns string type for a message object of type 'RobotActionState"
  "h12pro_controller_node/RobotActionState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotActionState>)))
  "Returns md5sum for a message object of type '<RobotActionState>"
  "800f34bc468def1d86e2d42bea5648c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotActionState)))
  "Returns md5sum for a message object of type 'RobotActionState"
  "800f34bc468def1d86e2d42bea5648c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotActionState>)))
  "Returns full string definition for message of type '<RobotActionState>"
  (cl:format cl:nil "uint8 state  ~%# 0:失败 ()~%# 1:执行中~%# 2:成功~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotActionState)))
  "Returns full string definition for message of type 'RobotActionState"
  (cl:format cl:nil "uint8 state  ~%# 0:失败 ()~%# 1:执行中~%# 2:成功~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotActionState>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotActionState>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotActionState
    (cl:cons ':state (state msg))
))
