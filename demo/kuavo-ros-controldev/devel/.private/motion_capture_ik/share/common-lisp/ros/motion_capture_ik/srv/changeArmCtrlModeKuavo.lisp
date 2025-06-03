; Auto-generated. Do not edit!


(cl:in-package motion_capture_ik-srv)


;//! \htmlinclude changeArmCtrlModeKuavo-request.msg.html

(cl:defclass <changeArmCtrlModeKuavo-request> (roslisp-msg-protocol:ros-message)
  ((control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass changeArmCtrlModeKuavo-request (<changeArmCtrlModeKuavo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <changeArmCtrlModeKuavo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'changeArmCtrlModeKuavo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_capture_ik-srv:<changeArmCtrlModeKuavo-request> is deprecated: use motion_capture_ik-srv:changeArmCtrlModeKuavo-request instead.")))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <changeArmCtrlModeKuavo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_capture_ik-srv:control_mode-val is deprecated.  Use motion_capture_ik-srv:control_mode instead.")
  (control_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <changeArmCtrlModeKuavo-request>) ostream)
  "Serializes a message object of type '<changeArmCtrlModeKuavo-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'control_mode) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <changeArmCtrlModeKuavo-request>) istream)
  "Deserializes a message object of type '<changeArmCtrlModeKuavo-request>"
    (cl:setf (cl:slot-value msg 'control_mode) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<changeArmCtrlModeKuavo-request>)))
  "Returns string type for a service object of type '<changeArmCtrlModeKuavo-request>"
  "motion_capture_ik/changeArmCtrlModeKuavoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'changeArmCtrlModeKuavo-request)))
  "Returns string type for a service object of type 'changeArmCtrlModeKuavo-request"
  "motion_capture_ik/changeArmCtrlModeKuavoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<changeArmCtrlModeKuavo-request>)))
  "Returns md5sum for a message object of type '<changeArmCtrlModeKuavo-request>"
  "f89438f9d6f48f748eabe64775a22261")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'changeArmCtrlModeKuavo-request)))
  "Returns md5sum for a message object of type 'changeArmCtrlModeKuavo-request"
  "f89438f9d6f48f748eabe64775a22261")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<changeArmCtrlModeKuavo-request>)))
  "Returns full string definition for message of type '<changeArmCtrlModeKuavo-request>"
  (cl:format cl:nil "bool control_mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'changeArmCtrlModeKuavo-request)))
  "Returns full string definition for message of type 'changeArmCtrlModeKuavo-request"
  (cl:format cl:nil "bool control_mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <changeArmCtrlModeKuavo-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <changeArmCtrlModeKuavo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'changeArmCtrlModeKuavo-request
    (cl:cons ':control_mode (control_mode msg))
))
;//! \htmlinclude changeArmCtrlModeKuavo-response.msg.html

(cl:defclass <changeArmCtrlModeKuavo-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass changeArmCtrlModeKuavo-response (<changeArmCtrlModeKuavo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <changeArmCtrlModeKuavo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'changeArmCtrlModeKuavo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_capture_ik-srv:<changeArmCtrlModeKuavo-response> is deprecated: use motion_capture_ik-srv:changeArmCtrlModeKuavo-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <changeArmCtrlModeKuavo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_capture_ik-srv:result-val is deprecated.  Use motion_capture_ik-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <changeArmCtrlModeKuavo-response>) ostream)
  "Serializes a message object of type '<changeArmCtrlModeKuavo-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <changeArmCtrlModeKuavo-response>) istream)
  "Deserializes a message object of type '<changeArmCtrlModeKuavo-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<changeArmCtrlModeKuavo-response>)))
  "Returns string type for a service object of type '<changeArmCtrlModeKuavo-response>"
  "motion_capture_ik/changeArmCtrlModeKuavoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'changeArmCtrlModeKuavo-response)))
  "Returns string type for a service object of type 'changeArmCtrlModeKuavo-response"
  "motion_capture_ik/changeArmCtrlModeKuavoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<changeArmCtrlModeKuavo-response>)))
  "Returns md5sum for a message object of type '<changeArmCtrlModeKuavo-response>"
  "f89438f9d6f48f748eabe64775a22261")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'changeArmCtrlModeKuavo-response)))
  "Returns md5sum for a message object of type 'changeArmCtrlModeKuavo-response"
  "f89438f9d6f48f748eabe64775a22261")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<changeArmCtrlModeKuavo-response>)))
  "Returns full string definition for message of type '<changeArmCtrlModeKuavo-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'changeArmCtrlModeKuavo-response)))
  "Returns full string definition for message of type 'changeArmCtrlModeKuavo-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <changeArmCtrlModeKuavo-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <changeArmCtrlModeKuavo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'changeArmCtrlModeKuavo-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'changeArmCtrlModeKuavo)))
  'changeArmCtrlModeKuavo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'changeArmCtrlModeKuavo)))
  'changeArmCtrlModeKuavo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'changeArmCtrlModeKuavo)))
  "Returns string type for a service object of type '<changeArmCtrlModeKuavo>"
  "motion_capture_ik/changeArmCtrlModeKuavo")