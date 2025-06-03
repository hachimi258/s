; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-srv)


;//! \htmlinclude gestureList-request.msg.html

(cl:defclass <gestureList-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass gestureList-request (<gestureList-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gestureList-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gestureList-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<gestureList-request> is deprecated: use kuavo_msgs-srv:gestureList-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gestureList-request>) ostream)
  "Serializes a message object of type '<gestureList-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gestureList-request>) istream)
  "Deserializes a message object of type '<gestureList-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gestureList-request>)))
  "Returns string type for a service object of type '<gestureList-request>"
  "kuavo_msgs/gestureListRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gestureList-request)))
  "Returns string type for a service object of type 'gestureList-request"
  "kuavo_msgs/gestureListRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gestureList-request>)))
  "Returns md5sum for a message object of type '<gestureList-request>"
  "ee839a0568f441526fe05bf2e2f25572")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gestureList-request)))
  "Returns md5sum for a message object of type 'gestureList-request"
  "ee839a0568f441526fe05bf2e2f25572")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gestureList-request>)))
  "Returns full string definition for message of type '<gestureList-request>"
  (cl:format cl:nil "# This service returns a list of available gestures.~%# It is used to query the system for all gestures that can be recognized or performed.~%#~%# Request:~%# No input parameters are required.~%#~%# Response:~%# bool success                # Indicates whether the request was successful.~%# int32 gesture_count         # The number of gestures returned in the list.~%# string message              # A message indicating the result of the request.~%# kuavo_msgs/gestureInfo[] gesture_infos # A list of gesture information, each containing the name, alias, and description of a gesture.~%~%# Define the GestureInfo message~%# string gesture_name        # The name of the gesture.~%# string[] alias             # A list of aliases for the gesture.~%# string description         # A description of the gesture.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gestureList-request)))
  "Returns full string definition for message of type 'gestureList-request"
  (cl:format cl:nil "# This service returns a list of available gestures.~%# It is used to query the system for all gestures that can be recognized or performed.~%#~%# Request:~%# No input parameters are required.~%#~%# Response:~%# bool success                # Indicates whether the request was successful.~%# int32 gesture_count         # The number of gestures returned in the list.~%# string message              # A message indicating the result of the request.~%# kuavo_msgs/gestureInfo[] gesture_infos # A list of gesture information, each containing the name, alias, and description of a gesture.~%~%# Define the GestureInfo message~%# string gesture_name        # The name of the gesture.~%# string[] alias             # A list of aliases for the gesture.~%# string description         # A description of the gesture.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gestureList-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gestureList-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gestureList-request
))
;//! \htmlinclude gestureList-response.msg.html

(cl:defclass <gestureList-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (gesture_count
    :reader gesture_count
    :initarg :gesture_count
    :type cl:integer
    :initform 0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (gesture_infos
    :reader gesture_infos
    :initarg :gesture_infos
    :type (cl:vector kuavo_msgs-msg:gestureInfo)
   :initform (cl:make-array 0 :element-type 'kuavo_msgs-msg:gestureInfo :initial-element (cl:make-instance 'kuavo_msgs-msg:gestureInfo))))
)

(cl:defclass gestureList-response (<gestureList-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gestureList-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gestureList-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<gestureList-response> is deprecated: use kuavo_msgs-srv:gestureList-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <gestureList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:success-val is deprecated.  Use kuavo_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'gesture_count-val :lambda-list '(m))
(cl:defmethod gesture_count-val ((m <gestureList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:gesture_count-val is deprecated.  Use kuavo_msgs-srv:gesture_count instead.")
  (gesture_count m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <gestureList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:message-val is deprecated.  Use kuavo_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'gesture_infos-val :lambda-list '(m))
(cl:defmethod gesture_infos-val ((m <gestureList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:gesture_infos-val is deprecated.  Use kuavo_msgs-srv:gesture_infos instead.")
  (gesture_infos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gestureList-response>) ostream)
  "Serializes a message object of type '<gestureList-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'gesture_count)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gesture_infos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'gesture_infos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gestureList-response>) istream)
  "Deserializes a message object of type '<gestureList-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gesture_count) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gesture_infos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gesture_infos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kuavo_msgs-msg:gestureInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gestureList-response>)))
  "Returns string type for a service object of type '<gestureList-response>"
  "kuavo_msgs/gestureListResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gestureList-response)))
  "Returns string type for a service object of type 'gestureList-response"
  "kuavo_msgs/gestureListResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gestureList-response>)))
  "Returns md5sum for a message object of type '<gestureList-response>"
  "ee839a0568f441526fe05bf2e2f25572")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gestureList-response)))
  "Returns md5sum for a message object of type 'gestureList-response"
  "ee839a0568f441526fe05bf2e2f25572")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gestureList-response>)))
  "Returns full string definition for message of type '<gestureList-response>"
  (cl:format cl:nil "bool success~%int32 gesture_count~%string message~%kuavo_msgs/gestureInfo[] gesture_infos~%~%================================================================================~%MSG: kuavo_msgs/gestureInfo~%# This message defines the information for a single gesture.~%# It includes the name, a list of aliases, and a description of the gesture.~%~%# The name of the gesture.~%string gesture_name~%~%# A list of aliases for the gesture. These can be alternative names or shortcuts.~%string[] alias~%~%# A description of the gesture, providing more detailed information about its purpose and usage.~%string description~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gestureList-response)))
  "Returns full string definition for message of type 'gestureList-response"
  (cl:format cl:nil "bool success~%int32 gesture_count~%string message~%kuavo_msgs/gestureInfo[] gesture_infos~%~%================================================================================~%MSG: kuavo_msgs/gestureInfo~%# This message defines the information for a single gesture.~%# It includes the name, a list of aliases, and a description of the gesture.~%~%# The name of the gesture.~%string gesture_name~%~%# A list of aliases for the gesture. These can be alternative names or shortcuts.~%string[] alias~%~%# A description of the gesture, providing more detailed information about its purpose and usage.~%string description~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gestureList-response>))
  (cl:+ 0
     1
     4
     4 (cl:length (cl:slot-value msg 'message))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gesture_infos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gestureList-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gestureList-response
    (cl:cons ':success (success msg))
    (cl:cons ':gesture_count (gesture_count msg))
    (cl:cons ':message (message msg))
    (cl:cons ':gesture_infos (gesture_infos msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gestureList)))
  'gestureList-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gestureList)))
  'gestureList-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gestureList)))
  "Returns string type for a service object of type '<gestureList>"
  "kuavo_msgs/gestureList")