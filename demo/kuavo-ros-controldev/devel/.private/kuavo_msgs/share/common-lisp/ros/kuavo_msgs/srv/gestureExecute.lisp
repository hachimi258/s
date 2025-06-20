; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-srv)


;//! \htmlinclude gestureExecute-request.msg.html

(cl:defclass <gestureExecute-request> (roslisp-msg-protocol:ros-message)
  ((gestures
    :reader gestures
    :initarg :gestures
    :type (cl:vector kuavo_msgs-msg:gestureTask)
   :initform (cl:make-array 0 :element-type 'kuavo_msgs-msg:gestureTask :initial-element (cl:make-instance 'kuavo_msgs-msg:gestureTask))))
)

(cl:defclass gestureExecute-request (<gestureExecute-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gestureExecute-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gestureExecute-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<gestureExecute-request> is deprecated: use kuavo_msgs-srv:gestureExecute-request instead.")))

(cl:ensure-generic-function 'gestures-val :lambda-list '(m))
(cl:defmethod gestures-val ((m <gestureExecute-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:gestures-val is deprecated.  Use kuavo_msgs-srv:gestures instead.")
  (gestures m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gestureExecute-request>) ostream)
  "Serializes a message object of type '<gestureExecute-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gestures))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'gestures))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gestureExecute-request>) istream)
  "Deserializes a message object of type '<gestureExecute-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gestures) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gestures)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kuavo_msgs-msg:gestureTask))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gestureExecute-request>)))
  "Returns string type for a service object of type '<gestureExecute-request>"
  "kuavo_msgs/gestureExecuteRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gestureExecute-request)))
  "Returns string type for a service object of type 'gestureExecute-request"
  "kuavo_msgs/gestureExecuteRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gestureExecute-request>)))
  "Returns md5sum for a message object of type '<gestureExecute-request>"
  "b599da36839d439975fbac8d4bfbeb7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gestureExecute-request)))
  "Returns md5sum for a message object of type 'gestureExecute-request"
  "b599da36839d439975fbac8d4bfbeb7e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gestureExecute-request>)))
  "Returns full string definition for message of type '<gestureExecute-request>"
  (cl:format cl:nil "# This service executes a specified gesture.~%# It is used to trigger a gesture by providing its name and the side of the hand(s) to use.~%#~%# Request:~%# kuavo_msgs/gestureTask[] gestures # An array of gestures to execute, each with a name and hand side~%#~%# Response:~%# bool success         # Indicates whether the gesture execution was successful.~%# string message       # A message providing additional information (e.g., error details if the gesture failed).~%~%kuavo_msgs/gestureTask[] gestures~%~%================================================================================~%MSG: kuavo_msgs/gestureTask~%# This message is used to specify a gesture to execute.~%# The gesture is triggered by providing its name and the side of the hand(s) to use.~%~%string gesture_name  # Name of the gesture to execute~%int8   hand_side    # Side of the hand to use (e.g., 0 for left, 1 for right, 2 for both)~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gestureExecute-request)))
  "Returns full string definition for message of type 'gestureExecute-request"
  (cl:format cl:nil "# This service executes a specified gesture.~%# It is used to trigger a gesture by providing its name and the side of the hand(s) to use.~%#~%# Request:~%# kuavo_msgs/gestureTask[] gestures # An array of gestures to execute, each with a name and hand side~%#~%# Response:~%# bool success         # Indicates whether the gesture execution was successful.~%# string message       # A message providing additional information (e.g., error details if the gesture failed).~%~%kuavo_msgs/gestureTask[] gestures~%~%================================================================================~%MSG: kuavo_msgs/gestureTask~%# This message is used to specify a gesture to execute.~%# The gesture is triggered by providing its name and the side of the hand(s) to use.~%~%string gesture_name  # Name of the gesture to execute~%int8   hand_side    # Side of the hand to use (e.g., 0 for left, 1 for right, 2 for both)~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gestureExecute-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gestures) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gestureExecute-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gestureExecute-request
    (cl:cons ':gestures (gestures msg))
))
;//! \htmlinclude gestureExecute-response.msg.html

(cl:defclass <gestureExecute-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass gestureExecute-response (<gestureExecute-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gestureExecute-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gestureExecute-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<gestureExecute-response> is deprecated: use kuavo_msgs-srv:gestureExecute-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <gestureExecute-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:success-val is deprecated.  Use kuavo_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <gestureExecute-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:message-val is deprecated.  Use kuavo_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gestureExecute-response>) ostream)
  "Serializes a message object of type '<gestureExecute-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gestureExecute-response>) istream)
  "Deserializes a message object of type '<gestureExecute-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gestureExecute-response>)))
  "Returns string type for a service object of type '<gestureExecute-response>"
  "kuavo_msgs/gestureExecuteResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gestureExecute-response)))
  "Returns string type for a service object of type 'gestureExecute-response"
  "kuavo_msgs/gestureExecuteResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gestureExecute-response>)))
  "Returns md5sum for a message object of type '<gestureExecute-response>"
  "b599da36839d439975fbac8d4bfbeb7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gestureExecute-response)))
  "Returns md5sum for a message object of type 'gestureExecute-response"
  "b599da36839d439975fbac8d4bfbeb7e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gestureExecute-response>)))
  "Returns full string definition for message of type '<gestureExecute-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gestureExecute-response)))
  "Returns full string definition for message of type 'gestureExecute-response"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gestureExecute-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gestureExecute-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gestureExecute-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gestureExecute)))
  'gestureExecute-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gestureExecute)))
  'gestureExecute-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gestureExecute)))
  "Returns string type for a service object of type '<gestureExecute>"
  "kuavo_msgs/gestureExecute")