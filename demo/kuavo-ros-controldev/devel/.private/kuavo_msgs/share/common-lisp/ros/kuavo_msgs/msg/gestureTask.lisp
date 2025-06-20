; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-msg)


;//! \htmlinclude gestureTask.msg.html

(cl:defclass <gestureTask> (roslisp-msg-protocol:ros-message)
  ((gesture_name
    :reader gesture_name
    :initarg :gesture_name
    :type cl:string
    :initform "")
   (hand_side
    :reader hand_side
    :initarg :hand_side
    :type cl:fixnum
    :initform 0))
)

(cl:defclass gestureTask (<gestureTask>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gestureTask>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gestureTask)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-msg:<gestureTask> is deprecated: use kuavo_msgs-msg:gestureTask instead.")))

(cl:ensure-generic-function 'gesture_name-val :lambda-list '(m))
(cl:defmethod gesture_name-val ((m <gestureTask>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:gesture_name-val is deprecated.  Use kuavo_msgs-msg:gesture_name instead.")
  (gesture_name m))

(cl:ensure-generic-function 'hand_side-val :lambda-list '(m))
(cl:defmethod hand_side-val ((m <gestureTask>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:hand_side-val is deprecated.  Use kuavo_msgs-msg:hand_side instead.")
  (hand_side m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gestureTask>) ostream)
  "Serializes a message object of type '<gestureTask>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'gesture_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'gesture_name))
  (cl:let* ((signed (cl:slot-value msg 'hand_side)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gestureTask>) istream)
  "Deserializes a message object of type '<gestureTask>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gesture_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'gesture_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hand_side) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gestureTask>)))
  "Returns string type for a message object of type '<gestureTask>"
  "kuavo_msgs/gestureTask")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gestureTask)))
  "Returns string type for a message object of type 'gestureTask"
  "kuavo_msgs/gestureTask")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gestureTask>)))
  "Returns md5sum for a message object of type '<gestureTask>"
  "be7fe1eba1df13c392c3a5d13b9f3dae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gestureTask)))
  "Returns md5sum for a message object of type 'gestureTask"
  "be7fe1eba1df13c392c3a5d13b9f3dae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gestureTask>)))
  "Returns full string definition for message of type '<gestureTask>"
  (cl:format cl:nil "# This message is used to specify a gesture to execute.~%# The gesture is triggered by providing its name and the side of the hand(s) to use.~%~%string gesture_name  # Name of the gesture to execute~%int8   hand_side    # Side of the hand to use (e.g., 0 for left, 1 for right, 2 for both)~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gestureTask)))
  "Returns full string definition for message of type 'gestureTask"
  (cl:format cl:nil "# This message is used to specify a gesture to execute.~%# The gesture is triggered by providing its name and the side of the hand(s) to use.~%~%string gesture_name  # Name of the gesture to execute~%int8   hand_side    # Side of the hand to use (e.g., 0 for left, 1 for right, 2 for both)~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gestureTask>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'gesture_name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gestureTask>))
  "Converts a ROS message object to a list"
  (cl:list 'gestureTask
    (cl:cons ':gesture_name (gesture_name msg))
    (cl:cons ':hand_side (hand_side msg))
))
