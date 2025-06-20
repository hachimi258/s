; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-msg)


;//! \htmlinclude gestureInfo.msg.html

(cl:defclass <gestureInfo> (roslisp-msg-protocol:ros-message)
  ((gesture_name
    :reader gesture_name
    :initarg :gesture_name
    :type cl:string
    :initform "")
   (alias
    :reader alias
    :initarg :alias
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (description
    :reader description
    :initarg :description
    :type cl:string
    :initform ""))
)

(cl:defclass gestureInfo (<gestureInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gestureInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gestureInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-msg:<gestureInfo> is deprecated: use kuavo_msgs-msg:gestureInfo instead.")))

(cl:ensure-generic-function 'gesture_name-val :lambda-list '(m))
(cl:defmethod gesture_name-val ((m <gestureInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:gesture_name-val is deprecated.  Use kuavo_msgs-msg:gesture_name instead.")
  (gesture_name m))

(cl:ensure-generic-function 'alias-val :lambda-list '(m))
(cl:defmethod alias-val ((m <gestureInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:alias-val is deprecated.  Use kuavo_msgs-msg:alias instead.")
  (alias m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <gestureInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:description-val is deprecated.  Use kuavo_msgs-msg:description instead.")
  (description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gestureInfo>) ostream)
  "Serializes a message object of type '<gestureInfo>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'gesture_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'gesture_name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'alias))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'alias))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gestureInfo>) istream)
  "Deserializes a message object of type '<gestureInfo>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gesture_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'gesture_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'alias) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'alias)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gestureInfo>)))
  "Returns string type for a message object of type '<gestureInfo>"
  "kuavo_msgs/gestureInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gestureInfo)))
  "Returns string type for a message object of type 'gestureInfo"
  "kuavo_msgs/gestureInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gestureInfo>)))
  "Returns md5sum for a message object of type '<gestureInfo>"
  "65efb896db2f0292354e0a9098b39b97")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gestureInfo)))
  "Returns md5sum for a message object of type 'gestureInfo"
  "65efb896db2f0292354e0a9098b39b97")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gestureInfo>)))
  "Returns full string definition for message of type '<gestureInfo>"
  (cl:format cl:nil "# This message defines the information for a single gesture.~%# It includes the name, a list of aliases, and a description of the gesture.~%~%# The name of the gesture.~%string gesture_name~%~%# A list of aliases for the gesture. These can be alternative names or shortcuts.~%string[] alias~%~%# A description of the gesture, providing more detailed information about its purpose and usage.~%string description~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gestureInfo)))
  "Returns full string definition for message of type 'gestureInfo"
  (cl:format cl:nil "# This message defines the information for a single gesture.~%# It includes the name, a list of aliases, and a description of the gesture.~%~%# The name of the gesture.~%string gesture_name~%~%# A list of aliases for the gesture. These can be alternative names or shortcuts.~%string[] alias~%~%# A description of the gesture, providing more detailed information about its purpose and usage.~%string description~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gestureInfo>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'gesture_name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'alias) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:length (cl:slot-value msg 'description))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gestureInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'gestureInfo
    (cl:cons ':gesture_name (gesture_name msg))
    (cl:cons ':alias (alias msg))
    (cl:cons ':description (description msg))
))
