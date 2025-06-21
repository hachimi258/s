; Auto-generated. Do not edit!


(cl:in-package kuavo_audio_player-srv)


;//! \htmlinclude playmusic-request.msg.html

(cl:defclass <playmusic-request> (roslisp-msg-protocol:ros-message)
  ((music_number
    :reader music_number
    :initarg :music_number
    :type cl:string
    :initform "")
   (volume
    :reader volume
    :initarg :volume
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass playmusic-request (<playmusic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <playmusic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'playmusic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_audio_player-srv:<playmusic-request> is deprecated: use kuavo_audio_player-srv:playmusic-request instead.")))

(cl:ensure-generic-function 'music_number-val :lambda-list '(m))
(cl:defmethod music_number-val ((m <playmusic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_audio_player-srv:music_number-val is deprecated.  Use kuavo_audio_player-srv:music_number instead.")
  (music_number m))

(cl:ensure-generic-function 'volume-val :lambda-list '(m))
(cl:defmethod volume-val ((m <playmusic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_audio_player-srv:volume-val is deprecated.  Use kuavo_audio_player-srv:volume instead.")
  (volume m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <playmusic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_audio_player-srv:speed-val is deprecated.  Use kuavo_audio_player-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <playmusic-request>) ostream)
  "Serializes a message object of type '<playmusic-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'music_number))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'music_number))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'volume))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <playmusic-request>) istream)
  "Deserializes a message object of type '<playmusic-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'music_number) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'music_number) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'volume) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<playmusic-request>)))
  "Returns string type for a service object of type '<playmusic-request>"
  "kuavo_audio_player/playmusicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'playmusic-request)))
  "Returns string type for a service object of type 'playmusic-request"
  "kuavo_audio_player/playmusicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<playmusic-request>)))
  "Returns md5sum for a message object of type '<playmusic-request>"
  "348f2445d1e569f5dc4e72a19fb11934")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'playmusic-request)))
  "Returns md5sum for a message object of type 'playmusic-request"
  "348f2445d1e569f5dc4e72a19fb11934")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<playmusic-request>)))
  "Returns full string definition for message of type '<playmusic-request>"
  (cl:format cl:nil "string music_number~%float32 volume~%float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'playmusic-request)))
  "Returns full string definition for message of type 'playmusic-request"
  (cl:format cl:nil "string music_number~%float32 volume~%float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <playmusic-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'music_number))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <playmusic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'playmusic-request
    (cl:cons ':music_number (music_number msg))
    (cl:cons ':volume (volume msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude playmusic-response.msg.html

(cl:defclass <playmusic-response> (roslisp-msg-protocol:ros-message)
  ((success_flag
    :reader success_flag
    :initarg :success_flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass playmusic-response (<playmusic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <playmusic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'playmusic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_audio_player-srv:<playmusic-response> is deprecated: use kuavo_audio_player-srv:playmusic-response instead.")))

(cl:ensure-generic-function 'success_flag-val :lambda-list '(m))
(cl:defmethod success_flag-val ((m <playmusic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_audio_player-srv:success_flag-val is deprecated.  Use kuavo_audio_player-srv:success_flag instead.")
  (success_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <playmusic-response>) ostream)
  "Serializes a message object of type '<playmusic-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success_flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <playmusic-response>) istream)
  "Deserializes a message object of type '<playmusic-response>"
    (cl:setf (cl:slot-value msg 'success_flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<playmusic-response>)))
  "Returns string type for a service object of type '<playmusic-response>"
  "kuavo_audio_player/playmusicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'playmusic-response)))
  "Returns string type for a service object of type 'playmusic-response"
  "kuavo_audio_player/playmusicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<playmusic-response>)))
  "Returns md5sum for a message object of type '<playmusic-response>"
  "348f2445d1e569f5dc4e72a19fb11934")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'playmusic-response)))
  "Returns md5sum for a message object of type 'playmusic-response"
  "348f2445d1e569f5dc4e72a19fb11934")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<playmusic-response>)))
  "Returns full string definition for message of type '<playmusic-response>"
  (cl:format cl:nil "bool success_flag~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'playmusic-response)))
  "Returns full string definition for message of type 'playmusic-response"
  (cl:format cl:nil "bool success_flag~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <playmusic-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <playmusic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'playmusic-response
    (cl:cons ':success_flag (success_flag msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'playmusic)))
  'playmusic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'playmusic)))
  'playmusic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'playmusic)))
  "Returns string type for a service object of type '<playmusic>"
  "kuavo_audio_player/playmusic")