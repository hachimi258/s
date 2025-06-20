;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::setMotorEncoderRoundService)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'setMotorEncoderRoundService (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::SETMOTORENCODERROUNDSERVICE")
  (make-package "KUAVO_MSGS::SETMOTORENCODERROUNDSERVICE"))
(unless (find-package "KUAVO_MSGS::SETMOTORENCODERROUNDSERVICEREQUEST")
  (make-package "KUAVO_MSGS::SETMOTORENCODERROUNDSERVICEREQUEST"))
(unless (find-package "KUAVO_MSGS::SETMOTORENCODERROUNDSERVICERESPONSE")
  (make-package "KUAVO_MSGS::SETMOTORENCODERROUNDSERVICERESPONSE"))

(in-package "ROS")





(defclass kuavo_msgs::setMotorEncoderRoundServiceRequest
  :super ros::object
  :slots (_motor_id _direction _save_offset ))

(defmethod kuavo_msgs::setMotorEncoderRoundServiceRequest
  (:init
   (&key
    ((:motor_id __motor_id) 0)
    ((:direction __direction) 0)
    ((:save_offset __save_offset) nil)
    )
   (send-super :init)
   (setq _motor_id (round __motor_id))
   (setq _direction (round __direction))
   (setq _save_offset __save_offset)
   self)
  (:motor_id
   (&optional __motor_id)
   (if __motor_id (setq _motor_id __motor_id)) _motor_id)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:save_offset
   (&optional (__save_offset :null))
   (if (not (eq __save_offset :null)) (setq _save_offset __save_offset)) _save_offset)
  (:serialization-length
   ()
   (+
    ;; int32 _motor_id
    4
    ;; int32 _direction
    4
    ;; bool _save_offset
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _motor_id
       (write-long _motor_id s)
     ;; int32 _direction
       (write-long _direction s)
     ;; bool _save_offset
       (if _save_offset (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _motor_id
     (setq _motor_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _direction
     (setq _direction (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _save_offset
     (setq _save_offset (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass kuavo_msgs::setMotorEncoderRoundServiceResponse
  :super ros::object
  :slots (_success _message ))

(defmethod kuavo_msgs::setMotorEncoderRoundServiceResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass kuavo_msgs::setMotorEncoderRoundService
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::setMotorEncoderRoundService :md5sum-) "c8e2df10122ff9a180b403f54a3f8831")
(setf (get kuavo_msgs::setMotorEncoderRoundService :datatype-) "kuavo_msgs/setMotorEncoderRoundService")
(setf (get kuavo_msgs::setMotorEncoderRoundService :request) kuavo_msgs::setMotorEncoderRoundServiceRequest)
(setf (get kuavo_msgs::setMotorEncoderRoundService :response) kuavo_msgs::setMotorEncoderRoundServiceResponse)

(defmethod kuavo_msgs::setMotorEncoderRoundServiceRequest
  (:response () (instance kuavo_msgs::setMotorEncoderRoundServiceResponse :init)))

(setf (get kuavo_msgs::setMotorEncoderRoundServiceRequest :md5sum-) "c8e2df10122ff9a180b403f54a3f8831")
(setf (get kuavo_msgs::setMotorEncoderRoundServiceRequest :datatype-) "kuavo_msgs/setMotorEncoderRoundServiceRequest")
(setf (get kuavo_msgs::setMotorEncoderRoundServiceRequest :definition-)
      "# 设置电机编码器圈数，用于单编码器版本的校准
# 传入参数
int32 motor_id     # 电机 ID
int32 direction    # 方向，1 表示顺时针+1圈，-1 表示逆时针-1圈
bool save_offset   # 是否保存偏移量到文件
---
# 返回结果
bool success       # 成功与否
string message     # 返回信息

")

(setf (get kuavo_msgs::setMotorEncoderRoundServiceResponse :md5sum-) "c8e2df10122ff9a180b403f54a3f8831")
(setf (get kuavo_msgs::setMotorEncoderRoundServiceResponse :datatype-) "kuavo_msgs/setMotorEncoderRoundServiceResponse")
(setf (get kuavo_msgs::setMotorEncoderRoundServiceResponse :definition-)
      "# 设置电机编码器圈数，用于单编码器版本的校准
# 传入参数
int32 motor_id     # 电机 ID
int32 direction    # 方向，1 表示顺时针+1圈，-1 表示逆时针-1圈
bool save_offset   # 是否保存偏移量到文件
---
# 返回结果
bool success       # 成功与否
string message     # 返回信息

")



(provide :kuavo_msgs/setMotorEncoderRoundService "c8e2df10122ff9a180b403f54a3f8831")


