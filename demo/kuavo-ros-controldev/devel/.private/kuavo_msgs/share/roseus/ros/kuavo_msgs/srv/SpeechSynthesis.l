;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::SpeechSynthesis)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'SpeechSynthesis (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::SPEECHSYNTHESIS")
  (make-package "KUAVO_MSGS::SPEECHSYNTHESIS"))
(unless (find-package "KUAVO_MSGS::SPEECHSYNTHESISREQUEST")
  (make-package "KUAVO_MSGS::SPEECHSYNTHESISREQUEST"))
(unless (find-package "KUAVO_MSGS::SPEECHSYNTHESISRESPONSE")
  (make-package "KUAVO_MSGS::SPEECHSYNTHESISRESPONSE"))

(in-package "ROS")





(defclass kuavo_msgs::SpeechSynthesisRequest
  :super ros::object
  :slots (_data _volume ))

(defmethod kuavo_msgs::SpeechSynthesisRequest
  (:init
   (&key
    ((:data __data) "")
    ((:volume __volume) 0.0)
    )
   (send-super :init)
   (setq _data (string __data))
   (setq _volume (float __volume))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:volume
   (&optional __volume)
   (if __volume (setq _volume __volume)) _volume)
  (:serialization-length
   ()
   (+
    ;; string _data
    4 (length _data)
    ;; float32 _volume
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;; float32 _volume
       (sys::poke _volume (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _volume
     (setq _volume (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass kuavo_msgs::SpeechSynthesisResponse
  :super ros::object
  :slots (_success ))

(defmethod kuavo_msgs::SpeechSynthesisResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass kuavo_msgs::SpeechSynthesis
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::SpeechSynthesis :md5sum-) "c2d03ac0ff2ddd1cb86a5148ad0f9d5e")
(setf (get kuavo_msgs::SpeechSynthesis :datatype-) "kuavo_msgs/SpeechSynthesis")
(setf (get kuavo_msgs::SpeechSynthesis :request) kuavo_msgs::SpeechSynthesisRequest)
(setf (get kuavo_msgs::SpeechSynthesis :response) kuavo_msgs::SpeechSynthesisResponse)

(defmethod kuavo_msgs::SpeechSynthesisRequest
  (:response () (instance kuavo_msgs::SpeechSynthesisResponse :init)))

(setf (get kuavo_msgs::SpeechSynthesisRequest :md5sum-) "c2d03ac0ff2ddd1cb86a5148ad0f9d5e")
(setf (get kuavo_msgs::SpeechSynthesisRequest :datatype-) "kuavo_msgs/SpeechSynthesisRequest")
(setf (get kuavo_msgs::SpeechSynthesisRequest :definition-)
      "# 请求部分
string data
float32 volume
---
# 响应部分
bool success   

")

(setf (get kuavo_msgs::SpeechSynthesisResponse :md5sum-) "c2d03ac0ff2ddd1cb86a5148ad0f9d5e")
(setf (get kuavo_msgs::SpeechSynthesisResponse :datatype-) "kuavo_msgs/SpeechSynthesisResponse")
(setf (get kuavo_msgs::SpeechSynthesisResponse :definition-)
      "# 请求部分
string data
float32 volume
---
# 响应部分
bool success   

")



(provide :kuavo_msgs/SpeechSynthesis "c2d03ac0ff2ddd1cb86a5148ad0f9d5e")


