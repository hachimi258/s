;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::setTagId)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'setTagId (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::SETTAGID")
  (make-package "KUAVO_MSGS::SETTAGID"))
(unless (find-package "KUAVO_MSGS::SETTAGIDREQUEST")
  (make-package "KUAVO_MSGS::SETTAGIDREQUEST"))
(unless (find-package "KUAVO_MSGS::SETTAGIDRESPONSE")
  (make-package "KUAVO_MSGS::SETTAGIDRESPONSE"))

(in-package "ROS")





(defclass kuavo_msgs::setTagIdRequest
  :super ros::object
  :slots (_tag_id ))

(defmethod kuavo_msgs::setTagIdRequest
  (:init
   (&key
    ((:tag_id __tag_id) 0)
    )
   (send-super :init)
   (setq _tag_id (round __tag_id))
   self)
  (:tag_id
   (&optional __tag_id)
   (if __tag_id (setq _tag_id __tag_id)) _tag_id)
  (:serialization-length
   ()
   (+
    ;; int32 _tag_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _tag_id
       (write-long _tag_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _tag_id
     (setq _tag_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass kuavo_msgs::setTagIdResponse
  :super ros::object
  :slots (_success _tag_id _message ))

(defmethod kuavo_msgs::setTagIdResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:tag_id __tag_id) 0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _tag_id (round __tag_id))
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:tag_id
   (&optional __tag_id)
   (if __tag_id (setq _tag_id __tag_id)) _tag_id)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _tag_id
    4
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; int32 _tag_id
       (write-long _tag_id s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _tag_id
     (setq _tag_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass kuavo_msgs::setTagId
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::setTagId :md5sum-) "f6b2c1ef4eedb7525fd3a42b8e5cd61f")
(setf (get kuavo_msgs::setTagId :datatype-) "kuavo_msgs/setTagId")
(setf (get kuavo_msgs::setTagId :request) kuavo_msgs::setTagIdRequest)
(setf (get kuavo_msgs::setTagId :response) kuavo_msgs::setTagIdResponse)

(defmethod kuavo_msgs::setTagIdRequest
  (:response () (instance kuavo_msgs::setTagIdResponse :init)))

(setf (get kuavo_msgs::setTagIdRequest :md5sum-) "f6b2c1ef4eedb7525fd3a42b8e5cd61f")
(setf (get kuavo_msgs::setTagIdRequest :datatype-) "kuavo_msgs/setTagIdRequest")
(setf (get kuavo_msgs::setTagIdRequest :definition-)
      "int32 tag_id 
---
bool success
int32 tag_id 
string message


")

(setf (get kuavo_msgs::setTagIdResponse :md5sum-) "f6b2c1ef4eedb7525fd3a42b8e5cd61f")
(setf (get kuavo_msgs::setTagIdResponse :datatype-) "kuavo_msgs/setTagIdResponse")
(setf (get kuavo_msgs::setTagIdResponse :definition-)
      "int32 tag_id 
---
bool success
int32 tag_id 
string message


")



(provide :kuavo_msgs/setTagId "f6b2c1ef4eedb7525fd3a42b8e5cd61f")


