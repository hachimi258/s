;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::SetTagPose)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'SetTagPose (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::SETTAGPOSE")
  (make-package "KUAVO_MSGS::SETTAGPOSE"))
(unless (find-package "KUAVO_MSGS::SETTAGPOSEREQUEST")
  (make-package "KUAVO_MSGS::SETTAGPOSEREQUEST"))
(unless (find-package "KUAVO_MSGS::SETTAGPOSERESPONSE")
  (make-package "KUAVO_MSGS::SETTAGPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass kuavo_msgs::SetTagPoseRequest
  :super ros::object
  :slots (_tag_id _pose ))

(defmethod kuavo_msgs::SetTagPoseRequest
  (:init
   (&key
    ((:tag_id __tag_id) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _tag_id (round __tag_id))
   (setq _pose __pose)
   self)
  (:tag_id
   (&optional __tag_id)
   (if __tag_id (setq _tag_id __tag_id)) _tag_id)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; int32 _tag_id
    4
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _tag_id
       (write-long _tag_id s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _tag_id
     (setq _tag_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(defclass kuavo_msgs::SetTagPoseResponse
  :super ros::object
  :slots (_success _message ))

(defmethod kuavo_msgs::SetTagPoseResponse
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

(defclass kuavo_msgs::SetTagPose
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::SetTagPose :md5sum-) "04a66d62173ddb0945f89d9279dd4ddb")
(setf (get kuavo_msgs::SetTagPose :datatype-) "kuavo_msgs/SetTagPose")
(setf (get kuavo_msgs::SetTagPose :request) kuavo_msgs::SetTagPoseRequest)
(setf (get kuavo_msgs::SetTagPose :response) kuavo_msgs::SetTagPoseResponse)

(defmethod kuavo_msgs::SetTagPoseRequest
  (:response () (instance kuavo_msgs::SetTagPoseResponse :init)))

(setf (get kuavo_msgs::SetTagPoseRequest :md5sum-) "04a66d62173ddb0945f89d9279dd4ddb")
(setf (get kuavo_msgs::SetTagPoseRequest :datatype-) "kuavo_msgs/SetTagPoseRequest")
(setf (get kuavo_msgs::SetTagPoseRequest :definition-)
      "# Request
int32 tag_id
geometry_msgs/Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
# Response
bool success
string message 

")

(setf (get kuavo_msgs::SetTagPoseResponse :md5sum-) "04a66d62173ddb0945f89d9279dd4ddb")
(setf (get kuavo_msgs::SetTagPoseResponse :datatype-) "kuavo_msgs/SetTagPoseResponse")
(setf (get kuavo_msgs::SetTagPoseResponse :definition-)
      "# Request
int32 tag_id
geometry_msgs/Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
# Response
bool success
string message 

")



(provide :kuavo_msgs/SetTagPose "04a66d62173ddb0945f89d9279dd4ddb")


