;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::ikSolveError)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'ikSolveError (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::IKSOLVEERROR")
  (make-package "KUAVO_MSGS::IKSOLVEERROR"))

(in-package "ROS")
;;//! \htmlinclude ikSolveError.msg.html


(defclass kuavo_msgs::ikSolveError
  :super ros::object
  :slots (_ik_type _left_pose_error _right_pose_error ))

(defmethod kuavo_msgs::ikSolveError
  (:init
   (&key
    ((:ik_type __ik_type) "")
    ((:left_pose_error __left_pose_error) (instance kuavo_msgs::handPose :init))
    ((:right_pose_error __right_pose_error) (instance kuavo_msgs::handPose :init))
    )
   (send-super :init)
   (setq _ik_type (string __ik_type))
   (setq _left_pose_error __left_pose_error)
   (setq _right_pose_error __right_pose_error)
   self)
  (:ik_type
   (&optional __ik_type)
   (if __ik_type (setq _ik_type __ik_type)) _ik_type)
  (:left_pose_error
   (&rest __left_pose_error)
   (if (keywordp (car __left_pose_error))
       (send* _left_pose_error __left_pose_error)
     (progn
       (if __left_pose_error (setq _left_pose_error (car __left_pose_error)))
       _left_pose_error)))
  (:right_pose_error
   (&rest __right_pose_error)
   (if (keywordp (car __right_pose_error))
       (send* _right_pose_error __right_pose_error)
     (progn
       (if __right_pose_error (setq _right_pose_error (car __right_pose_error)))
       _right_pose_error)))
  (:serialization-length
   ()
   (+
    ;; string _ik_type
    4 (length _ik_type)
    ;; kuavo_msgs/handPose _left_pose_error
    (send _left_pose_error :serialization-length)
    ;; kuavo_msgs/handPose _right_pose_error
    (send _right_pose_error :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _ik_type
       (write-long (length _ik_type) s) (princ _ik_type s)
     ;; kuavo_msgs/handPose _left_pose_error
       (send _left_pose_error :serialize s)
     ;; kuavo_msgs/handPose _right_pose_error
       (send _right_pose_error :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _ik_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ik_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; kuavo_msgs/handPose _left_pose_error
     (send _left_pose_error :deserialize buf ptr-) (incf ptr- (send _left_pose_error :serialization-length))
   ;; kuavo_msgs/handPose _right_pose_error
     (send _right_pose_error :deserialize buf ptr-) (incf ptr- (send _right_pose_error :serialization-length))
   ;;
   self)
  )

(setf (get kuavo_msgs::ikSolveError :md5sum-) "06c12c0e6e08f286627a6f856e26223c")
(setf (get kuavo_msgs::ikSolveError :datatype-) "kuavo_msgs/ikSolveError")
(setf (get kuavo_msgs::ikSolveError :definition-)
      "string     ik_type 
handPose  left_pose_error
handPose  right_pose_error
================================================================================
MSG: kuavo_msgs/handPose
# pos
float64 x
float64 y
float64 z
# rpy
float64 roll
float64 pitch
float64 yaw
")



(provide :kuavo_msgs/ikSolveError "06c12c0e6e08f286627a6f856e26223c")


