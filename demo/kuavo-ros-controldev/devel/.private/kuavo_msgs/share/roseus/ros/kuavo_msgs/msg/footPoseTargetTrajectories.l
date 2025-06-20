;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::footPoseTargetTrajectories)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'footPoseTargetTrajectories (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::FOOTPOSETARGETTRAJECTORIES")
  (make-package "KUAVO_MSGS::FOOTPOSETARGETTRAJECTORIES"))

(in-package "ROS")
;;//! \htmlinclude footPoseTargetTrajectories.msg.html


(defclass kuavo_msgs::footPoseTargetTrajectories
  :super ros::object
  :slots (_timeTrajectory _footIndexTrajectory _footPoseTrajectory _additionalFootPoseTrajectory ))

(defmethod kuavo_msgs::footPoseTargetTrajectories
  (:init
   (&key
    ((:timeTrajectory __timeTrajectory) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:footIndexTrajectory __footIndexTrajectory) (make-array 0 :initial-element 0 :element-type :integer))
    ((:footPoseTrajectory __footPoseTrajectory) ())
    ((:additionalFootPoseTrajectory __additionalFootPoseTrajectory) ())
    )
   (send-super :init)
   (setq _timeTrajectory __timeTrajectory)
   (setq _footIndexTrajectory __footIndexTrajectory)
   (setq _footPoseTrajectory __footPoseTrajectory)
   (setq _additionalFootPoseTrajectory __additionalFootPoseTrajectory)
   self)
  (:timeTrajectory
   (&optional __timeTrajectory)
   (if __timeTrajectory (setq _timeTrajectory __timeTrajectory)) _timeTrajectory)
  (:footIndexTrajectory
   (&optional __footIndexTrajectory)
   (if __footIndexTrajectory (setq _footIndexTrajectory __footIndexTrajectory)) _footIndexTrajectory)
  (:footPoseTrajectory
   (&rest __footPoseTrajectory)
   (if (keywordp (car __footPoseTrajectory))
       (send* _footPoseTrajectory __footPoseTrajectory)
     (progn
       (if __footPoseTrajectory (setq _footPoseTrajectory (car __footPoseTrajectory)))
       _footPoseTrajectory)))
  (:additionalFootPoseTrajectory
   (&rest __additionalFootPoseTrajectory)
   (if (keywordp (car __additionalFootPoseTrajectory))
       (send* _additionalFootPoseTrajectory __additionalFootPoseTrajectory)
     (progn
       (if __additionalFootPoseTrajectory (setq _additionalFootPoseTrajectory (car __additionalFootPoseTrajectory)))
       _additionalFootPoseTrajectory)))
  (:serialization-length
   ()
   (+
    ;; float64[] _timeTrajectory
    (* 8    (length _timeTrajectory)) 4
    ;; int32[] _footIndexTrajectory
    (* 4    (length _footIndexTrajectory)) 4
    ;; kuavo_msgs/footPose[] _footPoseTrajectory
    (apply #'+ (send-all _footPoseTrajectory :serialization-length)) 4
    ;; kuavo_msgs/footPoses[] _additionalFootPoseTrajectory
    (apply #'+ (send-all _additionalFootPoseTrajectory :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _timeTrajectory
     (write-long (length _timeTrajectory) s)
     (dotimes (i (length _timeTrajectory))
       (sys::poke (elt _timeTrajectory i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32[] _footIndexTrajectory
     (write-long (length _footIndexTrajectory) s)
     (dotimes (i (length _footIndexTrajectory))
       (write-long (elt _footIndexTrajectory i) s)
       )
     ;; kuavo_msgs/footPose[] _footPoseTrajectory
     (write-long (length _footPoseTrajectory) s)
     (dolist (elem _footPoseTrajectory)
       (send elem :serialize s)
       )
     ;; kuavo_msgs/footPoses[] _additionalFootPoseTrajectory
     (write-long (length _additionalFootPoseTrajectory) s)
     (dolist (elem _additionalFootPoseTrajectory)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _timeTrajectory
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _timeTrajectory (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _timeTrajectory i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int32[] _footIndexTrajectory
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _footIndexTrajectory (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _footIndexTrajectory i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; kuavo_msgs/footPose[] _footPoseTrajectory
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _footPoseTrajectory (let (r) (dotimes (i n) (push (instance kuavo_msgs::footPose :init) r)) r))
     (dolist (elem- _footPoseTrajectory)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kuavo_msgs/footPoses[] _additionalFootPoseTrajectory
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _additionalFootPoseTrajectory (let (r) (dotimes (i n) (push (instance kuavo_msgs::footPoses :init) r)) r))
     (dolist (elem- _additionalFootPoseTrajectory)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kuavo_msgs::footPoseTargetTrajectories :md5sum-) "69f7e48d9a18b5c4756f9577aeefff25")
(setf (get kuavo_msgs::footPoseTargetTrajectories :datatype-) "kuavo_msgs/footPoseTargetTrajectories")
(setf (get kuavo_msgs::footPoseTargetTrajectories :definition-)
      "float64[]    timeTrajectory
int32[]      footIndexTrajectory
footPose[]   footPoseTrajectory
footPoses[]  additionalFootPoseTrajectory  # 可选字段，用于存储额外的轨迹点规划值

================================================================================
MSG: kuavo_msgs/footPose
float64[4] footPose # x, y, z, yaw
float64[4] torsoPose # x, y, z, yaw

================================================================================
MSG: kuavo_msgs/footPoses
footPose[] data

")



(provide :kuavo_msgs/footPoseTargetTrajectories "69f7e48d9a18b5c4756f9577aeefff25")


