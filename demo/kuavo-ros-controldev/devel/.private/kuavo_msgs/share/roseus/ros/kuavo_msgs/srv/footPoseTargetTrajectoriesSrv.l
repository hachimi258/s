;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::footPoseTargetTrajectoriesSrv)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'footPoseTargetTrajectoriesSrv (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::FOOTPOSETARGETTRAJECTORIESSRV")
  (make-package "KUAVO_MSGS::FOOTPOSETARGETTRAJECTORIESSRV"))
(unless (find-package "KUAVO_MSGS::FOOTPOSETARGETTRAJECTORIESSRVREQUEST")
  (make-package "KUAVO_MSGS::FOOTPOSETARGETTRAJECTORIESSRVREQUEST"))
(unless (find-package "KUAVO_MSGS::FOOTPOSETARGETTRAJECTORIESSRVRESPONSE")
  (make-package "KUAVO_MSGS::FOOTPOSETARGETTRAJECTORIESSRVRESPONSE"))

(in-package "ROS")





(defclass kuavo_msgs::footPoseTargetTrajectoriesSrvRequest
  :super ros::object
  :slots (_foot_pose_target_trajectories ))

(defmethod kuavo_msgs::footPoseTargetTrajectoriesSrvRequest
  (:init
   (&key
    ((:foot_pose_target_trajectories __foot_pose_target_trajectories) (instance kuavo_msgs::footPoseTargetTrajectories :init))
    )
   (send-super :init)
   (setq _foot_pose_target_trajectories __foot_pose_target_trajectories)
   self)
  (:foot_pose_target_trajectories
   (&rest __foot_pose_target_trajectories)
   (if (keywordp (car __foot_pose_target_trajectories))
       (send* _foot_pose_target_trajectories __foot_pose_target_trajectories)
     (progn
       (if __foot_pose_target_trajectories (setq _foot_pose_target_trajectories (car __foot_pose_target_trajectories)))
       _foot_pose_target_trajectories)))
  (:serialization-length
   ()
   (+
    ;; kuavo_msgs/footPoseTargetTrajectories _foot_pose_target_trajectories
    (send _foot_pose_target_trajectories :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kuavo_msgs/footPoseTargetTrajectories _foot_pose_target_trajectories
       (send _foot_pose_target_trajectories :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kuavo_msgs/footPoseTargetTrajectories _foot_pose_target_trajectories
     (send _foot_pose_target_trajectories :deserialize buf ptr-) (incf ptr- (send _foot_pose_target_trajectories :serialization-length))
   ;;
   self)
  )

(defclass kuavo_msgs::footPoseTargetTrajectoriesSrvResponse
  :super ros::object
  :slots (_success ))

(defmethod kuavo_msgs::footPoseTargetTrajectoriesSrvResponse
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

(defclass kuavo_msgs::footPoseTargetTrajectoriesSrv
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::footPoseTargetTrajectoriesSrv :md5sum-) "bc0c14793d24668f68a3b8473f6545ec")
(setf (get kuavo_msgs::footPoseTargetTrajectoriesSrv :datatype-) "kuavo_msgs/footPoseTargetTrajectoriesSrv")
(setf (get kuavo_msgs::footPoseTargetTrajectoriesSrv :request) kuavo_msgs::footPoseTargetTrajectoriesSrvRequest)
(setf (get kuavo_msgs::footPoseTargetTrajectoriesSrv :response) kuavo_msgs::footPoseTargetTrajectoriesSrvResponse)

(defmethod kuavo_msgs::footPoseTargetTrajectoriesSrvRequest
  (:response () (instance kuavo_msgs::footPoseTargetTrajectoriesSrvResponse :init)))

(setf (get kuavo_msgs::footPoseTargetTrajectoriesSrvRequest :md5sum-) "bc0c14793d24668f68a3b8473f6545ec")
(setf (get kuavo_msgs::footPoseTargetTrajectoriesSrvRequest :datatype-) "kuavo_msgs/footPoseTargetTrajectoriesSrvRequest")
(setf (get kuavo_msgs::footPoseTargetTrajectoriesSrvRequest :definition-)
      "footPoseTargetTrajectories foot_pose_target_trajectories

================================================================================
MSG: kuavo_msgs/footPoseTargetTrajectories
float64[]    timeTrajectory
int32[]      footIndexTrajectory
footPose[]   footPoseTrajectory
================================================================================
MSG: kuavo_msgs/footPose
float64[4] footPose # x, y, z, yaw
---
bool success
")

(setf (get kuavo_msgs::footPoseTargetTrajectoriesSrvResponse :md5sum-) "bc0c14793d24668f68a3b8473f6545ec")
(setf (get kuavo_msgs::footPoseTargetTrajectoriesSrvResponse :datatype-) "kuavo_msgs/footPoseTargetTrajectoriesSrvResponse")
(setf (get kuavo_msgs::footPoseTargetTrajectoriesSrvResponse :definition-)
      "footPoseTargetTrajectories foot_pose_target_trajectories

================================================================================
MSG: kuavo_msgs/footPoseTargetTrajectories
float64[]    timeTrajectory
int32[]      footIndexTrajectory
footPose[]   footPoseTrajectory
================================================================================
MSG: kuavo_msgs/footPose
float64[4] footPose # x, y, z, yaw
---
bool success
")



(provide :kuavo_msgs/footPoseTargetTrajectoriesSrv "bc0c14793d24668f68a3b8473f6545ec")


