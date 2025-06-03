;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::tagDataArray)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'tagDataArray (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::TAGDATAARRAY")
  (make-package "KUAVO_MSGS::TAGDATAARRAY"))

(in-package "ROS")
;;//! \htmlinclude tagDataArray.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass kuavo_msgs::tagDataArray
  :super ros::object
  :slots (_tag_ids _tag_poses ))

(defmethod kuavo_msgs::tagDataArray
  (:init
   (&key
    ((:tag_ids __tag_ids) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tag_poses __tag_poses) ())
    )
   (send-super :init)
   (setq _tag_ids __tag_ids)
   (setq _tag_poses __tag_poses)
   self)
  (:tag_ids
   (&optional __tag_ids)
   (if __tag_ids (setq _tag_ids __tag_ids)) _tag_ids)
  (:tag_poses
   (&rest __tag_poses)
   (if (keywordp (car __tag_poses))
       (send* _tag_poses __tag_poses)
     (progn
       (if __tag_poses (setq _tag_poses (car __tag_poses)))
       _tag_poses)))
  (:serialization-length
   ()
   (+
    ;; int32[] _tag_ids
    (* 4    (length _tag_ids)) 4
    ;; geometry_msgs/Pose[] _tag_poses
    (apply #'+ (send-all _tag_poses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _tag_ids
     (write-long (length _tag_ids) s)
     (dotimes (i (length _tag_ids))
       (write-long (elt _tag_ids i) s)
       )
     ;; geometry_msgs/Pose[] _tag_poses
     (write-long (length _tag_poses) s)
     (dolist (elem _tag_poses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _tag_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tag_ids (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tag_ids i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; geometry_msgs/Pose[] _tag_poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tag_poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _tag_poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kuavo_msgs::tagDataArray :md5sum-) "454da6edf551b421dda595d3272ef7ac")
(setf (get kuavo_msgs::tagDataArray :datatype-) "kuavo_msgs/tagDataArray")
(setf (get kuavo_msgs::tagDataArray :definition-)
      "int32[] tag_ids
geometry_msgs/Pose[] tag_poses

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

")



(provide :kuavo_msgs/tagDataArray "454da6edf551b421dda595d3272ef7ac")


