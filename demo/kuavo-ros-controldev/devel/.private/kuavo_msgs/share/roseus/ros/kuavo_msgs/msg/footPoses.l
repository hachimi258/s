;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::footPoses)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'footPoses (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::FOOTPOSES")
  (make-package "KUAVO_MSGS::FOOTPOSES"))

(in-package "ROS")
;;//! \htmlinclude footPoses.msg.html


(defclass kuavo_msgs::footPoses
  :super ros::object
  :slots (_data ))

(defmethod kuavo_msgs::footPoses
  (:init
   (&key
    ((:data __data) ())
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; kuavo_msgs/footPose[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kuavo_msgs/footPose[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kuavo_msgs/footPose[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance kuavo_msgs::footPose :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kuavo_msgs::footPoses :md5sum-) "2fad89e6e8a2f3c5d0ec891cc7c76c35")
(setf (get kuavo_msgs::footPoses :datatype-) "kuavo_msgs/footPoses")
(setf (get kuavo_msgs::footPoses :definition-)
      "footPose[] data

================================================================================
MSG: kuavo_msgs/footPose
float64[4] footPose # x, y, z, yaw
float64[4] torsoPose # x, y, z, yaw

")



(provide :kuavo_msgs/footPoses "2fad89e6e8a2f3c5d0ec891cc7c76c35")


