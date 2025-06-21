;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::touchSensorStatus)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'touchSensorStatus (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::TOUCHSENSORSTATUS")
  (make-package "KUAVO_MSGS::TOUCHSENSORSTATUS"))

(in-package "ROS")
;;//! \htmlinclude touchSensorStatus.msg.html


(defclass kuavo_msgs::touchSensorStatus
  :super ros::object
  :slots (_normal_force1 _normal_force2 _normal_force3 _tangential_force1 _tangential_force2 _tangential_force3 _tangential_direction1 _tangential_direction2 _tangential_direction3 _self_proximity1 _self_proximity2 _mutual_proximity _status ))

(defmethod kuavo_msgs::touchSensorStatus
  (:init
   (&key
    ((:normal_force1 __normal_force1) 0)
    ((:normal_force2 __normal_force2) 0)
    ((:normal_force3 __normal_force3) 0)
    ((:tangential_force1 __tangential_force1) 0)
    ((:tangential_force2 __tangential_force2) 0)
    ((:tangential_force3 __tangential_force3) 0)
    ((:tangential_direction1 __tangential_direction1) 0)
    ((:tangential_direction2 __tangential_direction2) 0)
    ((:tangential_direction3 __tangential_direction3) 0)
    ((:self_proximity1 __self_proximity1) 0)
    ((:self_proximity2 __self_proximity2) 0)
    ((:mutual_proximity __mutual_proximity) 0)
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _normal_force1 (round __normal_force1))
   (setq _normal_force2 (round __normal_force2))
   (setq _normal_force3 (round __normal_force3))
   (setq _tangential_force1 (round __tangential_force1))
   (setq _tangential_force2 (round __tangential_force2))
   (setq _tangential_force3 (round __tangential_force3))
   (setq _tangential_direction1 (round __tangential_direction1))
   (setq _tangential_direction2 (round __tangential_direction2))
   (setq _tangential_direction3 (round __tangential_direction3))
   (setq _self_proximity1 (round __self_proximity1))
   (setq _self_proximity2 (round __self_proximity2))
   (setq _mutual_proximity (round __mutual_proximity))
   (setq _status (round __status))
   self)
  (:normal_force1
   (&optional __normal_force1)
   (if __normal_force1 (setq _normal_force1 __normal_force1)) _normal_force1)
  (:normal_force2
   (&optional __normal_force2)
   (if __normal_force2 (setq _normal_force2 __normal_force2)) _normal_force2)
  (:normal_force3
   (&optional __normal_force3)
   (if __normal_force3 (setq _normal_force3 __normal_force3)) _normal_force3)
  (:tangential_force1
   (&optional __tangential_force1)
   (if __tangential_force1 (setq _tangential_force1 __tangential_force1)) _tangential_force1)
  (:tangential_force2
   (&optional __tangential_force2)
   (if __tangential_force2 (setq _tangential_force2 __tangential_force2)) _tangential_force2)
  (:tangential_force3
   (&optional __tangential_force3)
   (if __tangential_force3 (setq _tangential_force3 __tangential_force3)) _tangential_force3)
  (:tangential_direction1
   (&optional __tangential_direction1)
   (if __tangential_direction1 (setq _tangential_direction1 __tangential_direction1)) _tangential_direction1)
  (:tangential_direction2
   (&optional __tangential_direction2)
   (if __tangential_direction2 (setq _tangential_direction2 __tangential_direction2)) _tangential_direction2)
  (:tangential_direction3
   (&optional __tangential_direction3)
   (if __tangential_direction3 (setq _tangential_direction3 __tangential_direction3)) _tangential_direction3)
  (:self_proximity1
   (&optional __self_proximity1)
   (if __self_proximity1 (setq _self_proximity1 __self_proximity1)) _self_proximity1)
  (:self_proximity2
   (&optional __self_proximity2)
   (if __self_proximity2 (setq _self_proximity2 __self_proximity2)) _self_proximity2)
  (:mutual_proximity
   (&optional __mutual_proximity)
   (if __mutual_proximity (setq _mutual_proximity __mutual_proximity)) _mutual_proximity)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; uint16 _normal_force1
    2
    ;; uint16 _normal_force2
    2
    ;; uint16 _normal_force3
    2
    ;; uint16 _tangential_force1
    2
    ;; uint16 _tangential_force2
    2
    ;; uint16 _tangential_force3
    2
    ;; uint16 _tangential_direction1
    2
    ;; uint16 _tangential_direction2
    2
    ;; uint16 _tangential_direction3
    2
    ;; uint32 _self_proximity1
    4
    ;; uint32 _self_proximity2
    4
    ;; uint32 _mutual_proximity
    4
    ;; uint16 _status
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _normal_force1
       (write-word _normal_force1 s)
     ;; uint16 _normal_force2
       (write-word _normal_force2 s)
     ;; uint16 _normal_force3
       (write-word _normal_force3 s)
     ;; uint16 _tangential_force1
       (write-word _tangential_force1 s)
     ;; uint16 _tangential_force2
       (write-word _tangential_force2 s)
     ;; uint16 _tangential_force3
       (write-word _tangential_force3 s)
     ;; uint16 _tangential_direction1
       (write-word _tangential_direction1 s)
     ;; uint16 _tangential_direction2
       (write-word _tangential_direction2 s)
     ;; uint16 _tangential_direction3
       (write-word _tangential_direction3 s)
     ;; uint32 _self_proximity1
       (write-long _self_proximity1 s)
     ;; uint32 _self_proximity2
       (write-long _self_proximity2 s)
     ;; uint32 _mutual_proximity
       (write-long _mutual_proximity s)
     ;; uint16 _status
       (write-word _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _normal_force1
     (setq _normal_force1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _normal_force2
     (setq _normal_force2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _normal_force3
     (setq _normal_force3 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _tangential_force1
     (setq _tangential_force1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _tangential_force2
     (setq _tangential_force2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _tangential_force3
     (setq _tangential_force3 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _tangential_direction1
     (setq _tangential_direction1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _tangential_direction2
     (setq _tangential_direction2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _tangential_direction3
     (setq _tangential_direction3 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint32 _self_proximity1
     (setq _self_proximity1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _self_proximity2
     (setq _self_proximity2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _mutual_proximity
     (setq _mutual_proximity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _status
     (setq _status (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get kuavo_msgs::touchSensorStatus :md5sum-) "08cd59dc396363cba4d4f01df99ec86c")
(setf (get kuavo_msgs::touchSensorStatus :datatype-) "kuavo_msgs/touchSensorStatus")
(setf (get kuavo_msgs::touchSensorStatus :definition-)
      "uint16 normal_force1  # 法向力1
uint16 normal_force2  # 法向力2
uint16 normal_force3  # 法向力3
uint16 tangential_force1  # 切向力1
uint16 tangential_force2  # 切向力2
uint16 tangential_force3  # 切向力3
uint16 tangential_direction1  # 切向力方向1
uint16 tangential_direction2  # 切向力方向2
uint16 tangential_direction3  # 切向力方向3
uint32 self_proximity1  # 自电容接近传感器1
uint32 self_proximity2  # 自电容接近传感器2
uint32 mutual_proximity  # 互电容接近传感器
uint16 status  # 传感器状态
")



(provide :kuavo_msgs/touchSensorStatus "08cd59dc396363cba4d4f01df99ec86c")


