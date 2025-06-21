;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::jointData)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'jointData (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::JOINTDATA")
  (make-package "KUAVO_MSGS::JOINTDATA"))

(in-package "ROS")
;;//! \htmlinclude jointData.msg.html


(defclass kuavo_msgs::jointData
  :super ros::object
  :slots (_joint_q _joint_v _joint_vd _joint_torque ))

(defmethod kuavo_msgs::jointData
  (:init
   (&key
    ((:joint_q __joint_q) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_v __joint_v) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_vd __joint_vd) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_torque __joint_torque) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _joint_q __joint_q)
   (setq _joint_v __joint_v)
   (setq _joint_vd __joint_vd)
   (setq _joint_torque __joint_torque)
   self)
  (:joint_q
   (&optional __joint_q)
   (if __joint_q (setq _joint_q __joint_q)) _joint_q)
  (:joint_v
   (&optional __joint_v)
   (if __joint_v (setq _joint_v __joint_v)) _joint_v)
  (:joint_vd
   (&optional __joint_vd)
   (if __joint_vd (setq _joint_vd __joint_vd)) _joint_vd)
  (:joint_torque
   (&optional __joint_torque)
   (if __joint_torque (setq _joint_torque __joint_torque)) _joint_torque)
  (:serialization-length
   ()
   (+
    ;; float64[] _joint_q
    (* 8    (length _joint_q)) 4
    ;; float64[] _joint_v
    (* 8    (length _joint_v)) 4
    ;; float64[] _joint_vd
    (* 8    (length _joint_vd)) 4
    ;; float64[] _joint_torque
    (* 8    (length _joint_torque)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _joint_q
     (write-long (length _joint_q) s)
     (dotimes (i (length _joint_q))
       (sys::poke (elt _joint_q i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_v
     (write-long (length _joint_v) s)
     (dotimes (i (length _joint_v))
       (sys::poke (elt _joint_v i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_vd
     (write-long (length _joint_vd) s)
     (dotimes (i (length _joint_vd))
       (sys::poke (elt _joint_vd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_torque
     (write-long (length _joint_torque) s)
     (dotimes (i (length _joint_torque))
       (sys::poke (elt _joint_torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _joint_q
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_q (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_q i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_v
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_v (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_v i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_vd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_vd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_vd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_torque
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_torque (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get kuavo_msgs::jointData :md5sum-) "2e01436cbc40e94e8fe8f54a2c4ea282")
(setf (get kuavo_msgs::jointData :datatype-) "kuavo_msgs/jointData")
(setf (get kuavo_msgs::jointData :definition-)
      "float64[] joint_q  
float64[] joint_v  
float64[] joint_vd    
float64[] joint_torque  

")



(provide :kuavo_msgs/jointData "2e01436cbc40e94e8fe8f54a2c4ea282")


