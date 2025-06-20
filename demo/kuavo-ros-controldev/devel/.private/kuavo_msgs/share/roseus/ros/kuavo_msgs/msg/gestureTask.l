;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::gestureTask)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'gestureTask (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::GESTURETASK")
  (make-package "KUAVO_MSGS::GESTURETASK"))

(in-package "ROS")
;;//! \htmlinclude gestureTask.msg.html


(defclass kuavo_msgs::gestureTask
  :super ros::object
  :slots (_gesture_name _hand_side ))

(defmethod kuavo_msgs::gestureTask
  (:init
   (&key
    ((:gesture_name __gesture_name) "")
    ((:hand_side __hand_side) 0)
    )
   (send-super :init)
   (setq _gesture_name (string __gesture_name))
   (setq _hand_side (round __hand_side))
   self)
  (:gesture_name
   (&optional __gesture_name)
   (if __gesture_name (setq _gesture_name __gesture_name)) _gesture_name)
  (:hand_side
   (&optional __hand_side)
   (if __hand_side (setq _hand_side __hand_side)) _hand_side)
  (:serialization-length
   ()
   (+
    ;; string _gesture_name
    4 (length _gesture_name)
    ;; int8 _hand_side
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _gesture_name
       (write-long (length _gesture_name) s) (princ _gesture_name s)
     ;; int8 _hand_side
       (write-byte _hand_side s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _gesture_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gesture_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _hand_side
     (setq _hand_side (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _hand_side 127) (setq _hand_side (- _hand_side 256)))
   ;;
   self)
  )

(setf (get kuavo_msgs::gestureTask :md5sum-) "be7fe1eba1df13c392c3a5d13b9f3dae")
(setf (get kuavo_msgs::gestureTask :datatype-) "kuavo_msgs/gestureTask")
(setf (get kuavo_msgs::gestureTask :definition-)
      "# This message is used to specify a gesture to execute.
# The gesture is triggered by providing its name and the side of the hand(s) to use.

string gesture_name  # Name of the gesture to execute
int8   hand_side    # Side of the hand to use (e.g., 0 for left, 1 for right, 2 for both)
")



(provide :kuavo_msgs/gestureTask "be7fe1eba1df13c392c3a5d13b9f3dae")


