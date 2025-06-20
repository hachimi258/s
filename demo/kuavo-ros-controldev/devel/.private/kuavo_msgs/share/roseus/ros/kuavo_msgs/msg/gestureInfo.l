;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::gestureInfo)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'gestureInfo (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::GESTUREINFO")
  (make-package "KUAVO_MSGS::GESTUREINFO"))

(in-package "ROS")
;;//! \htmlinclude gestureInfo.msg.html


(defclass kuavo_msgs::gestureInfo
  :super ros::object
  :slots (_gesture_name _alias _description ))

(defmethod kuavo_msgs::gestureInfo
  (:init
   (&key
    ((:gesture_name __gesture_name) "")
    ((:alias __alias) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:description __description) "")
    )
   (send-super :init)
   (setq _gesture_name (string __gesture_name))
   (setq _alias __alias)
   (setq _description (string __description))
   self)
  (:gesture_name
   (&optional __gesture_name)
   (if __gesture_name (setq _gesture_name __gesture_name)) _gesture_name)
  (:alias
   (&optional __alias)
   (if __alias (setq _alias __alias)) _alias)
  (:description
   (&optional __description)
   (if __description (setq _description __description)) _description)
  (:serialization-length
   ()
   (+
    ;; string _gesture_name
    4 (length _gesture_name)
    ;; string[] _alias
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _alias)) 4
    ;; string _description
    4 (length _description)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _gesture_name
       (write-long (length _gesture_name) s) (princ _gesture_name s)
     ;; string[] _alias
     (write-long (length _alias) s)
     (dolist (elem _alias)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string _description
       (write-long (length _description) s) (princ _description s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _gesture_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gesture_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _alias
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _alias (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _alias i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string _description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kuavo_msgs::gestureInfo :md5sum-) "65efb896db2f0292354e0a9098b39b97")
(setf (get kuavo_msgs::gestureInfo :datatype-) "kuavo_msgs/gestureInfo")
(setf (get kuavo_msgs::gestureInfo :definition-)
      "# This message defines the information for a single gesture.
# It includes the name, a list of aliases, and a description of the gesture.

# The name of the gesture.
string gesture_name

# A list of aliases for the gesture. These can be alternative names or shortcuts.
string[] alias

# A description of the gesture, providing more detailed information about its purpose and usage.
string description
")



(provide :kuavo_msgs/gestureInfo "65efb896db2f0292354e0a9098b39b97")


