;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::yoloOutputData)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'yoloOutputData (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::YOLOOUTPUTDATA")
  (make-package "KUAVO_MSGS::YOLOOUTPUTDATA"))

(in-package "ROS")
;;//! \htmlinclude yoloOutputData.msg.html


(defclass kuavo_msgs::yoloOutputData
  :super ros::object
  :slots (_class_name _class_id _confidence _x_pos _y_pos _height _width ))

(defmethod kuavo_msgs::yoloOutputData
  (:init
   (&key
    ((:class_name __class_name) "")
    ((:class_id __class_id) 0)
    ((:confidence __confidence) 0.0)
    ((:x_pos __x_pos) 0.0)
    ((:y_pos __y_pos) 0.0)
    ((:height __height) 0.0)
    ((:width __width) 0.0)
    )
   (send-super :init)
   (setq _class_name (string __class_name))
   (setq _class_id (round __class_id))
   (setq _confidence (float __confidence))
   (setq _x_pos (float __x_pos))
   (setq _y_pos (float __y_pos))
   (setq _height (float __height))
   (setq _width (float __width))
   self)
  (:class_name
   (&optional __class_name)
   (if __class_name (setq _class_name __class_name)) _class_name)
  (:class_id
   (&optional __class_id)
   (if __class_id (setq _class_id __class_id)) _class_id)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:x_pos
   (&optional __x_pos)
   (if __x_pos (setq _x_pos __x_pos)) _x_pos)
  (:y_pos
   (&optional __y_pos)
   (if __y_pos (setq _y_pos __y_pos)) _y_pos)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:serialization-length
   ()
   (+
    ;; string _class_name
    4 (length _class_name)
    ;; int32 _class_id
    4
    ;; float32 _confidence
    4
    ;; float32 _x_pos
    4
    ;; float32 _y_pos
    4
    ;; float32 _height
    4
    ;; float32 _width
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _class_name
       (write-long (length _class_name) s) (princ _class_name s)
     ;; int32 _class_id
       (write-long _class_id s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_pos
       (sys::poke _x_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_pos
       (sys::poke _y_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _width
       (sys::poke _width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _class_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _class_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _class_id
     (setq _class_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_pos
     (setq _x_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_pos
     (setq _y_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _width
     (setq _width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kuavo_msgs::yoloOutputData :md5sum-) "137434f9d0388d58c5d9dc2d88f9e8f1")
(setf (get kuavo_msgs::yoloOutputData :datatype-) "kuavo_msgs/yoloOutputData")
(setf (get kuavo_msgs::yoloOutputData :definition-)
      "string class_name
int32 class_id
float32 confidence
float32 x_pos
float32 y_pos
float32 height
float32 width
")



(provide :kuavo_msgs/yoloOutputData "137434f9d0388d58c5d9dc2d88f9e8f1")


