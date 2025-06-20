;; Auto-generated. Do not edit!


(when (boundp 'h12pro_controller_node::playmusic)
  (if (not (find-package "H12PRO_CONTROLLER_NODE"))
    (make-package "H12PRO_CONTROLLER_NODE"))
  (shadow 'playmusic (find-package "H12PRO_CONTROLLER_NODE")))
(unless (find-package "H12PRO_CONTROLLER_NODE::PLAYMUSIC")
  (make-package "H12PRO_CONTROLLER_NODE::PLAYMUSIC"))
(unless (find-package "H12PRO_CONTROLLER_NODE::PLAYMUSICREQUEST")
  (make-package "H12PRO_CONTROLLER_NODE::PLAYMUSICREQUEST"))
(unless (find-package "H12PRO_CONTROLLER_NODE::PLAYMUSICRESPONSE")
  (make-package "H12PRO_CONTROLLER_NODE::PLAYMUSICRESPONSE"))

(in-package "ROS")





(defclass h12pro_controller_node::playmusicRequest
  :super ros::object
  :slots (_music_number _volume ))

(defmethod h12pro_controller_node::playmusicRequest
  (:init
   (&key
    ((:music_number __music_number) "")
    ((:volume __volume) 0)
    )
   (send-super :init)
   (setq _music_number (string __music_number))
   (setq _volume (round __volume))
   self)
  (:music_number
   (&optional __music_number)
   (if __music_number (setq _music_number __music_number)) _music_number)
  (:volume
   (&optional __volume)
   (if __volume (setq _volume __volume)) _volume)
  (:serialization-length
   ()
   (+
    ;; string _music_number
    4 (length _music_number)
    ;; int64 _volume
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _music_number
       (write-long (length _music_number) s) (princ _music_number s)
     ;; int64 _volume
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _volume (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _volume) (= (length (_volume . bv)) 2)) ;; bignum
              (write-long (ash (elt (_volume . bv) 0) 0) s)
              (write-long (ash (elt (_volume . bv) 1) -1) s))
             ((and (class _volume) (= (length (_volume . bv)) 1)) ;; big1
              (write-long (elt (_volume . bv) 0) s)
              (write-long (if (>= _volume 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _volume s)(write-long (if (>= _volume 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _music_number
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _music_number (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _volume
#+(or :alpha :irix6 :x86_64)
      (setf _volume (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _volume (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass h12pro_controller_node::playmusicResponse
  :super ros::object
  :slots (_success_flag ))

(defmethod h12pro_controller_node::playmusicResponse
  (:init
   (&key
    ((:success_flag __success_flag) nil)
    )
   (send-super :init)
   (setq _success_flag __success_flag)
   self)
  (:success_flag
   (&optional (__success_flag :null))
   (if (not (eq __success_flag :null)) (setq _success_flag __success_flag)) _success_flag)
  (:serialization-length
   ()
   (+
    ;; bool _success_flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success_flag
       (if _success_flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success_flag
     (setq _success_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass h12pro_controller_node::playmusic
  :super ros::object
  :slots ())

(setf (get h12pro_controller_node::playmusic :md5sum-) "3d99283888736c5e18f20ac685e5f8bf")
(setf (get h12pro_controller_node::playmusic :datatype-) "h12pro_controller_node/playmusic")
(setf (get h12pro_controller_node::playmusic :request) h12pro_controller_node::playmusicRequest)
(setf (get h12pro_controller_node::playmusic :response) h12pro_controller_node::playmusicResponse)

(defmethod h12pro_controller_node::playmusicRequest
  (:response () (instance h12pro_controller_node::playmusicResponse :init)))

(setf (get h12pro_controller_node::playmusicRequest :md5sum-) "3d99283888736c5e18f20ac685e5f8bf")
(setf (get h12pro_controller_node::playmusicRequest :datatype-) "h12pro_controller_node/playmusicRequest")
(setf (get h12pro_controller_node::playmusicRequest :definition-)
      "string music_number
int64 volume
---
bool success_flag

")

(setf (get h12pro_controller_node::playmusicResponse :md5sum-) "3d99283888736c5e18f20ac685e5f8bf")
(setf (get h12pro_controller_node::playmusicResponse :datatype-) "h12pro_controller_node/playmusicResponse")
(setf (get h12pro_controller_node::playmusicResponse :definition-)
      "string music_number
int64 volume
---
bool success_flag

")



(provide :h12pro_controller_node/playmusic "3d99283888736c5e18f20ac685e5f8bf")


