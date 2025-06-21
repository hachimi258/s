;; Auto-generated. Do not edit!


(when (boundp 'kuavo_audio_player::playmusic)
  (if (not (find-package "KUAVO_AUDIO_PLAYER"))
    (make-package "KUAVO_AUDIO_PLAYER"))
  (shadow 'playmusic (find-package "KUAVO_AUDIO_PLAYER")))
(unless (find-package "KUAVO_AUDIO_PLAYER::PLAYMUSIC")
  (make-package "KUAVO_AUDIO_PLAYER::PLAYMUSIC"))
(unless (find-package "KUAVO_AUDIO_PLAYER::PLAYMUSICREQUEST")
  (make-package "KUAVO_AUDIO_PLAYER::PLAYMUSICREQUEST"))
(unless (find-package "KUAVO_AUDIO_PLAYER::PLAYMUSICRESPONSE")
  (make-package "KUAVO_AUDIO_PLAYER::PLAYMUSICRESPONSE"))

(in-package "ROS")





(defclass kuavo_audio_player::playmusicRequest
  :super ros::object
  :slots (_music_number _volume _speed ))

(defmethod kuavo_audio_player::playmusicRequest
  (:init
   (&key
    ((:music_number __music_number) "")
    ((:volume __volume) 0.0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _music_number (string __music_number))
   (setq _volume (float __volume))
   (setq _speed (float __speed))
   self)
  (:music_number
   (&optional __music_number)
   (if __music_number (setq _music_number __music_number)) _music_number)
  (:volume
   (&optional __volume)
   (if __volume (setq _volume __volume)) _volume)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; string _music_number
    4 (length _music_number)
    ;; float32 _volume
    4
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _music_number
       (write-long (length _music_number) s) (princ _music_number s)
     ;; float32 _volume
       (sys::poke _volume (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _music_number
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _music_number (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _volume
     (setq _volume (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass kuavo_audio_player::playmusicResponse
  :super ros::object
  :slots (_success_flag ))

(defmethod kuavo_audio_player::playmusicResponse
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

(defclass kuavo_audio_player::playmusic
  :super ros::object
  :slots ())

(setf (get kuavo_audio_player::playmusic :md5sum-) "348f2445d1e569f5dc4e72a19fb11934")
(setf (get kuavo_audio_player::playmusic :datatype-) "kuavo_audio_player/playmusic")
(setf (get kuavo_audio_player::playmusic :request) kuavo_audio_player::playmusicRequest)
(setf (get kuavo_audio_player::playmusic :response) kuavo_audio_player::playmusicResponse)

(defmethod kuavo_audio_player::playmusicRequest
  (:response () (instance kuavo_audio_player::playmusicResponse :init)))

(setf (get kuavo_audio_player::playmusicRequest :md5sum-) "348f2445d1e569f5dc4e72a19fb11934")
(setf (get kuavo_audio_player::playmusicRequest :datatype-) "kuavo_audio_player/playmusicRequest")
(setf (get kuavo_audio_player::playmusicRequest :definition-)
      "string music_number
float32 volume
float32 speed
---
bool success_flag

")

(setf (get kuavo_audio_player::playmusicResponse :md5sum-) "348f2445d1e569f5dc4e72a19fb11934")
(setf (get kuavo_audio_player::playmusicResponse :datatype-) "kuavo_audio_player/playmusicResponse")
(setf (get kuavo_audio_player::playmusicResponse :definition-)
      "string music_number
float32 volume
float32 speed
---
bool success_flag

")



(provide :kuavo_audio_player/playmusic "348f2445d1e569f5dc4e72a19fb11934")


