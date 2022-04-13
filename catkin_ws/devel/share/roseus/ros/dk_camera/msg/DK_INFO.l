;; Auto-generated. Do not edit!


(when (boundp 'dk_camera::DK_INFO)
  (if (not (find-package "DK_CAMERA"))
    (make-package "DK_CAMERA"))
  (shadow 'DK_INFO (find-package "DK_CAMERA")))
(unless (find-package "DK_CAMERA::DK_INFO")
  (make-package "DK_CAMERA::DK_INFO"))

(in-package "ROS")
;;//! \htmlinclude DK_INFO.msg.html


(defclass dk_camera::DK_INFO
  :super ros::object
  :slots (_data _angle _station_number ))

(defmethod dk_camera::DK_INFO
  (:init
   (&key
    ((:data __data) "")
    ((:angle __angle) 0)
    ((:station_number __station_number) 0)
    )
   (send-super :init)
   (setq _data (string __data))
   (setq _angle (round __angle))
   (setq _station_number (round __station_number))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:station_number
   (&optional __station_number)
   (if __station_number (setq _station_number __station_number)) _station_number)
  (:serialization-length
   ()
   (+
    ;; string _data
    4 (length _data)
    ;; int32 _angle
    4
    ;; int32 _station_number
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;; int32 _angle
       (write-long _angle s)
     ;; int32 _station_number
       (write-long _station_number s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _angle
     (setq _angle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _station_number
     (setq _station_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dk_camera::DK_INFO :md5sum-) "a18b9ce4dd51326814ca565d3f1e4457")
(setf (get dk_camera::DK_INFO :datatype-) "dk_camera/DK_INFO")
(setf (get dk_camera::DK_INFO :definition-)
      "string data
int32 angle
int32 station_number

")



(provide :dk_camera/DK_INFO "a18b9ce4dd51326814ca565d3f1e4457")


