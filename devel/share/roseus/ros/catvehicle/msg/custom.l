;; Auto-generated. Do not edit!


(when (boundp 'catvehicle::custom)
  (if (not (find-package "CATVEHICLE"))
    (make-package "CATVEHICLE"))
  (shadow 'custom (find-package "CATVEHICLE")))
(unless (find-package "CATVEHICLE::CUSTOM")
  (make-package "CATVEHICLE::CUSTOM"))

(in-package "ROS")
;;//! \htmlinclude custom.msg.html


(defclass catvehicle::custom
  :super ros::object
  :slots (_x _y ))

(defmethod catvehicle::custom
  (:init
   (&key
    ((:x __x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y __y) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _x __x)
   (setq _y __y)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; float64[] _x
    (* 8    (length _x)) 4
    ;; float64[] _y
    (* 8    (length _y)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _x
     (write-long (length _x) s)
     (dotimes (i (length _x))
       (sys::poke (elt _x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (sys::poke (elt _y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get catvehicle::custom :md5sum-) "796182af49bd48aa8ba60ed21519725c")
(setf (get catvehicle::custom :datatype-) "catvehicle/custom")
(setf (get catvehicle::custom :definition-)
      "float64[] x
float64[] y
")



(provide :catvehicle/custom "796182af49bd48aa8ba60ed21519725c")


