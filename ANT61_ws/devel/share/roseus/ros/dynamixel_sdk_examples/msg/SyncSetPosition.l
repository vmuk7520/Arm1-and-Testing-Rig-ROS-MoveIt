;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_sdk_examples::SyncSetPosition)
  (if (not (find-package "DYNAMIXEL_SDK_EXAMPLES"))
    (make-package "DYNAMIXEL_SDK_EXAMPLES"))
  (shadow 'SyncSetPosition (find-package "DYNAMIXEL_SDK_EXAMPLES")))
(unless (find-package "DYNAMIXEL_SDK_EXAMPLES::SYNCSETPOSITION")
  (make-package "DYNAMIXEL_SDK_EXAMPLES::SYNCSETPOSITION"))

(in-package "ROS")
;;//! \htmlinclude SyncSetPosition.msg.html


(defclass dynamixel_sdk_examples::SyncSetPosition
  :super ros::object
  :slots (_id0 _id1 _id2 _position0 _position1 _position2 ))

(defmethod dynamixel_sdk_examples::SyncSetPosition
  (:init
   (&key
    ((:id0 __id0) 0)
    ((:id1 __id1) 0)
    ((:id2 __id2) 0)
    ((:position0 __position0) 0)
    ((:position1 __position1) 0)
    ((:position2 __position2) 0)
    )
   (send-super :init)
   (setq _id0 (round __id0))
   (setq _id1 (round __id1))
   (setq _id2 (round __id2))
   (setq _position0 (round __position0))
   (setq _position1 (round __position1))
   (setq _position2 (round __position2))
   self)
  (:id0
   (&optional __id0)
   (if __id0 (setq _id0 __id0)) _id0)
  (:id1
   (&optional __id1)
   (if __id1 (setq _id1 __id1)) _id1)
  (:id2
   (&optional __id2)
   (if __id2 (setq _id2 __id2)) _id2)
  (:position0
   (&optional __position0)
   (if __position0 (setq _position0 __position0)) _position0)
  (:position1
   (&optional __position1)
   (if __position1 (setq _position1 __position1)) _position1)
  (:position2
   (&optional __position2)
   (if __position2 (setq _position2 __position2)) _position2)
  (:serialization-length
   ()
   (+
    ;; uint8 _id0
    1
    ;; uint8 _id1
    1
    ;; uint8 _id2
    1
    ;; int32 _position0
    4
    ;; int32 _position1
    4
    ;; int32 _position2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id0
       (write-byte _id0 s)
     ;; uint8 _id1
       (write-byte _id1 s)
     ;; uint8 _id2
       (write-byte _id2 s)
     ;; int32 _position0
       (write-long _position0 s)
     ;; int32 _position1
       (write-long _position1 s)
     ;; int32 _position2
       (write-long _position2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id0
     (setq _id0 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _id1
     (setq _id1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _id2
     (setq _id2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _position0
     (setq _position0 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _position1
     (setq _position1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _position2
     (setq _position2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dynamixel_sdk_examples::SyncSetPosition :md5sum-) "1f8428b176ccf88bc7f2c826e9b347e6")
(setf (get dynamixel_sdk_examples::SyncSetPosition :datatype-) "dynamixel_sdk_examples/SyncSetPosition")
(setf (get dynamixel_sdk_examples::SyncSetPosition :definition-)
      "uint8 id0
uint8 id1
uint8 id2
int32 position0
int32 position1
int32 position2

")



(provide :dynamixel_sdk_examples/SyncSetPosition "1f8428b176ccf88bc7f2c826e9b347e6")


