;; Auto-generated. Do not edit!


(when (boundp 'testing_rig_control::SetPosition)
  (if (not (find-package "TESTING_RIG_CONTROL"))
    (make-package "TESTING_RIG_CONTROL"))
  (shadow 'SetPosition (find-package "TESTING_RIG_CONTROL")))
(unless (find-package "TESTING_RIG_CONTROL::SETPOSITION")
  (make-package "TESTING_RIG_CONTROL::SETPOSITION"))

(in-package "ROS")
;;//! \htmlinclude SetPosition.msg.html


(defclass testing_rig_control::SetPosition
  :super ros::object
  :slots (_id _position ))

(defmethod testing_rig_control::SetPosition
  (:init
   (&key
    ((:id __id) 0)
    ((:position __position) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _position (round __position))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; int32 _position
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; int32 _position
       (write-long _position s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get testing_rig_control::SetPosition :md5sum-) "0a775458729eb3272bc88b4f5f764cc8")
(setf (get testing_rig_control::SetPosition :datatype-) "testing_rig_control/SetPosition")
(setf (get testing_rig_control::SetPosition :definition-)
      "uint8 id
int32 position

")



(provide :testing_rig_control/SetPosition "0a775458729eb3272bc88b4f5f764cc8")


