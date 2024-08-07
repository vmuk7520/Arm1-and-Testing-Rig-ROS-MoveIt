;; Auto-generated. Do not edit!


(when (boundp 'testing_rig_control::GetPosition)
  (if (not (find-package "TESTING_RIG_CONTROL"))
    (make-package "TESTING_RIG_CONTROL"))
  (shadow 'GetPosition (find-package "TESTING_RIG_CONTROL")))
(unless (find-package "TESTING_RIG_CONTROL::GETPOSITION")
  (make-package "TESTING_RIG_CONTROL::GETPOSITION"))
(unless (find-package "TESTING_RIG_CONTROL::GETPOSITIONREQUEST")
  (make-package "TESTING_RIG_CONTROL::GETPOSITIONREQUEST"))
(unless (find-package "TESTING_RIG_CONTROL::GETPOSITIONRESPONSE")
  (make-package "TESTING_RIG_CONTROL::GETPOSITIONRESPONSE"))

(in-package "ROS")





(defclass testing_rig_control::GetPositionRequest
  :super ros::object
  :slots (_id ))

(defmethod testing_rig_control::GetPositionRequest
  (:init
   (&key
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass testing_rig_control::GetPositionResponse
  :super ros::object
  :slots (_position ))

(defmethod testing_rig_control::GetPositionResponse
  (:init
   (&key
    ((:position __position) 0)
    )
   (send-super :init)
   (setq _position (round __position))
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:serialization-length
   ()
   (+
    ;; int32 _position
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _position
       (write-long _position s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass testing_rig_control::GetPosition
  :super ros::object
  :slots ())

(setf (get testing_rig_control::GetPosition :md5sum-) "b532ace3b383dc4c9e64687156423ac0")
(setf (get testing_rig_control::GetPosition :datatype-) "testing_rig_control/GetPosition")
(setf (get testing_rig_control::GetPosition :request) testing_rig_control::GetPositionRequest)
(setf (get testing_rig_control::GetPosition :response) testing_rig_control::GetPositionResponse)

(defmethod testing_rig_control::GetPositionRequest
  (:response () (instance testing_rig_control::GetPositionResponse :init)))

(setf (get testing_rig_control::GetPositionRequest :md5sum-) "b532ace3b383dc4c9e64687156423ac0")
(setf (get testing_rig_control::GetPositionRequest :datatype-) "testing_rig_control/GetPositionRequest")
(setf (get testing_rig_control::GetPositionRequest :definition-)
      "uint8 id
---
int32 position

")

(setf (get testing_rig_control::GetPositionResponse :md5sum-) "b532ace3b383dc4c9e64687156423ac0")
(setf (get testing_rig_control::GetPositionResponse :datatype-) "testing_rig_control/GetPositionResponse")
(setf (get testing_rig_control::GetPositionResponse :definition-)
      "uint8 id
---
int32 position

")



(provide :testing_rig_control/GetPosition "b532ace3b383dc4c9e64687156423ac0")


