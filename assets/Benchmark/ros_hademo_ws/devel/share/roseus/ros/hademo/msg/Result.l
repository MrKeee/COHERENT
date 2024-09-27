;; Auto-generated. Do not edit!


(when (boundp 'hademo::Result)
  (if (not (find-package "HADEMO"))
    (make-package "HADEMO"))
  (shadow 'Result (find-package "HADEMO")))
(unless (find-package "HADEMO::RESULT")
  (make-package "HADEMO::RESULT"))

(in-package "ROS")
;;//! \htmlinclude Result.msg.html


(defclass hademo::Result
  :super ros::object
  :slots (_franka_0 _franka_1 _franka_2 _aliengo_0 _aliengo_1 _aliengo_2 _quadrotor_0 _quadrotor_1 _quadrotor_2 ))

(defmethod hademo::Result
  (:init
   (&key
    ((:franka_0 __franka_0) (instance hademo::ResultInfo :init))
    ((:franka_1 __franka_1) (instance hademo::ResultInfo :init))
    ((:franka_2 __franka_2) (instance hademo::ResultInfo :init))
    ((:aliengo_0 __aliengo_0) (instance hademo::ResultInfo :init))
    ((:aliengo_1 __aliengo_1) (instance hademo::ResultInfo :init))
    ((:aliengo_2 __aliengo_2) (instance hademo::ResultInfo :init))
    ((:quadrotor_0 __quadrotor_0) (instance hademo::ResultInfo :init))
    ((:quadrotor_1 __quadrotor_1) (instance hademo::ResultInfo :init))
    ((:quadrotor_2 __quadrotor_2) (instance hademo::ResultInfo :init))
    )
   (send-super :init)
   (setq _franka_0 __franka_0)
   (setq _franka_1 __franka_1)
   (setq _franka_2 __franka_2)
   (setq _aliengo_0 __aliengo_0)
   (setq _aliengo_1 __aliengo_1)
   (setq _aliengo_2 __aliengo_2)
   (setq _quadrotor_0 __quadrotor_0)
   (setq _quadrotor_1 __quadrotor_1)
   (setq _quadrotor_2 __quadrotor_2)
   self)
  (:franka_0
   (&rest __franka_0)
   (if (keywordp (car __franka_0))
       (send* _franka_0 __franka_0)
     (progn
       (if __franka_0 (setq _franka_0 (car __franka_0)))
       _franka_0)))
  (:franka_1
   (&rest __franka_1)
   (if (keywordp (car __franka_1))
       (send* _franka_1 __franka_1)
     (progn
       (if __franka_1 (setq _franka_1 (car __franka_1)))
       _franka_1)))
  (:franka_2
   (&rest __franka_2)
   (if (keywordp (car __franka_2))
       (send* _franka_2 __franka_2)
     (progn
       (if __franka_2 (setq _franka_2 (car __franka_2)))
       _franka_2)))
  (:aliengo_0
   (&rest __aliengo_0)
   (if (keywordp (car __aliengo_0))
       (send* _aliengo_0 __aliengo_0)
     (progn
       (if __aliengo_0 (setq _aliengo_0 (car __aliengo_0)))
       _aliengo_0)))
  (:aliengo_1
   (&rest __aliengo_1)
   (if (keywordp (car __aliengo_1))
       (send* _aliengo_1 __aliengo_1)
     (progn
       (if __aliengo_1 (setq _aliengo_1 (car __aliengo_1)))
       _aliengo_1)))
  (:aliengo_2
   (&rest __aliengo_2)
   (if (keywordp (car __aliengo_2))
       (send* _aliengo_2 __aliengo_2)
     (progn
       (if __aliengo_2 (setq _aliengo_2 (car __aliengo_2)))
       _aliengo_2)))
  (:quadrotor_0
   (&rest __quadrotor_0)
   (if (keywordp (car __quadrotor_0))
       (send* _quadrotor_0 __quadrotor_0)
     (progn
       (if __quadrotor_0 (setq _quadrotor_0 (car __quadrotor_0)))
       _quadrotor_0)))
  (:quadrotor_1
   (&rest __quadrotor_1)
   (if (keywordp (car __quadrotor_1))
       (send* _quadrotor_1 __quadrotor_1)
     (progn
       (if __quadrotor_1 (setq _quadrotor_1 (car __quadrotor_1)))
       _quadrotor_1)))
  (:quadrotor_2
   (&rest __quadrotor_2)
   (if (keywordp (car __quadrotor_2))
       (send* _quadrotor_2 __quadrotor_2)
     (progn
       (if __quadrotor_2 (setq _quadrotor_2 (car __quadrotor_2)))
       _quadrotor_2)))
  (:serialization-length
   ()
   (+
    ;; hademo/ResultInfo _franka_0
    (send _franka_0 :serialization-length)
    ;; hademo/ResultInfo _franka_1
    (send _franka_1 :serialization-length)
    ;; hademo/ResultInfo _franka_2
    (send _franka_2 :serialization-length)
    ;; hademo/ResultInfo _aliengo_0
    (send _aliengo_0 :serialization-length)
    ;; hademo/ResultInfo _aliengo_1
    (send _aliengo_1 :serialization-length)
    ;; hademo/ResultInfo _aliengo_2
    (send _aliengo_2 :serialization-length)
    ;; hademo/ResultInfo _quadrotor_0
    (send _quadrotor_0 :serialization-length)
    ;; hademo/ResultInfo _quadrotor_1
    (send _quadrotor_1 :serialization-length)
    ;; hademo/ResultInfo _quadrotor_2
    (send _quadrotor_2 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; hademo/ResultInfo _franka_0
       (send _franka_0 :serialize s)
     ;; hademo/ResultInfo _franka_1
       (send _franka_1 :serialize s)
     ;; hademo/ResultInfo _franka_2
       (send _franka_2 :serialize s)
     ;; hademo/ResultInfo _aliengo_0
       (send _aliengo_0 :serialize s)
     ;; hademo/ResultInfo _aliengo_1
       (send _aliengo_1 :serialize s)
     ;; hademo/ResultInfo _aliengo_2
       (send _aliengo_2 :serialize s)
     ;; hademo/ResultInfo _quadrotor_0
       (send _quadrotor_0 :serialize s)
     ;; hademo/ResultInfo _quadrotor_1
       (send _quadrotor_1 :serialize s)
     ;; hademo/ResultInfo _quadrotor_2
       (send _quadrotor_2 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; hademo/ResultInfo _franka_0
     (send _franka_0 :deserialize buf ptr-) (incf ptr- (send _franka_0 :serialization-length))
   ;; hademo/ResultInfo _franka_1
     (send _franka_1 :deserialize buf ptr-) (incf ptr- (send _franka_1 :serialization-length))
   ;; hademo/ResultInfo _franka_2
     (send _franka_2 :deserialize buf ptr-) (incf ptr- (send _franka_2 :serialization-length))
   ;; hademo/ResultInfo _aliengo_0
     (send _aliengo_0 :deserialize buf ptr-) (incf ptr- (send _aliengo_0 :serialization-length))
   ;; hademo/ResultInfo _aliengo_1
     (send _aliengo_1 :deserialize buf ptr-) (incf ptr- (send _aliengo_1 :serialization-length))
   ;; hademo/ResultInfo _aliengo_2
     (send _aliengo_2 :deserialize buf ptr-) (incf ptr- (send _aliengo_2 :serialization-length))
   ;; hademo/ResultInfo _quadrotor_0
     (send _quadrotor_0 :deserialize buf ptr-) (incf ptr- (send _quadrotor_0 :serialization-length))
   ;; hademo/ResultInfo _quadrotor_1
     (send _quadrotor_1 :deserialize buf ptr-) (incf ptr- (send _quadrotor_1 :serialization-length))
   ;; hademo/ResultInfo _quadrotor_2
     (send _quadrotor_2 :deserialize buf ptr-) (incf ptr- (send _quadrotor_2 :serialization-length))
   ;;
   self)
  )

(setf (get hademo::Result :md5sum-) "6a97bcb6cbe98951369cc3755518f34c")
(setf (get hademo::Result :datatype-) "hademo/Result")
(setf (get hademo::Result :definition-)
      "hademo/ResultInfo franka_0
hademo/ResultInfo franka_1
hademo/ResultInfo franka_2
hademo/ResultInfo aliengo_0
hademo/ResultInfo aliengo_1
hademo/ResultInfo aliengo_2
hademo/ResultInfo quadrotor_0
hademo/ResultInfo quadrotor_1
hademo/ResultInfo quadrotor_2

================================================================================
MSG: hademo/ResultInfo
bool has_result
bool success
string info

")



(provide :hademo/Result "6a97bcb6cbe98951369cc3755518f34c")


