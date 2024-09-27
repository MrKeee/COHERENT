;; Auto-generated. Do not edit!


(when (boundp 'hademo::Action)
  (if (not (find-package "HADEMO"))
    (make-package "HADEMO"))
  (shadow 'Action (find-package "HADEMO")))
(unless (find-package "HADEMO::ACTION")
  (make-package "HADEMO::ACTION"))

(in-package "ROS")
;;//! \htmlinclude Action.msg.html


(defclass hademo::Action
  :super ros::object
  :slots (_franka_0 _franka_1 _franka_2 _aliengo_0 _aliengo_1 _aliengo_2 _quadrotor_0 _quadrotor_1 _quadrotor_2 ))

(defmethod hademo::Action
  (:init
   (&key
    ((:franka_0 __franka_0) (instance hademo::Func_and_Args :init))
    ((:franka_1 __franka_1) (instance hademo::Func_and_Args :init))
    ((:franka_2 __franka_2) (instance hademo::Func_and_Args :init))
    ((:aliengo_0 __aliengo_0) (instance hademo::Func_and_Args :init))
    ((:aliengo_1 __aliengo_1) (instance hademo::Func_and_Args :init))
    ((:aliengo_2 __aliengo_2) (instance hademo::Func_and_Args :init))
    ((:quadrotor_0 __quadrotor_0) (instance hademo::Func_and_Args :init))
    ((:quadrotor_1 __quadrotor_1) (instance hademo::Func_and_Args :init))
    ((:quadrotor_2 __quadrotor_2) (instance hademo::Func_and_Args :init))
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
    ;; hademo/Func_and_Args _franka_0
    (send _franka_0 :serialization-length)
    ;; hademo/Func_and_Args _franka_1
    (send _franka_1 :serialization-length)
    ;; hademo/Func_and_Args _franka_2
    (send _franka_2 :serialization-length)
    ;; hademo/Func_and_Args _aliengo_0
    (send _aliengo_0 :serialization-length)
    ;; hademo/Func_and_Args _aliengo_1
    (send _aliengo_1 :serialization-length)
    ;; hademo/Func_and_Args _aliengo_2
    (send _aliengo_2 :serialization-length)
    ;; hademo/Func_and_Args _quadrotor_0
    (send _quadrotor_0 :serialization-length)
    ;; hademo/Func_and_Args _quadrotor_1
    (send _quadrotor_1 :serialization-length)
    ;; hademo/Func_and_Args _quadrotor_2
    (send _quadrotor_2 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; hademo/Func_and_Args _franka_0
       (send _franka_0 :serialize s)
     ;; hademo/Func_and_Args _franka_1
       (send _franka_1 :serialize s)
     ;; hademo/Func_and_Args _franka_2
       (send _franka_2 :serialize s)
     ;; hademo/Func_and_Args _aliengo_0
       (send _aliengo_0 :serialize s)
     ;; hademo/Func_and_Args _aliengo_1
       (send _aliengo_1 :serialize s)
     ;; hademo/Func_and_Args _aliengo_2
       (send _aliengo_2 :serialize s)
     ;; hademo/Func_and_Args _quadrotor_0
       (send _quadrotor_0 :serialize s)
     ;; hademo/Func_and_Args _quadrotor_1
       (send _quadrotor_1 :serialize s)
     ;; hademo/Func_and_Args _quadrotor_2
       (send _quadrotor_2 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; hademo/Func_and_Args _franka_0
     (send _franka_0 :deserialize buf ptr-) (incf ptr- (send _franka_0 :serialization-length))
   ;; hademo/Func_and_Args _franka_1
     (send _franka_1 :deserialize buf ptr-) (incf ptr- (send _franka_1 :serialization-length))
   ;; hademo/Func_and_Args _franka_2
     (send _franka_2 :deserialize buf ptr-) (incf ptr- (send _franka_2 :serialization-length))
   ;; hademo/Func_and_Args _aliengo_0
     (send _aliengo_0 :deserialize buf ptr-) (incf ptr- (send _aliengo_0 :serialization-length))
   ;; hademo/Func_and_Args _aliengo_1
     (send _aliengo_1 :deserialize buf ptr-) (incf ptr- (send _aliengo_1 :serialization-length))
   ;; hademo/Func_and_Args _aliengo_2
     (send _aliengo_2 :deserialize buf ptr-) (incf ptr- (send _aliengo_2 :serialization-length))
   ;; hademo/Func_and_Args _quadrotor_0
     (send _quadrotor_0 :deserialize buf ptr-) (incf ptr- (send _quadrotor_0 :serialization-length))
   ;; hademo/Func_and_Args _quadrotor_1
     (send _quadrotor_1 :deserialize buf ptr-) (incf ptr- (send _quadrotor_1 :serialization-length))
   ;; hademo/Func_and_Args _quadrotor_2
     (send _quadrotor_2 :deserialize buf ptr-) (incf ptr- (send _quadrotor_2 :serialization-length))
   ;;
   self)
  )

(setf (get hademo::Action :md5sum-) "d976dee5559eb23c2568d9c0e79066c7")
(setf (get hademo::Action :datatype-) "hademo/Action")
(setf (get hademo::Action :definition-)
      "hademo/Func_and_Args franka_0
hademo/Func_and_Args franka_1
hademo/Func_and_Args franka_2
hademo/Func_and_Args aliengo_0
hademo/Func_and_Args aliengo_1
hademo/Func_and_Args aliengo_2
hademo/Func_and_Args quadrotor_0
hademo/Func_and_Args quadrotor_1
hademo/Func_and_Args quadrotor_2

================================================================================
MSG: hademo/Func_and_Args
bool has_func
string func_name
hademo/Args args

================================================================================
MSG: hademo/Args
bool has_args
string attached_prim_path
std_msgs/Float64MultiArray waypoint_pos
std_msgs/Float64MultiArray waypoint_ori
int32 waypoint_ind

================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
")



(provide :hademo/Action "d976dee5559eb23c2568d9c0e79066c7")


