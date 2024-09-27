;; Auto-generated. Do not edit!


(when (boundp 'hademo::Args)
  (if (not (find-package "HADEMO"))
    (make-package "HADEMO"))
  (shadow 'Args (find-package "HADEMO")))
(unless (find-package "HADEMO::ARGS")
  (make-package "HADEMO::ARGS"))

(in-package "ROS")
;;//! \htmlinclude Args.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass hademo::Args
  :super ros::object
  :slots (_has_args _attached_prim_path _waypoint_pos _waypoint_ori _waypoint_ind ))

(defmethod hademo::Args
  (:init
   (&key
    ((:has_args __has_args) nil)
    ((:attached_prim_path __attached_prim_path) "")
    ((:waypoint_pos __waypoint_pos) (instance std_msgs::Float64MultiArray :init))
    ((:waypoint_ori __waypoint_ori) (instance std_msgs::Float64MultiArray :init))
    ((:waypoint_ind __waypoint_ind) 0)
    )
   (send-super :init)
   (setq _has_args __has_args)
   (setq _attached_prim_path (string __attached_prim_path))
   (setq _waypoint_pos __waypoint_pos)
   (setq _waypoint_ori __waypoint_ori)
   (setq _waypoint_ind (round __waypoint_ind))
   self)
  (:has_args
   (&optional (__has_args :null))
   (if (not (eq __has_args :null)) (setq _has_args __has_args)) _has_args)
  (:attached_prim_path
   (&optional __attached_prim_path)
   (if __attached_prim_path (setq _attached_prim_path __attached_prim_path)) _attached_prim_path)
  (:waypoint_pos
   (&rest __waypoint_pos)
   (if (keywordp (car __waypoint_pos))
       (send* _waypoint_pos __waypoint_pos)
     (progn
       (if __waypoint_pos (setq _waypoint_pos (car __waypoint_pos)))
       _waypoint_pos)))
  (:waypoint_ori
   (&rest __waypoint_ori)
   (if (keywordp (car __waypoint_ori))
       (send* _waypoint_ori __waypoint_ori)
     (progn
       (if __waypoint_ori (setq _waypoint_ori (car __waypoint_ori)))
       _waypoint_ori)))
  (:waypoint_ind
   (&optional __waypoint_ind)
   (if __waypoint_ind (setq _waypoint_ind __waypoint_ind)) _waypoint_ind)
  (:serialization-length
   ()
   (+
    ;; bool _has_args
    1
    ;; string _attached_prim_path
    4 (length _attached_prim_path)
    ;; std_msgs/Float64MultiArray _waypoint_pos
    (send _waypoint_pos :serialization-length)
    ;; std_msgs/Float64MultiArray _waypoint_ori
    (send _waypoint_ori :serialization-length)
    ;; int32 _waypoint_ind
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _has_args
       (if _has_args (write-byte -1 s) (write-byte 0 s))
     ;; string _attached_prim_path
       (write-long (length _attached_prim_path) s) (princ _attached_prim_path s)
     ;; std_msgs/Float64MultiArray _waypoint_pos
       (send _waypoint_pos :serialize s)
     ;; std_msgs/Float64MultiArray _waypoint_ori
       (send _waypoint_ori :serialize s)
     ;; int32 _waypoint_ind
       (write-long _waypoint_ind s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _has_args
     (setq _has_args (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _attached_prim_path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _attached_prim_path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; std_msgs/Float64MultiArray _waypoint_pos
     (send _waypoint_pos :deserialize buf ptr-) (incf ptr- (send _waypoint_pos :serialization-length))
   ;; std_msgs/Float64MultiArray _waypoint_ori
     (send _waypoint_ori :deserialize buf ptr-) (incf ptr- (send _waypoint_ori :serialization-length))
   ;; int32 _waypoint_ind
     (setq _waypoint_ind (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get hademo::Args :md5sum-) "482e2fefd3fc2e17f27a0aa075e67ef3")
(setf (get hademo::Args :datatype-) "hademo/Args")
(setf (get hademo::Args :definition-)
      "bool has_args
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



(provide :hademo/Args "482e2fefd3fc2e17f27a0aa075e67ef3")


