;; Auto-generated. Do not edit!


(when (boundp 'hademo::Func_and_Args)
  (if (not (find-package "HADEMO"))
    (make-package "HADEMO"))
  (shadow 'Func_and_Args (find-package "HADEMO")))
(unless (find-package "HADEMO::FUNC_AND_ARGS")
  (make-package "HADEMO::FUNC_AND_ARGS"))

(in-package "ROS")
;;//! \htmlinclude Func_and_Args.msg.html


(defclass hademo::Func_and_Args
  :super ros::object
  :slots (_has_func _func_name _args ))

(defmethod hademo::Func_and_Args
  (:init
   (&key
    ((:has_func __has_func) nil)
    ((:func_name __func_name) "")
    ((:args __args) (instance hademo::Args :init))
    )
   (send-super :init)
   (setq _has_func __has_func)
   (setq _func_name (string __func_name))
   (setq _args __args)
   self)
  (:has_func
   (&optional (__has_func :null))
   (if (not (eq __has_func :null)) (setq _has_func __has_func)) _has_func)
  (:func_name
   (&optional __func_name)
   (if __func_name (setq _func_name __func_name)) _func_name)
  (:args
   (&rest __args)
   (if (keywordp (car __args))
       (send* _args __args)
     (progn
       (if __args (setq _args (car __args)))
       _args)))
  (:serialization-length
   ()
   (+
    ;; bool _has_func
    1
    ;; string _func_name
    4 (length _func_name)
    ;; hademo/Args _args
    (send _args :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _has_func
       (if _has_func (write-byte -1 s) (write-byte 0 s))
     ;; string _func_name
       (write-long (length _func_name) s) (princ _func_name s)
     ;; hademo/Args _args
       (send _args :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _has_func
     (setq _has_func (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _func_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _func_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; hademo/Args _args
     (send _args :deserialize buf ptr-) (incf ptr- (send _args :serialization-length))
   ;;
   self)
  )

(setf (get hademo::Func_and_Args :md5sum-) "2cf85b88f235eeb7b23bd9784805fafa")
(setf (get hademo::Func_and_Args :datatype-) "hademo/Func_and_Args")
(setf (get hademo::Func_and_Args :definition-)
      "bool has_func
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



(provide :hademo/Func_and_Args "2cf85b88f235eeb7b23bd9784805fafa")


