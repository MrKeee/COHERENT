; Auto-generated. Do not edit!


(cl:in-package hademo-msg)


;//! \htmlinclude Action.msg.html

(cl:defclass <Action> (roslisp-msg-protocol:ros-message)
  ((franka_0
    :reader franka_0
    :initarg :franka_0
    :type hademo-msg:Func_and_Args
    :initform (cl:make-instance 'hademo-msg:Func_and_Args))
   (franka_1
    :reader franka_1
    :initarg :franka_1
    :type hademo-msg:Func_and_Args
    :initform (cl:make-instance 'hademo-msg:Func_and_Args))
   (franka_2
    :reader franka_2
    :initarg :franka_2
    :type hademo-msg:Func_and_Args
    :initform (cl:make-instance 'hademo-msg:Func_and_Args))
   (aliengo_0
    :reader aliengo_0
    :initarg :aliengo_0
    :type hademo-msg:Func_and_Args
    :initform (cl:make-instance 'hademo-msg:Func_and_Args))
   (aliengo_1
    :reader aliengo_1
    :initarg :aliengo_1
    :type hademo-msg:Func_and_Args
    :initform (cl:make-instance 'hademo-msg:Func_and_Args))
   (aliengo_2
    :reader aliengo_2
    :initarg :aliengo_2
    :type hademo-msg:Func_and_Args
    :initform (cl:make-instance 'hademo-msg:Func_and_Args))
   (quadrotor_0
    :reader quadrotor_0
    :initarg :quadrotor_0
    :type hademo-msg:Func_and_Args
    :initform (cl:make-instance 'hademo-msg:Func_and_Args))
   (quadrotor_1
    :reader quadrotor_1
    :initarg :quadrotor_1
    :type hademo-msg:Func_and_Args
    :initform (cl:make-instance 'hademo-msg:Func_and_Args))
   (quadrotor_2
    :reader quadrotor_2
    :initarg :quadrotor_2
    :type hademo-msg:Func_and_Args
    :initform (cl:make-instance 'hademo-msg:Func_and_Args)))
)

(cl:defclass Action (<Action>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Action>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Action)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hademo-msg:<Action> is deprecated: use hademo-msg:Action instead.")))

(cl:ensure-generic-function 'franka_0-val :lambda-list '(m))
(cl:defmethod franka_0-val ((m <Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:franka_0-val is deprecated.  Use hademo-msg:franka_0 instead.")
  (franka_0 m))

(cl:ensure-generic-function 'franka_1-val :lambda-list '(m))
(cl:defmethod franka_1-val ((m <Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:franka_1-val is deprecated.  Use hademo-msg:franka_1 instead.")
  (franka_1 m))

(cl:ensure-generic-function 'franka_2-val :lambda-list '(m))
(cl:defmethod franka_2-val ((m <Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:franka_2-val is deprecated.  Use hademo-msg:franka_2 instead.")
  (franka_2 m))

(cl:ensure-generic-function 'aliengo_0-val :lambda-list '(m))
(cl:defmethod aliengo_0-val ((m <Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:aliengo_0-val is deprecated.  Use hademo-msg:aliengo_0 instead.")
  (aliengo_0 m))

(cl:ensure-generic-function 'aliengo_1-val :lambda-list '(m))
(cl:defmethod aliengo_1-val ((m <Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:aliengo_1-val is deprecated.  Use hademo-msg:aliengo_1 instead.")
  (aliengo_1 m))

(cl:ensure-generic-function 'aliengo_2-val :lambda-list '(m))
(cl:defmethod aliengo_2-val ((m <Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:aliengo_2-val is deprecated.  Use hademo-msg:aliengo_2 instead.")
  (aliengo_2 m))

(cl:ensure-generic-function 'quadrotor_0-val :lambda-list '(m))
(cl:defmethod quadrotor_0-val ((m <Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:quadrotor_0-val is deprecated.  Use hademo-msg:quadrotor_0 instead.")
  (quadrotor_0 m))

(cl:ensure-generic-function 'quadrotor_1-val :lambda-list '(m))
(cl:defmethod quadrotor_1-val ((m <Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:quadrotor_1-val is deprecated.  Use hademo-msg:quadrotor_1 instead.")
  (quadrotor_1 m))

(cl:ensure-generic-function 'quadrotor_2-val :lambda-list '(m))
(cl:defmethod quadrotor_2-val ((m <Action>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:quadrotor_2-val is deprecated.  Use hademo-msg:quadrotor_2 instead.")
  (quadrotor_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Action>) ostream)
  "Serializes a message object of type '<Action>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'franka_0) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'franka_1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'franka_2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'aliengo_0) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'aliengo_1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'aliengo_2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'quadrotor_0) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'quadrotor_1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'quadrotor_2) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Action>) istream)
  "Deserializes a message object of type '<Action>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'franka_0) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'franka_1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'franka_2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'aliengo_0) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'aliengo_1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'aliengo_2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'quadrotor_0) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'quadrotor_1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'quadrotor_2) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Action>)))
  "Returns string type for a message object of type '<Action>"
  "hademo/Action")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Action)))
  "Returns string type for a message object of type 'Action"
  "hademo/Action")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Action>)))
  "Returns md5sum for a message object of type '<Action>"
  "d976dee5559eb23c2568d9c0e79066c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Action)))
  "Returns md5sum for a message object of type 'Action"
  "d976dee5559eb23c2568d9c0e79066c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Action>)))
  "Returns full string definition for message of type '<Action>"
  (cl:format cl:nil "hademo/Func_and_Args franka_0~%hademo/Func_and_Args franka_1~%hademo/Func_and_Args franka_2~%hademo/Func_and_Args aliengo_0~%hademo/Func_and_Args aliengo_1~%hademo/Func_and_Args aliengo_2~%hademo/Func_and_Args quadrotor_0~%hademo/Func_and_Args quadrotor_1~%hademo/Func_and_Args quadrotor_2~%~%================================================================================~%MSG: hademo/Func_and_Args~%bool has_func~%string func_name~%hademo/Args args~%~%================================================================================~%MSG: hademo/Args~%bool has_args~%string attached_prim_path~%std_msgs/Float64MultiArray waypoint_pos~%std_msgs/Float64MultiArray waypoint_ori~%int32 waypoint_ind~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Action)))
  "Returns full string definition for message of type 'Action"
  (cl:format cl:nil "hademo/Func_and_Args franka_0~%hademo/Func_and_Args franka_1~%hademo/Func_and_Args franka_2~%hademo/Func_and_Args aliengo_0~%hademo/Func_and_Args aliengo_1~%hademo/Func_and_Args aliengo_2~%hademo/Func_and_Args quadrotor_0~%hademo/Func_and_Args quadrotor_1~%hademo/Func_and_Args quadrotor_2~%~%================================================================================~%MSG: hademo/Func_and_Args~%bool has_func~%string func_name~%hademo/Args args~%~%================================================================================~%MSG: hademo/Args~%bool has_args~%string attached_prim_path~%std_msgs/Float64MultiArray waypoint_pos~%std_msgs/Float64MultiArray waypoint_ori~%int32 waypoint_ind~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Action>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'franka_0))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'franka_1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'franka_2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'aliengo_0))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'aliengo_1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'aliengo_2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'quadrotor_0))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'quadrotor_1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'quadrotor_2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Action>))
  "Converts a ROS message object to a list"
  (cl:list 'Action
    (cl:cons ':franka_0 (franka_0 msg))
    (cl:cons ':franka_1 (franka_1 msg))
    (cl:cons ':franka_2 (franka_2 msg))
    (cl:cons ':aliengo_0 (aliengo_0 msg))
    (cl:cons ':aliengo_1 (aliengo_1 msg))
    (cl:cons ':aliengo_2 (aliengo_2 msg))
    (cl:cons ':quadrotor_0 (quadrotor_0 msg))
    (cl:cons ':quadrotor_1 (quadrotor_1 msg))
    (cl:cons ':quadrotor_2 (quadrotor_2 msg))
))
