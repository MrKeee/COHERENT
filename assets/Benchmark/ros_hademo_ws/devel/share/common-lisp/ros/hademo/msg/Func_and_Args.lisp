; Auto-generated. Do not edit!


(cl:in-package hademo-msg)


;//! \htmlinclude Func_and_Args.msg.html

(cl:defclass <Func_and_Args> (roslisp-msg-protocol:ros-message)
  ((has_func
    :reader has_func
    :initarg :has_func
    :type cl:boolean
    :initform cl:nil)
   (func_name
    :reader func_name
    :initarg :func_name
    :type cl:string
    :initform "")
   (args
    :reader args
    :initarg :args
    :type hademo-msg:Args
    :initform (cl:make-instance 'hademo-msg:Args)))
)

(cl:defclass Func_and_Args (<Func_and_Args>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Func_and_Args>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Func_and_Args)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hademo-msg:<Func_and_Args> is deprecated: use hademo-msg:Func_and_Args instead.")))

(cl:ensure-generic-function 'has_func-val :lambda-list '(m))
(cl:defmethod has_func-val ((m <Func_and_Args>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:has_func-val is deprecated.  Use hademo-msg:has_func instead.")
  (has_func m))

(cl:ensure-generic-function 'func_name-val :lambda-list '(m))
(cl:defmethod func_name-val ((m <Func_and_Args>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:func_name-val is deprecated.  Use hademo-msg:func_name instead.")
  (func_name m))

(cl:ensure-generic-function 'args-val :lambda-list '(m))
(cl:defmethod args-val ((m <Func_and_Args>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:args-val is deprecated.  Use hademo-msg:args instead.")
  (args m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Func_and_Args>) ostream)
  "Serializes a message object of type '<Func_and_Args>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'has_func) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'func_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'func_name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'args) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Func_and_Args>) istream)
  "Deserializes a message object of type '<Func_and_Args>"
    (cl:setf (cl:slot-value msg 'has_func) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'func_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'func_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'args) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Func_and_Args>)))
  "Returns string type for a message object of type '<Func_and_Args>"
  "hademo/Func_and_Args")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Func_and_Args)))
  "Returns string type for a message object of type 'Func_and_Args"
  "hademo/Func_and_Args")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Func_and_Args>)))
  "Returns md5sum for a message object of type '<Func_and_Args>"
  "2cf85b88f235eeb7b23bd9784805fafa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Func_and_Args)))
  "Returns md5sum for a message object of type 'Func_and_Args"
  "2cf85b88f235eeb7b23bd9784805fafa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Func_and_Args>)))
  "Returns full string definition for message of type '<Func_and_Args>"
  (cl:format cl:nil "bool has_func~%string func_name~%hademo/Args args~%~%================================================================================~%MSG: hademo/Args~%bool has_args~%string attached_prim_path~%std_msgs/Float64MultiArray waypoint_pos~%std_msgs/Float64MultiArray waypoint_ori~%int32 waypoint_ind~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Func_and_Args)))
  "Returns full string definition for message of type 'Func_and_Args"
  (cl:format cl:nil "bool has_func~%string func_name~%hademo/Args args~%~%================================================================================~%MSG: hademo/Args~%bool has_args~%string attached_prim_path~%std_msgs/Float64MultiArray waypoint_pos~%std_msgs/Float64MultiArray waypoint_ori~%int32 waypoint_ind~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Func_and_Args>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'func_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'args))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Func_and_Args>))
  "Converts a ROS message object to a list"
  (cl:list 'Func_and_Args
    (cl:cons ':has_func (has_func msg))
    (cl:cons ':func_name (func_name msg))
    (cl:cons ':args (args msg))
))
