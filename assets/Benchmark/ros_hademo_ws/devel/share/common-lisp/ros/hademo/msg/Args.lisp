; Auto-generated. Do not edit!


(cl:in-package hademo-msg)


;//! \htmlinclude Args.msg.html

(cl:defclass <Args> (roslisp-msg-protocol:ros-message)
  ((has_args
    :reader has_args
    :initarg :has_args
    :type cl:boolean
    :initform cl:nil)
   (attached_prim_path
    :reader attached_prim_path
    :initarg :attached_prim_path
    :type cl:string
    :initform "")
   (waypoint_pos
    :reader waypoint_pos
    :initarg :waypoint_pos
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (waypoint_ori
    :reader waypoint_ori
    :initarg :waypoint_ori
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray))
   (waypoint_ind
    :reader waypoint_ind
    :initarg :waypoint_ind
    :type cl:integer
    :initform 0))
)

(cl:defclass Args (<Args>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Args>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Args)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hademo-msg:<Args> is deprecated: use hademo-msg:Args instead.")))

(cl:ensure-generic-function 'has_args-val :lambda-list '(m))
(cl:defmethod has_args-val ((m <Args>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:has_args-val is deprecated.  Use hademo-msg:has_args instead.")
  (has_args m))

(cl:ensure-generic-function 'attached_prim_path-val :lambda-list '(m))
(cl:defmethod attached_prim_path-val ((m <Args>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:attached_prim_path-val is deprecated.  Use hademo-msg:attached_prim_path instead.")
  (attached_prim_path m))

(cl:ensure-generic-function 'waypoint_pos-val :lambda-list '(m))
(cl:defmethod waypoint_pos-val ((m <Args>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:waypoint_pos-val is deprecated.  Use hademo-msg:waypoint_pos instead.")
  (waypoint_pos m))

(cl:ensure-generic-function 'waypoint_ori-val :lambda-list '(m))
(cl:defmethod waypoint_ori-val ((m <Args>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:waypoint_ori-val is deprecated.  Use hademo-msg:waypoint_ori instead.")
  (waypoint_ori m))

(cl:ensure-generic-function 'waypoint_ind-val :lambda-list '(m))
(cl:defmethod waypoint_ind-val ((m <Args>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:waypoint_ind-val is deprecated.  Use hademo-msg:waypoint_ind instead.")
  (waypoint_ind m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Args>) ostream)
  "Serializes a message object of type '<Args>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'has_args) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'attached_prim_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'attached_prim_path))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'waypoint_pos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'waypoint_ori) ostream)
  (cl:let* ((signed (cl:slot-value msg 'waypoint_ind)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Args>) istream)
  "Deserializes a message object of type '<Args>"
    (cl:setf (cl:slot-value msg 'has_args) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'attached_prim_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'attached_prim_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'waypoint_pos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'waypoint_ori) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'waypoint_ind) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Args>)))
  "Returns string type for a message object of type '<Args>"
  "hademo/Args")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Args)))
  "Returns string type for a message object of type 'Args"
  "hademo/Args")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Args>)))
  "Returns md5sum for a message object of type '<Args>"
  "482e2fefd3fc2e17f27a0aa075e67ef3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Args)))
  "Returns md5sum for a message object of type 'Args"
  "482e2fefd3fc2e17f27a0aa075e67ef3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Args>)))
  "Returns full string definition for message of type '<Args>"
  (cl:format cl:nil "bool has_args~%string attached_prim_path~%std_msgs/Float64MultiArray waypoint_pos~%std_msgs/Float64MultiArray waypoint_ori~%int32 waypoint_ind~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Args)))
  "Returns full string definition for message of type 'Args"
  (cl:format cl:nil "bool has_args~%string attached_prim_path~%std_msgs/Float64MultiArray waypoint_pos~%std_msgs/Float64MultiArray waypoint_ori~%int32 waypoint_ind~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Args>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'attached_prim_path))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'waypoint_pos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'waypoint_ori))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Args>))
  "Converts a ROS message object to a list"
  (cl:list 'Args
    (cl:cons ':has_args (has_args msg))
    (cl:cons ':attached_prim_path (attached_prim_path msg))
    (cl:cons ':waypoint_pos (waypoint_pos msg))
    (cl:cons ':waypoint_ori (waypoint_ori msg))
    (cl:cons ':waypoint_ind (waypoint_ind msg))
))
