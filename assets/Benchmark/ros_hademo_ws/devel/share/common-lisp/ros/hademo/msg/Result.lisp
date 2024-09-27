; Auto-generated. Do not edit!


(cl:in-package hademo-msg)


;//! \htmlinclude Result.msg.html

(cl:defclass <Result> (roslisp-msg-protocol:ros-message)
  ((franka_0
    :reader franka_0
    :initarg :franka_0
    :type hademo-msg:ResultInfo
    :initform (cl:make-instance 'hademo-msg:ResultInfo))
   (franka_1
    :reader franka_1
    :initarg :franka_1
    :type hademo-msg:ResultInfo
    :initform (cl:make-instance 'hademo-msg:ResultInfo))
   (franka_2
    :reader franka_2
    :initarg :franka_2
    :type hademo-msg:ResultInfo
    :initform (cl:make-instance 'hademo-msg:ResultInfo))
   (aliengo_0
    :reader aliengo_0
    :initarg :aliengo_0
    :type hademo-msg:ResultInfo
    :initform (cl:make-instance 'hademo-msg:ResultInfo))
   (aliengo_1
    :reader aliengo_1
    :initarg :aliengo_1
    :type hademo-msg:ResultInfo
    :initform (cl:make-instance 'hademo-msg:ResultInfo))
   (aliengo_2
    :reader aliengo_2
    :initarg :aliengo_2
    :type hademo-msg:ResultInfo
    :initform (cl:make-instance 'hademo-msg:ResultInfo))
   (quadrotor_0
    :reader quadrotor_0
    :initarg :quadrotor_0
    :type hademo-msg:ResultInfo
    :initform (cl:make-instance 'hademo-msg:ResultInfo))
   (quadrotor_1
    :reader quadrotor_1
    :initarg :quadrotor_1
    :type hademo-msg:ResultInfo
    :initform (cl:make-instance 'hademo-msg:ResultInfo))
   (quadrotor_2
    :reader quadrotor_2
    :initarg :quadrotor_2
    :type hademo-msg:ResultInfo
    :initform (cl:make-instance 'hademo-msg:ResultInfo)))
)

(cl:defclass Result (<Result>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Result>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Result)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hademo-msg:<Result> is deprecated: use hademo-msg:Result instead.")))

(cl:ensure-generic-function 'franka_0-val :lambda-list '(m))
(cl:defmethod franka_0-val ((m <Result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:franka_0-val is deprecated.  Use hademo-msg:franka_0 instead.")
  (franka_0 m))

(cl:ensure-generic-function 'franka_1-val :lambda-list '(m))
(cl:defmethod franka_1-val ((m <Result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:franka_1-val is deprecated.  Use hademo-msg:franka_1 instead.")
  (franka_1 m))

(cl:ensure-generic-function 'franka_2-val :lambda-list '(m))
(cl:defmethod franka_2-val ((m <Result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:franka_2-val is deprecated.  Use hademo-msg:franka_2 instead.")
  (franka_2 m))

(cl:ensure-generic-function 'aliengo_0-val :lambda-list '(m))
(cl:defmethod aliengo_0-val ((m <Result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:aliengo_0-val is deprecated.  Use hademo-msg:aliengo_0 instead.")
  (aliengo_0 m))

(cl:ensure-generic-function 'aliengo_1-val :lambda-list '(m))
(cl:defmethod aliengo_1-val ((m <Result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:aliengo_1-val is deprecated.  Use hademo-msg:aliengo_1 instead.")
  (aliengo_1 m))

(cl:ensure-generic-function 'aliengo_2-val :lambda-list '(m))
(cl:defmethod aliengo_2-val ((m <Result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:aliengo_2-val is deprecated.  Use hademo-msg:aliengo_2 instead.")
  (aliengo_2 m))

(cl:ensure-generic-function 'quadrotor_0-val :lambda-list '(m))
(cl:defmethod quadrotor_0-val ((m <Result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:quadrotor_0-val is deprecated.  Use hademo-msg:quadrotor_0 instead.")
  (quadrotor_0 m))

(cl:ensure-generic-function 'quadrotor_1-val :lambda-list '(m))
(cl:defmethod quadrotor_1-val ((m <Result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:quadrotor_1-val is deprecated.  Use hademo-msg:quadrotor_1 instead.")
  (quadrotor_1 m))

(cl:ensure-generic-function 'quadrotor_2-val :lambda-list '(m))
(cl:defmethod quadrotor_2-val ((m <Result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:quadrotor_2-val is deprecated.  Use hademo-msg:quadrotor_2 instead.")
  (quadrotor_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Result>) ostream)
  "Serializes a message object of type '<Result>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Result>) istream)
  "Deserializes a message object of type '<Result>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Result>)))
  "Returns string type for a message object of type '<Result>"
  "hademo/Result")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Result)))
  "Returns string type for a message object of type 'Result"
  "hademo/Result")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Result>)))
  "Returns md5sum for a message object of type '<Result>"
  "6a97bcb6cbe98951369cc3755518f34c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Result)))
  "Returns md5sum for a message object of type 'Result"
  "6a97bcb6cbe98951369cc3755518f34c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Result>)))
  "Returns full string definition for message of type '<Result>"
  (cl:format cl:nil "hademo/ResultInfo franka_0~%hademo/ResultInfo franka_1~%hademo/ResultInfo franka_2~%hademo/ResultInfo aliengo_0~%hademo/ResultInfo aliengo_1~%hademo/ResultInfo aliengo_2~%hademo/ResultInfo quadrotor_0~%hademo/ResultInfo quadrotor_1~%hademo/ResultInfo quadrotor_2~%~%================================================================================~%MSG: hademo/ResultInfo~%bool has_result~%bool success~%string info~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Result)))
  "Returns full string definition for message of type 'Result"
  (cl:format cl:nil "hademo/ResultInfo franka_0~%hademo/ResultInfo franka_1~%hademo/ResultInfo franka_2~%hademo/ResultInfo aliengo_0~%hademo/ResultInfo aliengo_1~%hademo/ResultInfo aliengo_2~%hademo/ResultInfo quadrotor_0~%hademo/ResultInfo quadrotor_1~%hademo/ResultInfo quadrotor_2~%~%================================================================================~%MSG: hademo/ResultInfo~%bool has_result~%bool success~%string info~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Result>))
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Result>))
  "Converts a ROS message object to a list"
  (cl:list 'Result
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
