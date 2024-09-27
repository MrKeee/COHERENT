; Auto-generated. Do not edit!


(cl:in-package hademo-msg)


;//! \htmlinclude ResultInfo.msg.html

(cl:defclass <ResultInfo> (roslisp-msg-protocol:ros-message)
  ((has_result
    :reader has_result
    :initarg :has_result
    :type cl:boolean
    :initform cl:nil)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (info
    :reader info
    :initarg :info
    :type cl:string
    :initform ""))
)

(cl:defclass ResultInfo (<ResultInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResultInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResultInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hademo-msg:<ResultInfo> is deprecated: use hademo-msg:ResultInfo instead.")))

(cl:ensure-generic-function 'has_result-val :lambda-list '(m))
(cl:defmethod has_result-val ((m <ResultInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:has_result-val is deprecated.  Use hademo-msg:has_result instead.")
  (has_result m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ResultInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:success-val is deprecated.  Use hademo-msg:success instead.")
  (success m))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <ResultInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hademo-msg:info-val is deprecated.  Use hademo-msg:info instead.")
  (info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResultInfo>) ostream)
  "Serializes a message object of type '<ResultInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'has_result) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'info))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResultInfo>) istream)
  "Deserializes a message object of type '<ResultInfo>"
    (cl:setf (cl:slot-value msg 'has_result) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'info) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'info) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResultInfo>)))
  "Returns string type for a message object of type '<ResultInfo>"
  "hademo/ResultInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResultInfo)))
  "Returns string type for a message object of type 'ResultInfo"
  "hademo/ResultInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResultInfo>)))
  "Returns md5sum for a message object of type '<ResultInfo>"
  "fc965c583533320bba311cfc20577920")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResultInfo)))
  "Returns md5sum for a message object of type 'ResultInfo"
  "fc965c583533320bba311cfc20577920")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResultInfo>)))
  "Returns full string definition for message of type '<ResultInfo>"
  (cl:format cl:nil "bool has_result~%bool success~%string info~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResultInfo)))
  "Returns full string definition for message of type 'ResultInfo"
  (cl:format cl:nil "bool has_result~%bool success~%string info~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResultInfo>))
  (cl:+ 0
     1
     1
     4 (cl:length (cl:slot-value msg 'info))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResultInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'ResultInfo
    (cl:cons ':has_result (has_result msg))
    (cl:cons ':success (success msg))
    (cl:cons ':info (info msg))
))
