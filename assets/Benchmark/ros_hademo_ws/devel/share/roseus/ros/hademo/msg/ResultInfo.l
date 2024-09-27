;; Auto-generated. Do not edit!


(when (boundp 'hademo::ResultInfo)
  (if (not (find-package "HADEMO"))
    (make-package "HADEMO"))
  (shadow 'ResultInfo (find-package "HADEMO")))
(unless (find-package "HADEMO::RESULTINFO")
  (make-package "HADEMO::RESULTINFO"))

(in-package "ROS")
;;//! \htmlinclude ResultInfo.msg.html


(defclass hademo::ResultInfo
  :super ros::object
  :slots (_has_result _success _info ))

(defmethod hademo::ResultInfo
  (:init
   (&key
    ((:has_result __has_result) nil)
    ((:success __success) nil)
    ((:info __info) "")
    )
   (send-super :init)
   (setq _has_result __has_result)
   (setq _success __success)
   (setq _info (string __info))
   self)
  (:has_result
   (&optional (__has_result :null))
   (if (not (eq __has_result :null)) (setq _has_result __has_result)) _has_result)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:info
   (&optional __info)
   (if __info (setq _info __info)) _info)
  (:serialization-length
   ()
   (+
    ;; bool _has_result
    1
    ;; bool _success
    1
    ;; string _info
    4 (length _info)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _has_result
       (if _has_result (write-byte -1 s) (write-byte 0 s))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _info
       (write-long (length _info) s) (princ _info s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _has_result
     (setq _has_result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _info
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _info (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get hademo::ResultInfo :md5sum-) "fc965c583533320bba311cfc20577920")
(setf (get hademo::ResultInfo :datatype-) "hademo/ResultInfo")
(setf (get hademo::ResultInfo :definition-)
      "bool has_result
bool success
string info

")



(provide :hademo/ResultInfo "fc965c583533320bba311cfc20577920")


