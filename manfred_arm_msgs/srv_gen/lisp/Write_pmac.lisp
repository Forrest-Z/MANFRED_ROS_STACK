; Auto-generated. Do not edit!


(cl:in-package manfred_arm_msgs-srv)


;//! \htmlinclude Write_pmac-request.msg.html

(cl:defclass <Write_pmac-request> (roslisp-msg-protocol:ros-message)
  ((send
    :reader send
    :initarg :send
    :type cl:string
    :initform ""))
)

(cl:defclass Write_pmac-request (<Write_pmac-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Write_pmac-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Write_pmac-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manfred_arm_msgs-srv:<Write_pmac-request> is deprecated: use manfred_arm_msgs-srv:Write_pmac-request instead.")))

(cl:ensure-generic-function 'send-val :lambda-list '(m))
(cl:defmethod send-val ((m <Write_pmac-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:send-val is deprecated.  Use manfred_arm_msgs-srv:send instead.")
  (send m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Write_pmac-request>) ostream)
  "Serializes a message object of type '<Write_pmac-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'send))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'send))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Write_pmac-request>) istream)
  "Deserializes a message object of type '<Write_pmac-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'send) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'send) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Write_pmac-request>)))
  "Returns string type for a service object of type '<Write_pmac-request>"
  "manfred_arm_msgs/Write_pmacRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Write_pmac-request)))
  "Returns string type for a service object of type 'Write_pmac-request"
  "manfred_arm_msgs/Write_pmacRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Write_pmac-request>)))
  "Returns md5sum for a message object of type '<Write_pmac-request>"
  "90ae29861113f3282de4f6e9e93502ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Write_pmac-request)))
  "Returns md5sum for a message object of type 'Write_pmac-request"
  "90ae29861113f3282de4f6e9e93502ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Write_pmac-request>)))
  "Returns full string definition for message of type '<Write_pmac-request>"
  (cl:format cl:nil "~%~%~%~%string send~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Write_pmac-request)))
  "Returns full string definition for message of type 'Write_pmac-request"
  (cl:format cl:nil "~%~%~%~%string send~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Write_pmac-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'send))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Write_pmac-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Write_pmac-request
    (cl:cons ':send (send msg))
))
;//! \htmlinclude Write_pmac-response.msg.html

(cl:defclass <Write_pmac-response> (roslisp-msg-protocol:ros-message)
  ((answer
    :reader answer
    :initarg :answer
    :type cl:string
    :initform "")
   (error_code
    :reader error_code
    :initarg :error_code
    :type cl:integer
    :initform 0))
)

(cl:defclass Write_pmac-response (<Write_pmac-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Write_pmac-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Write_pmac-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manfred_arm_msgs-srv:<Write_pmac-response> is deprecated: use manfred_arm_msgs-srv:Write_pmac-response instead.")))

(cl:ensure-generic-function 'answer-val :lambda-list '(m))
(cl:defmethod answer-val ((m <Write_pmac-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:answer-val is deprecated.  Use manfred_arm_msgs-srv:answer instead.")
  (answer m))

(cl:ensure-generic-function 'error_code-val :lambda-list '(m))
(cl:defmethod error_code-val ((m <Write_pmac-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:error_code-val is deprecated.  Use manfred_arm_msgs-srv:error_code instead.")
  (error_code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Write_pmac-response>) ostream)
  "Serializes a message object of type '<Write_pmac-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'answer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'answer))
  (cl:let* ((signed (cl:slot-value msg 'error_code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Write_pmac-response>) istream)
  "Deserializes a message object of type '<Write_pmac-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'answer) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'answer) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error_code) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Write_pmac-response>)))
  "Returns string type for a service object of type '<Write_pmac-response>"
  "manfred_arm_msgs/Write_pmacResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Write_pmac-response)))
  "Returns string type for a service object of type 'Write_pmac-response"
  "manfred_arm_msgs/Write_pmacResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Write_pmac-response>)))
  "Returns md5sum for a message object of type '<Write_pmac-response>"
  "90ae29861113f3282de4f6e9e93502ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Write_pmac-response)))
  "Returns md5sum for a message object of type 'Write_pmac-response"
  "90ae29861113f3282de4f6e9e93502ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Write_pmac-response>)))
  "Returns full string definition for message of type '<Write_pmac-response>"
  (cl:format cl:nil "~%string answer~%~%int64 error_code~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Write_pmac-response)))
  "Returns full string definition for message of type 'Write_pmac-response"
  (cl:format cl:nil "~%string answer~%~%int64 error_code~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Write_pmac-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'answer))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Write_pmac-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Write_pmac-response
    (cl:cons ':answer (answer msg))
    (cl:cons ':error_code (error_code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Write_pmac)))
  'Write_pmac-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Write_pmac)))
  'Write_pmac-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Write_pmac)))
  "Returns string type for a service object of type '<Write_pmac>"
  "manfred_arm_msgs/Write_pmac")