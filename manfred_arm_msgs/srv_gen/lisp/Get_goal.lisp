; Auto-generated. Do not edit!


(cl:in-package manfred_arm_msgs-srv)


;//! \htmlinclude Get_goal-request.msg.html

(cl:defclass <Get_goal-request> (roslisp-msg-protocol:ros-message)
  ((hola
    :reader hola
    :initarg :hola
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Get_goal-request (<Get_goal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Get_goal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Get_goal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manfred_arm_msgs-srv:<Get_goal-request> is deprecated: use manfred_arm_msgs-srv:Get_goal-request instead.")))

(cl:ensure-generic-function 'hola-val :lambda-list '(m))
(cl:defmethod hola-val ((m <Get_goal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:hola-val is deprecated.  Use manfred_arm_msgs-srv:hola instead.")
  (hola m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Get_goal-request>) ostream)
  "Serializes a message object of type '<Get_goal-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'hola) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Get_goal-request>) istream)
  "Deserializes a message object of type '<Get_goal-request>"
    (cl:setf (cl:slot-value msg 'hola) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Get_goal-request>)))
  "Returns string type for a service object of type '<Get_goal-request>"
  "manfred_arm_msgs/Get_goalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Get_goal-request)))
  "Returns string type for a service object of type 'Get_goal-request"
  "manfred_arm_msgs/Get_goalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Get_goal-request>)))
  "Returns md5sum for a message object of type '<Get_goal-request>"
  "a426a8387670350bdffe669155a91dd4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Get_goal-request)))
  "Returns md5sum for a message object of type 'Get_goal-request"
  "a426a8387670350bdffe669155a91dd4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Get_goal-request>)))
  "Returns full string definition for message of type '<Get_goal-request>"
  (cl:format cl:nil "~%~%bool hola~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Get_goal-request)))
  "Returns full string definition for message of type 'Get_goal-request"
  (cl:format cl:nil "~%~%bool hola~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Get_goal-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Get_goal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Get_goal-request
    (cl:cons ':hola (hola msg))
))
;//! \htmlinclude Get_goal-response.msg.html

(cl:defclass <Get_goal-response> (roslisp-msg-protocol:ros-message)
  ((posic
    :reader posic
    :initarg :posic
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (rpy
    :reader rpy
    :initarg :rpy
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (ori
    :reader ori
    :initarg :ori
    :type cl:boolean
    :initform cl:nil)
   (ultimo
    :reader ultimo
    :initarg :ultimo
    :type cl:boolean
    :initform cl:nil)
   (error_code
    :reader error_code
    :initarg :error_code
    :type cl:integer
    :initform 0))
)

(cl:defclass Get_goal-response (<Get_goal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Get_goal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Get_goal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manfred_arm_msgs-srv:<Get_goal-response> is deprecated: use manfred_arm_msgs-srv:Get_goal-response instead.")))

(cl:ensure-generic-function 'posic-val :lambda-list '(m))
(cl:defmethod posic-val ((m <Get_goal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:posic-val is deprecated.  Use manfred_arm_msgs-srv:posic instead.")
  (posic m))

(cl:ensure-generic-function 'rpy-val :lambda-list '(m))
(cl:defmethod rpy-val ((m <Get_goal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:rpy-val is deprecated.  Use manfred_arm_msgs-srv:rpy instead.")
  (rpy m))

(cl:ensure-generic-function 'ori-val :lambda-list '(m))
(cl:defmethod ori-val ((m <Get_goal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:ori-val is deprecated.  Use manfred_arm_msgs-srv:ori instead.")
  (ori m))

(cl:ensure-generic-function 'ultimo-val :lambda-list '(m))
(cl:defmethod ultimo-val ((m <Get_goal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:ultimo-val is deprecated.  Use manfred_arm_msgs-srv:ultimo instead.")
  (ultimo m))

(cl:ensure-generic-function 'error_code-val :lambda-list '(m))
(cl:defmethod error_code-val ((m <Get_goal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:error_code-val is deprecated.  Use manfred_arm_msgs-srv:error_code instead.")
  (error_code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Get_goal-response>) ostream)
  "Serializes a message object of type '<Get_goal-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'posic) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rpy) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ori) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ultimo) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'error_code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Get_goal-response>) istream)
  "Deserializes a message object of type '<Get_goal-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'posic) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rpy) istream)
    (cl:setf (cl:slot-value msg 'ori) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ultimo) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error_code) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Get_goal-response>)))
  "Returns string type for a service object of type '<Get_goal-response>"
  "manfred_arm_msgs/Get_goalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Get_goal-response)))
  "Returns string type for a service object of type 'Get_goal-response"
  "manfred_arm_msgs/Get_goalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Get_goal-response>)))
  "Returns md5sum for a message object of type '<Get_goal-response>"
  "a426a8387670350bdffe669155a91dd4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Get_goal-response)))
  "Returns md5sum for a message object of type 'Get_goal-response"
  "a426a8387670350bdffe669155a91dd4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Get_goal-response>)))
  "Returns full string definition for message of type '<Get_goal-response>"
  (cl:format cl:nil "~%geometry_msgs/Point32 posic~%geometry_msgs/Vector3 rpy~%~%~%~%bool ori~%~%~%bool ultimo~%~%int32 error_code~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Get_goal-response)))
  "Returns full string definition for message of type 'Get_goal-response"
  (cl:format cl:nil "~%geometry_msgs/Point32 posic~%geometry_msgs/Vector3 rpy~%~%~%~%bool ori~%~%~%bool ultimo~%~%int32 error_code~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Get_goal-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'posic))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rpy))
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Get_goal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Get_goal-response
    (cl:cons ':posic (posic msg))
    (cl:cons ':rpy (rpy msg))
    (cl:cons ':ori (ori msg))
    (cl:cons ':ultimo (ultimo msg))
    (cl:cons ':error_code (error_code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Get_goal)))
  'Get_goal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Get_goal)))
  'Get_goal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Get_goal)))
  "Returns string type for a service object of type '<Get_goal>"
  "manfred_arm_msgs/Get_goal")