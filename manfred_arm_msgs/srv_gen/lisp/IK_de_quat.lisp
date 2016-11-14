; Auto-generated. Do not edit!


(cl:in-package manfred_arm_msgs-srv)


;//! \htmlinclude IK_de_quat-request.msg.html

(cl:defclass <IK_de_quat-request> (roslisp-msg-protocol:ros-message)
  ((joint_state
    :reader joint_state
    :initarg :joint_state
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState))
   (goal_posic
    :reader goal_posic
    :initarg :goal_posic
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (goal_quat
    :reader goal_quat
    :initarg :goal_quat
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion)))
)

(cl:defclass IK_de_quat-request (<IK_de_quat-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IK_de_quat-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IK_de_quat-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manfred_arm_msgs-srv:<IK_de_quat-request> is deprecated: use manfred_arm_msgs-srv:IK_de_quat-request instead.")))

(cl:ensure-generic-function 'joint_state-val :lambda-list '(m))
(cl:defmethod joint_state-val ((m <IK_de_quat-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:joint_state-val is deprecated.  Use manfred_arm_msgs-srv:joint_state instead.")
  (joint_state m))

(cl:ensure-generic-function 'goal_posic-val :lambda-list '(m))
(cl:defmethod goal_posic-val ((m <IK_de_quat-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:goal_posic-val is deprecated.  Use manfred_arm_msgs-srv:goal_posic instead.")
  (goal_posic m))

(cl:ensure-generic-function 'goal_quat-val :lambda-list '(m))
(cl:defmethod goal_quat-val ((m <IK_de_quat-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:goal_quat-val is deprecated.  Use manfred_arm_msgs-srv:goal_quat instead.")
  (goal_quat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IK_de_quat-request>) ostream)
  "Serializes a message object of type '<IK_de_quat-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_posic) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_quat) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IK_de_quat-request>) istream)
  "Deserializes a message object of type '<IK_de_quat-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_posic) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_quat) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IK_de_quat-request>)))
  "Returns string type for a service object of type '<IK_de_quat-request>"
  "manfred_arm_msgs/IK_de_quatRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IK_de_quat-request)))
  "Returns string type for a service object of type 'IK_de_quat-request"
  "manfred_arm_msgs/IK_de_quatRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IK_de_quat-request>)))
  "Returns md5sum for a message object of type '<IK_de_quat-request>"
  "c9e231805aca89efded59e5284014ced")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IK_de_quat-request)))
  "Returns md5sum for a message object of type 'IK_de_quat-request"
  "c9e231805aca89efded59e5284014ced")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IK_de_quat-request>)))
  "Returns full string definition for message of type '<IK_de_quat-request>"
  (cl:format cl:nil "~%sensor_msgs/JointState joint_state~%~%~%geometry_msgs/Point32 goal_posic~%geometry_msgs/Quaternion goal_quat~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IK_de_quat-request)))
  "Returns full string definition for message of type 'IK_de_quat-request"
  (cl:format cl:nil "~%sensor_msgs/JointState joint_state~%~%~%geometry_msgs/Point32 goal_posic~%geometry_msgs/Quaternion goal_quat~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IK_de_quat-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_posic))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_quat))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IK_de_quat-request>))
  "Converts a ROS message object to a list"
  (cl:list 'IK_de_quat-request
    (cl:cons ':joint_state (joint_state msg))
    (cl:cons ':goal_posic (goal_posic msg))
    (cl:cons ':goal_quat (goal_quat msg))
))
;//! \htmlinclude IK_de_quat-response.msg.html

(cl:defclass <IK_de_quat-response> (roslisp-msg-protocol:ros-message)
  ((joint_state
    :reader joint_state
    :initarg :joint_state
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState))
   (error_code
    :reader error_code
    :initarg :error_code
    :type cl:integer
    :initform 0))
)

(cl:defclass IK_de_quat-response (<IK_de_quat-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IK_de_quat-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IK_de_quat-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manfred_arm_msgs-srv:<IK_de_quat-response> is deprecated: use manfred_arm_msgs-srv:IK_de_quat-response instead.")))

(cl:ensure-generic-function 'joint_state-val :lambda-list '(m))
(cl:defmethod joint_state-val ((m <IK_de_quat-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:joint_state-val is deprecated.  Use manfred_arm_msgs-srv:joint_state instead.")
  (joint_state m))

(cl:ensure-generic-function 'error_code-val :lambda-list '(m))
(cl:defmethod error_code-val ((m <IK_de_quat-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-srv:error_code-val is deprecated.  Use manfred_arm_msgs-srv:error_code instead.")
  (error_code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IK_de_quat-response>) ostream)
  "Serializes a message object of type '<IK_de_quat-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_state) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IK_de_quat-response>) istream)
  "Deserializes a message object of type '<IK_de_quat-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_state) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IK_de_quat-response>)))
  "Returns string type for a service object of type '<IK_de_quat-response>"
  "manfred_arm_msgs/IK_de_quatResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IK_de_quat-response)))
  "Returns string type for a service object of type 'IK_de_quat-response"
  "manfred_arm_msgs/IK_de_quatResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IK_de_quat-response>)))
  "Returns md5sum for a message object of type '<IK_de_quat-response>"
  "c9e231805aca89efded59e5284014ced")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IK_de_quat-response)))
  "Returns md5sum for a message object of type 'IK_de_quat-response"
  "c9e231805aca89efded59e5284014ced")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IK_de_quat-response>)))
  "Returns full string definition for message of type '<IK_de_quat-response>"
  (cl:format cl:nil "~%~%sensor_msgs/JointState joint_state~%~%int64 error_code~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IK_de_quat-response)))
  "Returns full string definition for message of type 'IK_de_quat-response"
  (cl:format cl:nil "~%~%sensor_msgs/JointState joint_state~%~%int64 error_code~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IK_de_quat-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_state))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IK_de_quat-response>))
  "Converts a ROS message object to a list"
  (cl:list 'IK_de_quat-response
    (cl:cons ':joint_state (joint_state msg))
    (cl:cons ':error_code (error_code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'IK_de_quat)))
  'IK_de_quat-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'IK_de_quat)))
  'IK_de_quat-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IK_de_quat)))
  "Returns string type for a service object of type '<IK_de_quat>"
  "manfred_arm_msgs/IK_de_quat")