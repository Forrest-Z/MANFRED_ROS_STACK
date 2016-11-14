; Auto-generated. Do not edit!


(cl:in-package manfred_arm_msgs-msg)


;//! \htmlinclude pmac_info.msg.html

(cl:defclass <pmac_info> (roslisp-msg-protocol:ros-message)
  ((joint_state
    :reader joint_state
    :initarg :joint_state
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState)))
)

(cl:defclass pmac_info (<pmac_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pmac_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pmac_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manfred_arm_msgs-msg:<pmac_info> is deprecated: use manfred_arm_msgs-msg:pmac_info instead.")))

(cl:ensure-generic-function 'joint_state-val :lambda-list '(m))
(cl:defmethod joint_state-val ((m <pmac_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:joint_state-val is deprecated.  Use manfred_arm_msgs-msg:joint_state instead.")
  (joint_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pmac_info>) ostream)
  "Serializes a message object of type '<pmac_info>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pmac_info>) istream)
  "Deserializes a message object of type '<pmac_info>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pmac_info>)))
  "Returns string type for a message object of type '<pmac_info>"
  "manfred_arm_msgs/pmac_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pmac_info)))
  "Returns string type for a message object of type 'pmac_info"
  "manfred_arm_msgs/pmac_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pmac_info>)))
  "Returns md5sum for a message object of type '<pmac_info>"
  "9ca061465ef0ed08771ed240c43789f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pmac_info)))
  "Returns md5sum for a message object of type 'pmac_info"
  "9ca061465ef0ed08771ed240c43789f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pmac_info>)))
  "Returns full string definition for message of type '<pmac_info>"
  (cl:format cl:nil "# Posicion y velocidad de los motores~%sensor_msgs/JointState joint_state~%~%# error de posicion que da la pmac calculado como: posicion_comandada - posicion_alcanzada~%# como no lo encuentro entre los registros de la pmac, no se pondrá~%# sensor_msgs/JointState following_error~%~%#int64 error_code~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pmac_info)))
  "Returns full string definition for message of type 'pmac_info"
  (cl:format cl:nil "# Posicion y velocidad de los motores~%sensor_msgs/JointState joint_state~%~%# error de posicion que da la pmac calculado como: posicion_comandada - posicion_alcanzada~%# como no lo encuentro entre los registros de la pmac, no se pondrá~%# sensor_msgs/JointState following_error~%~%#int64 error_code~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pmac_info>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pmac_info>))
  "Converts a ROS message object to a list"
  (cl:list 'pmac_info
    (cl:cons ':joint_state (joint_state msg))
))
