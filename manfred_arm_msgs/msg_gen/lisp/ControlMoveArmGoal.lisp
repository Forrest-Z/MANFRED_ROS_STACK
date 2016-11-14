; Auto-generated. Do not edit!


(cl:in-package manfred_arm_msgs-msg)


;//! \htmlinclude ControlMoveArmGoal.msg.html

(cl:defclass <ControlMoveArmGoal> (roslisp-msg-protocol:ros-message)
  ((max_frequency
    :reader max_frequency
    :initarg :max_frequency
    :type cl:float
    :initform 0.0)
   (time_offset
    :reader time_offset
    :initarg :time_offset
    :type cl:real
    :initform 0)
   (posic
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
   (joint_state
    :reader joint_state
    :initarg :joint_state
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState)))
)

(cl:defclass ControlMoveArmGoal (<ControlMoveArmGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlMoveArmGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlMoveArmGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manfred_arm_msgs-msg:<ControlMoveArmGoal> is deprecated: use manfred_arm_msgs-msg:ControlMoveArmGoal instead.")))

(cl:ensure-generic-function 'max_frequency-val :lambda-list '(m))
(cl:defmethod max_frequency-val ((m <ControlMoveArmGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:max_frequency-val is deprecated.  Use manfred_arm_msgs-msg:max_frequency instead.")
  (max_frequency m))

(cl:ensure-generic-function 'time_offset-val :lambda-list '(m))
(cl:defmethod time_offset-val ((m <ControlMoveArmGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:time_offset-val is deprecated.  Use manfred_arm_msgs-msg:time_offset instead.")
  (time_offset m))

(cl:ensure-generic-function 'posic-val :lambda-list '(m))
(cl:defmethod posic-val ((m <ControlMoveArmGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:posic-val is deprecated.  Use manfred_arm_msgs-msg:posic instead.")
  (posic m))

(cl:ensure-generic-function 'rpy-val :lambda-list '(m))
(cl:defmethod rpy-val ((m <ControlMoveArmGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:rpy-val is deprecated.  Use manfred_arm_msgs-msg:rpy instead.")
  (rpy m))

(cl:ensure-generic-function 'ori-val :lambda-list '(m))
(cl:defmethod ori-val ((m <ControlMoveArmGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:ori-val is deprecated.  Use manfred_arm_msgs-msg:ori instead.")
  (ori m))

(cl:ensure-generic-function 'ultimo-val :lambda-list '(m))
(cl:defmethod ultimo-val ((m <ControlMoveArmGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:ultimo-val is deprecated.  Use manfred_arm_msgs-msg:ultimo instead.")
  (ultimo m))

(cl:ensure-generic-function 'joint_state-val :lambda-list '(m))
(cl:defmethod joint_state-val ((m <ControlMoveArmGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:joint_state-val is deprecated.  Use manfred_arm_msgs-msg:joint_state instead.")
  (joint_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlMoveArmGoal>) ostream)
  "Serializes a message object of type '<ControlMoveArmGoal>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time_offset)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time_offset) (cl:floor (cl:slot-value msg 'time_offset)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'posic) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rpy) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ori) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ultimo) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlMoveArmGoal>) istream)
  "Deserializes a message object of type '<ControlMoveArmGoal>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_frequency) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_offset) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'posic) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rpy) istream)
    (cl:setf (cl:slot-value msg 'ori) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ultimo) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlMoveArmGoal>)))
  "Returns string type for a message object of type '<ControlMoveArmGoal>"
  "manfred_arm_msgs/ControlMoveArmGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlMoveArmGoal)))
  "Returns string type for a message object of type 'ControlMoveArmGoal"
  "manfred_arm_msgs/ControlMoveArmGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlMoveArmGoal>)))
  "Returns md5sum for a message object of type '<ControlMoveArmGoal>"
  "5930a1d2d18f3be4631561bb538d0412")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlMoveArmGoal)))
  "Returns md5sum for a message object of type 'ControlMoveArmGoal"
  "5930a1d2d18f3be4631561bb538d0412")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlMoveArmGoal>)))
  "Returns full string definition for message of type '<ControlMoveArmGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%#Header header~%float32 max_frequency~%duration time_offset~%geometry_msgs/Point32 posic~%geometry_msgs/Vector3 rpy~%#Permite indicar si hay que tener en cuanta la orientación del efector final~%bool ori~%#Marca la última posición de una trayectoria~%bool ultimo~%# ¿Hara falta la velocidad? Al menos hace falta para la funcion de simulacion del robot, tanto vel como position~%sensor_msgs/JointState joint_state~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlMoveArmGoal)))
  "Returns full string definition for message of type 'ControlMoveArmGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%#Header header~%float32 max_frequency~%duration time_offset~%geometry_msgs/Point32 posic~%geometry_msgs/Vector3 rpy~%#Permite indicar si hay que tener en cuanta la orientación del efector final~%bool ori~%#Marca la última posición de una trayectoria~%bool ultimo~%# ¿Hara falta la velocidad? Al menos hace falta para la funcion de simulacion del robot, tanto vel como position~%sensor_msgs/JointState joint_state~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlMoveArmGoal>))
  (cl:+ 0
     4
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'posic))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rpy))
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlMoveArmGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlMoveArmGoal
    (cl:cons ':max_frequency (max_frequency msg))
    (cl:cons ':time_offset (time_offset msg))
    (cl:cons ':posic (posic msg))
    (cl:cons ':rpy (rpy msg))
    (cl:cons ':ori (ori msg))
    (cl:cons ':ultimo (ultimo msg))
    (cl:cons ':joint_state (joint_state msg))
))
