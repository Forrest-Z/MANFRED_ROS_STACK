; Auto-generated. Do not edit!


(cl:in-package manfred_arm_msgs-msg)


;//! \htmlinclude ControlMoveArmFeedback.msg.html

(cl:defclass <ControlMoveArmFeedback> (roslisp-msg-protocol:ros-message)
  ((feedbackStatus
    :reader feedbackStatus
    :initarg :feedbackStatus
    :type actionlib_msgs-msg:GoalStatus
    :initform (cl:make-instance 'actionlib_msgs-msg:GoalStatus))
   (posic_goal
    :reader posic_goal
    :initarg :posic_goal
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (rpy_goal
    :reader rpy_goal
    :initarg :rpy_goal
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (posic_actual
    :reader posic_actual
    :initarg :posic_actual
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (rpy_actual
    :reader rpy_actual
    :initarg :rpy_actual
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

(cl:defclass ControlMoveArmFeedback (<ControlMoveArmFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlMoveArmFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlMoveArmFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manfred_arm_msgs-msg:<ControlMoveArmFeedback> is deprecated: use manfred_arm_msgs-msg:ControlMoveArmFeedback instead.")))

(cl:ensure-generic-function 'feedbackStatus-val :lambda-list '(m))
(cl:defmethod feedbackStatus-val ((m <ControlMoveArmFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:feedbackStatus-val is deprecated.  Use manfred_arm_msgs-msg:feedbackStatus instead.")
  (feedbackStatus m))

(cl:ensure-generic-function 'posic_goal-val :lambda-list '(m))
(cl:defmethod posic_goal-val ((m <ControlMoveArmFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:posic_goal-val is deprecated.  Use manfred_arm_msgs-msg:posic_goal instead.")
  (posic_goal m))

(cl:ensure-generic-function 'rpy_goal-val :lambda-list '(m))
(cl:defmethod rpy_goal-val ((m <ControlMoveArmFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:rpy_goal-val is deprecated.  Use manfred_arm_msgs-msg:rpy_goal instead.")
  (rpy_goal m))

(cl:ensure-generic-function 'posic_actual-val :lambda-list '(m))
(cl:defmethod posic_actual-val ((m <ControlMoveArmFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:posic_actual-val is deprecated.  Use manfred_arm_msgs-msg:posic_actual instead.")
  (posic_actual m))

(cl:ensure-generic-function 'rpy_actual-val :lambda-list '(m))
(cl:defmethod rpy_actual-val ((m <ControlMoveArmFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:rpy_actual-val is deprecated.  Use manfred_arm_msgs-msg:rpy_actual instead.")
  (rpy_actual m))

(cl:ensure-generic-function 'ori-val :lambda-list '(m))
(cl:defmethod ori-val ((m <ControlMoveArmFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:ori-val is deprecated.  Use manfred_arm_msgs-msg:ori instead.")
  (ori m))

(cl:ensure-generic-function 'ultimo-val :lambda-list '(m))
(cl:defmethod ultimo-val ((m <ControlMoveArmFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:ultimo-val is deprecated.  Use manfred_arm_msgs-msg:ultimo instead.")
  (ultimo m))

(cl:ensure-generic-function 'joint_state-val :lambda-list '(m))
(cl:defmethod joint_state-val ((m <ControlMoveArmFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:joint_state-val is deprecated.  Use manfred_arm_msgs-msg:joint_state instead.")
  (joint_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlMoveArmFeedback>) ostream)
  "Serializes a message object of type '<ControlMoveArmFeedback>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'feedbackStatus) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'posic_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rpy_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'posic_actual) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rpy_actual) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ori) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ultimo) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlMoveArmFeedback>) istream)
  "Deserializes a message object of type '<ControlMoveArmFeedback>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'feedbackStatus) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'posic_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rpy_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'posic_actual) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rpy_actual) istream)
    (cl:setf (cl:slot-value msg 'ori) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ultimo) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlMoveArmFeedback>)))
  "Returns string type for a message object of type '<ControlMoveArmFeedback>"
  "manfred_arm_msgs/ControlMoveArmFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlMoveArmFeedback)))
  "Returns string type for a message object of type 'ControlMoveArmFeedback"
  "manfred_arm_msgs/ControlMoveArmFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlMoveArmFeedback>)))
  "Returns md5sum for a message object of type '<ControlMoveArmFeedback>"
  "199cddfab3aded47d8fe5d84a3572747")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlMoveArmFeedback)))
  "Returns md5sum for a message object of type 'ControlMoveArmFeedback"
  "199cddfab3aded47d8fe5d84a3572747")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlMoveArmFeedback>)))
  "Returns full string definition for message of type '<ControlMoveArmFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%#Header header~%actionlib_msgs/GoalStatus feedbackStatus~%geometry_msgs/Point32 posic_goal~%geometry_msgs/Vector3 rpy_goal~%geometry_msgs/Point32 posic_actual~%geometry_msgs/Vector3 rpy_actual~%#Permite indicar si hay que tener en cuanta la orientación del efector final~%bool ori~%#Marca la última posición de una trayectoria~%bool ultimo~%#para simulacion~%sensor_msgs/JointState joint_state~%~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlMoveArmFeedback)))
  "Returns full string definition for message of type 'ControlMoveArmFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%#Header header~%actionlib_msgs/GoalStatus feedbackStatus~%geometry_msgs/Point32 posic_goal~%geometry_msgs/Vector3 rpy_goal~%geometry_msgs/Point32 posic_actual~%geometry_msgs/Vector3 rpy_actual~%#Permite indicar si hay que tener en cuanta la orientación del efector final~%bool ori~%#Marca la última posición de una trayectoria~%bool ultimo~%#para simulacion~%sensor_msgs/JointState joint_state~%~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlMoveArmFeedback>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'feedbackStatus))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'posic_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rpy_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'posic_actual))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rpy_actual))
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlMoveArmFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlMoveArmFeedback
    (cl:cons ':feedbackStatus (feedbackStatus msg))
    (cl:cons ':posic_goal (posic_goal msg))
    (cl:cons ':rpy_goal (rpy_goal msg))
    (cl:cons ':posic_actual (posic_actual msg))
    (cl:cons ':rpy_actual (rpy_actual msg))
    (cl:cons ':ori (ori msg))
    (cl:cons ':ultimo (ultimo msg))
    (cl:cons ':joint_state (joint_state msg))
))
