; Auto-generated. Do not edit!


(cl:in-package manfred_arm_msgs-msg)


;//! \htmlinclude ControlMoveArmAction.msg.html

(cl:defclass <ControlMoveArmAction> (roslisp-msg-protocol:ros-message)
  ((action_goal
    :reader action_goal
    :initarg :action_goal
    :type manfred_arm_msgs-msg:ControlMoveArmActionGoal
    :initform (cl:make-instance 'manfred_arm_msgs-msg:ControlMoveArmActionGoal))
   (action_result
    :reader action_result
    :initarg :action_result
    :type manfred_arm_msgs-msg:ControlMoveArmActionResult
    :initform (cl:make-instance 'manfred_arm_msgs-msg:ControlMoveArmActionResult))
   (action_feedback
    :reader action_feedback
    :initarg :action_feedback
    :type manfred_arm_msgs-msg:ControlMoveArmActionFeedback
    :initform (cl:make-instance 'manfred_arm_msgs-msg:ControlMoveArmActionFeedback)))
)

(cl:defclass ControlMoveArmAction (<ControlMoveArmAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlMoveArmAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlMoveArmAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manfred_arm_msgs-msg:<ControlMoveArmAction> is deprecated: use manfred_arm_msgs-msg:ControlMoveArmAction instead.")))

(cl:ensure-generic-function 'action_goal-val :lambda-list '(m))
(cl:defmethod action_goal-val ((m <ControlMoveArmAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:action_goal-val is deprecated.  Use manfred_arm_msgs-msg:action_goal instead.")
  (action_goal m))

(cl:ensure-generic-function 'action_result-val :lambda-list '(m))
(cl:defmethod action_result-val ((m <ControlMoveArmAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:action_result-val is deprecated.  Use manfred_arm_msgs-msg:action_result instead.")
  (action_result m))

(cl:ensure-generic-function 'action_feedback-val :lambda-list '(m))
(cl:defmethod action_feedback-val ((m <ControlMoveArmAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manfred_arm_msgs-msg:action_feedback-val is deprecated.  Use manfred_arm_msgs-msg:action_feedback instead.")
  (action_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlMoveArmAction>) ostream)
  "Serializes a message object of type '<ControlMoveArmAction>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_result) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_feedback) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlMoveArmAction>) istream)
  "Deserializes a message object of type '<ControlMoveArmAction>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_result) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_feedback) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlMoveArmAction>)))
  "Returns string type for a message object of type '<ControlMoveArmAction>"
  "manfred_arm_msgs/ControlMoveArmAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlMoveArmAction)))
  "Returns string type for a message object of type 'ControlMoveArmAction"
  "manfred_arm_msgs/ControlMoveArmAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlMoveArmAction>)))
  "Returns md5sum for a message object of type '<ControlMoveArmAction>"
  "ac4a190c307267c99c01b2cc1dcc9a03")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlMoveArmAction)))
  "Returns md5sum for a message object of type 'ControlMoveArmAction"
  "ac4a190c307267c99c01b2cc1dcc9a03")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlMoveArmAction>)))
  "Returns full string definition for message of type '<ControlMoveArmAction>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%ControlMoveArmActionGoal action_goal~%ControlMoveArmActionResult action_result~%ControlMoveArmActionFeedback action_feedback~%~%================================================================================~%MSG: manfred_arm_msgs/ControlMoveArmActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%ControlMoveArmGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: manfred_arm_msgs/ControlMoveArmGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%#Header header~%float32 max_frequency~%duration time_offset~%geometry_msgs/Point32 posic~%geometry_msgs/Vector3 rpy~%#Permite indicar si hay que tener en cuanta la orientación del efector final~%bool ori~%#Marca la última posición de una trayectoria~%bool ultimo~%# ¿Hara falta la velocidad? Al menos hace falta para la funcion de simulacion del robot, tanto vel como position~%sensor_msgs/JointState joint_state~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: manfred_arm_msgs/ControlMoveArmActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ControlMoveArmResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: manfred_arm_msgs/ControlMoveArmResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%#Header header~%actionlib_msgs/GoalStatus resultStatus~%geometry_msgs/Point32 posic~%geometry_msgs/Vector3 rpy~%#para simulacion~%sensor_msgs/JointState joint_state~%~%================================================================================~%MSG: manfred_arm_msgs/ControlMoveArmActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ControlMoveArmFeedback feedback~%~%================================================================================~%MSG: manfred_arm_msgs/ControlMoveArmFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%#Header header~%actionlib_msgs/GoalStatus feedbackStatus~%geometry_msgs/Point32 posic_goal~%geometry_msgs/Vector3 rpy_goal~%geometry_msgs/Point32 posic_actual~%geometry_msgs/Vector3 rpy_actual~%#Permite indicar si hay que tener en cuanta la orientación del efector final~%bool ori~%#Marca la última posición de una trayectoria~%bool ultimo~%#para simulacion~%sensor_msgs/JointState joint_state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlMoveArmAction)))
  "Returns full string definition for message of type 'ControlMoveArmAction"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%ControlMoveArmActionGoal action_goal~%ControlMoveArmActionResult action_result~%ControlMoveArmActionFeedback action_feedback~%~%================================================================================~%MSG: manfred_arm_msgs/ControlMoveArmActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%ControlMoveArmGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: manfred_arm_msgs/ControlMoveArmGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%#Header header~%float32 max_frequency~%duration time_offset~%geometry_msgs/Point32 posic~%geometry_msgs/Vector3 rpy~%#Permite indicar si hay que tener en cuanta la orientación del efector final~%bool ori~%#Marca la última posición de una trayectoria~%bool ultimo~%# ¿Hara falta la velocidad? Al menos hace falta para la funcion de simulacion del robot, tanto vel como position~%sensor_msgs/JointState joint_state~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: manfred_arm_msgs/ControlMoveArmActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ControlMoveArmResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: manfred_arm_msgs/ControlMoveArmResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%#Header header~%actionlib_msgs/GoalStatus resultStatus~%geometry_msgs/Point32 posic~%geometry_msgs/Vector3 rpy~%#para simulacion~%sensor_msgs/JointState joint_state~%~%================================================================================~%MSG: manfred_arm_msgs/ControlMoveArmActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ControlMoveArmFeedback feedback~%~%================================================================================~%MSG: manfred_arm_msgs/ControlMoveArmFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%#Header header~%actionlib_msgs/GoalStatus feedbackStatus~%geometry_msgs/Point32 posic_goal~%geometry_msgs/Vector3 rpy_goal~%geometry_msgs/Point32 posic_actual~%geometry_msgs/Vector3 rpy_actual~%#Permite indicar si hay que tener en cuanta la orientación del efector final~%bool ori~%#Marca la última posición de una trayectoria~%bool ultimo~%#para simulacion~%sensor_msgs/JointState joint_state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlMoveArmAction>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_result))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlMoveArmAction>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlMoveArmAction
    (cl:cons ':action_goal (action_goal msg))
    (cl:cons ':action_result (action_result msg))
    (cl:cons ':action_feedback (action_feedback msg))
))