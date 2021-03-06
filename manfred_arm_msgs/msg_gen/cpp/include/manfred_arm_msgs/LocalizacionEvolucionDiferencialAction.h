/* Auto-generated by genmsg_cpp for file /home/disa1301/Documents/Codigo/Manfred/ros/manfred_arm_msgs/msg/LocalizacionEvolucionDiferencialAction.msg */
#ifndef MANFRED_ARM_MSGS_MESSAGE_LOCALIZACIONEVOLUCIONDIFERENCIALACTION_H
#define MANFRED_ARM_MSGS_MESSAGE_LOCALIZACIONEVOLUCIONDIFERENCIALACTION_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "manfred_arm_msgs/LocalizacionEvolucionDiferencialActionGoal.h"
#include "manfred_arm_msgs/LocalizacionEvolucionDiferencialActionResult.h"
#include "manfred_arm_msgs/LocalizacionEvolucionDiferencialActionFeedback.h"

namespace manfred_arm_msgs
{
template <class ContainerAllocator>
struct LocalizacionEvolucionDiferencialAction_ {
  typedef LocalizacionEvolucionDiferencialAction_<ContainerAllocator> Type;

  LocalizacionEvolucionDiferencialAction_()
  : action_goal()
  , action_result()
  , action_feedback()
  {
  }

  LocalizacionEvolucionDiferencialAction_(const ContainerAllocator& _alloc)
  : action_goal(_alloc)
  , action_result(_alloc)
  , action_feedback(_alloc)
  {
  }

  typedef  ::manfred_arm_msgs::LocalizacionEvolucionDiferencialActionGoal_<ContainerAllocator>  _action_goal_type;
   ::manfred_arm_msgs::LocalizacionEvolucionDiferencialActionGoal_<ContainerAllocator>  action_goal;

  typedef  ::manfred_arm_msgs::LocalizacionEvolucionDiferencialActionResult_<ContainerAllocator>  _action_result_type;
   ::manfred_arm_msgs::LocalizacionEvolucionDiferencialActionResult_<ContainerAllocator>  action_result;

  typedef  ::manfred_arm_msgs::LocalizacionEvolucionDiferencialActionFeedback_<ContainerAllocator>  _action_feedback_type;
   ::manfred_arm_msgs::LocalizacionEvolucionDiferencialActionFeedback_<ContainerAllocator>  action_feedback;


  typedef boost::shared_ptr< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct LocalizacionEvolucionDiferencialAction
typedef  ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<std::allocator<void> > LocalizacionEvolucionDiferencialAction;

typedef boost::shared_ptr< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction> LocalizacionEvolucionDiferencialActionPtr;
typedef boost::shared_ptr< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction const> LocalizacionEvolucionDiferencialActionConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace manfred_arm_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6606b5f8e787cdb9d2fe2feb101413e0";
  }

  static const char* value(const  ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6606b5f8e787cdb9ULL;
  static const uint64_t static_value2 = 0xd2fe2feb101413e0ULL;
};

template<class ContainerAllocator>
struct DataType< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manfred_arm_msgs/LocalizacionEvolucionDiferencialAction";
  }

  static const char* value(const  ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
LocalizacionEvolucionDiferencialActionGoal action_goal\n\
LocalizacionEvolucionDiferencialActionResult action_result\n\
LocalizacionEvolucionDiferencialActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: manfred_arm_msgs/LocalizacionEvolucionDiferencialActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
LocalizacionEvolucionDiferencialGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: manfred_arm_msgs/LocalizacionEvolucionDiferencialGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#goal definition\n\
bool dummyFlagGoal\n\
\n\
================================================================================\n\
MSG: manfred_arm_msgs/LocalizacionEvolucionDiferencialActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
LocalizacionEvolucionDiferencialResult result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: manfred_arm_msgs/LocalizacionEvolucionDiferencialResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#result definition\n\
# Ubicación del robot en el sistema de referencia global (SRG)\n\
geometry_msgs/PoseStamped ubicacionRobotSRG\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: manfred_arm_msgs/LocalizacionEvolucionDiferencialActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
LocalizacionEvolucionDiferencialFeedback feedback\n\
\n\
================================================================================\n\
MSG: manfred_arm_msgs/LocalizacionEvolucionDiferencialFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#feedback\n\
bool dummyFlagFeedback\n\
\n\
\n\
";
  }

  static const char* value(const  ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.action_goal);
    stream.next(m.action_result);
    stream.next(m.action_feedback);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct LocalizacionEvolucionDiferencialAction_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::manfred_arm_msgs::LocalizacionEvolucionDiferencialAction_<ContainerAllocator> & v) 
  {
    s << indent << "action_goal: ";
s << std::endl;
    Printer< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
s << std::endl;
    Printer< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
s << std::endl;
    Printer< ::manfred_arm_msgs::LocalizacionEvolucionDiferencialActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};


} // namespace message_operations
} // namespace ros

#endif // MANFRED_ARM_MSGS_MESSAGE_LOCALIZACIONEVOLUCIONDIFERENCIALACTION_H

