/* Auto-generated by genmsg_cpp for file /home/rob/ros_workspace/rosmodules/nao/stacks/nao_robot/nao_msgs/msg/JointTrajectoryAction.msg */
#ifndef NAO_MSGS_MESSAGE_JOINTTRAJECTORYACTION_H
#define NAO_MSGS_MESSAGE_JOINTTRAJECTORYACTION_H
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

#include "nao_msgs/JointTrajectoryActionGoal.h"
#include "nao_msgs/JointTrajectoryActionResult.h"
#include "nao_msgs/JointTrajectoryActionFeedback.h"

namespace nao_msgs
{
template <class ContainerAllocator>
struct JointTrajectoryAction_ {
  typedef JointTrajectoryAction_<ContainerAllocator> Type;

  JointTrajectoryAction_()
  : action_goal()
  , action_result()
  , action_feedback()
  {
  }

  JointTrajectoryAction_(const ContainerAllocator& _alloc)
  : action_goal(_alloc)
  , action_result(_alloc)
  , action_feedback(_alloc)
  {
  }

  typedef  ::nao_msgs::JointTrajectoryActionGoal_<ContainerAllocator>  _action_goal_type;
   ::nao_msgs::JointTrajectoryActionGoal_<ContainerAllocator>  action_goal;

  typedef  ::nao_msgs::JointTrajectoryActionResult_<ContainerAllocator>  _action_result_type;
   ::nao_msgs::JointTrajectoryActionResult_<ContainerAllocator>  action_result;

  typedef  ::nao_msgs::JointTrajectoryActionFeedback_<ContainerAllocator>  _action_feedback_type;
   ::nao_msgs::JointTrajectoryActionFeedback_<ContainerAllocator>  action_feedback;


  typedef boost::shared_ptr< ::nao_msgs::JointTrajectoryAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::JointTrajectoryAction_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct JointTrajectoryAction
typedef  ::nao_msgs::JointTrajectoryAction_<std::allocator<void> > JointTrajectoryAction;

typedef boost::shared_ptr< ::nao_msgs::JointTrajectoryAction> JointTrajectoryActionPtr;
typedef boost::shared_ptr< ::nao_msgs::JointTrajectoryAction const> JointTrajectoryActionConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::nao_msgs::JointTrajectoryAction_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::nao_msgs::JointTrajectoryAction_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace nao_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::JointTrajectoryAction_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::JointTrajectoryAction_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::JointTrajectoryAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "47b9ba22fa50be8763fd3e5c3b06e11f";
  }

  static const char* value(const  ::nao_msgs::JointTrajectoryAction_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x47b9ba22fa50be87ULL;
  static const uint64_t static_value2 = 0x63fd3e5c3b06e11fULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::JointTrajectoryAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/JointTrajectoryAction";
  }

  static const char* value(const  ::nao_msgs::JointTrajectoryAction_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::JointTrajectoryAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
JointTrajectoryActionGoal action_goal\n\
JointTrajectoryActionResult action_result\n\
JointTrajectoryActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: nao_msgs/JointTrajectoryActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
JointTrajectoryGoal goal\n\
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
MSG: nao_msgs/JointTrajectoryGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# goal: a joint angle trajectory\n\
trajectory_msgs/JointTrajectory trajectory\n\
# flag whether motion is absolute (=0, default) or relative (=1)\n\
uint8 relative\n\
\n\
================================================================================\n\
MSG: trajectory_msgs/JointTrajectory\n\
Header header\n\
string[] joint_names\n\
JointTrajectoryPoint[] points\n\
================================================================================\n\
MSG: trajectory_msgs/JointTrajectoryPoint\n\
float64[] positions\n\
float64[] velocities\n\
float64[] accelerations\n\
duration time_from_start\n\
================================================================================\n\
MSG: nao_msgs/JointTrajectoryActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
JointTrajectoryResult result\n\
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
MSG: nao_msgs/JointTrajectoryResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# result is the actually reached position\n\
sensor_msgs/JointState goal_position\n\
\n\
================================================================================\n\
MSG: sensor_msgs/JointState\n\
# This is a message that holds data to describe the state of a set of torque controlled joints. \n\
#\n\
# The state of each joint (revolute or prismatic) is defined by:\n\
#  * the position of the joint (rad or m),\n\
#  * the velocity of the joint (rad/s or m/s) and \n\
#  * the effort that is applied in the joint (Nm or N).\n\
#\n\
# Each joint is uniquely identified by its name\n\
# The header specifies the time at which the joint states were recorded. All the joint states\n\
# in one message have to be recorded at the same time.\n\
#\n\
# This message consists of a multiple arrays, one for each part of the joint state. \n\
# The goal is to make each of the fields optional. When e.g. your joints have no\n\
# effort associated with them, you can leave the effort array empty. \n\
#\n\
# All arrays in this message should have the same size, or be empty.\n\
# This is the only way to uniquely associate the joint name with the correct\n\
# states.\n\
\n\
\n\
Header header\n\
\n\
string[] name\n\
float64[] position\n\
float64[] velocity\n\
float64[] effort\n\
\n\
================================================================================\n\
MSG: nao_msgs/JointTrajectoryActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
JointTrajectoryFeedback feedback\n\
\n\
================================================================================\n\
MSG: nao_msgs/JointTrajectoryFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# no feedback currently \n\
\n\
";
  }

  static const char* value(const  ::nao_msgs::JointTrajectoryAction_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_msgs::JointTrajectoryAction_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.action_goal);
    stream.next(m.action_result);
    stream.next(m.action_feedback);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct JointTrajectoryAction_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nao_msgs::JointTrajectoryAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::nao_msgs::JointTrajectoryAction_<ContainerAllocator> & v) 
  {
    s << indent << "action_goal: ";
s << std::endl;
    Printer< ::nao_msgs::JointTrajectoryActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
s << std::endl;
    Printer< ::nao_msgs::JointTrajectoryActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
s << std::endl;
    Printer< ::nao_msgs::JointTrajectoryActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};


} // namespace message_operations
} // namespace ros

#endif // NAO_MSGS_MESSAGE_JOINTTRAJECTORYACTION_H

