/* Auto-generated by genmsg_cpp for file /home/rob/ros_workspace/rosmodules/nao/stacks/nao_robot/nao_msgs/msg/JointAnglesWithSpeedActionGoal.msg */
#ifndef NAO_MSGS_MESSAGE_JOINTANGLESWITHSPEEDACTIONGOAL_H
#define NAO_MSGS_MESSAGE_JOINTANGLESWITHSPEEDACTIONGOAL_H
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

#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "nao_msgs/JointAnglesWithSpeedGoal.h"

namespace nao_msgs
{
template <class ContainerAllocator>
struct JointAnglesWithSpeedActionGoal_ {
  typedef JointAnglesWithSpeedActionGoal_<ContainerAllocator> Type;

  JointAnglesWithSpeedActionGoal_()
  : header()
  , goal_id()
  , goal()
  {
  }

  JointAnglesWithSpeedActionGoal_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , goal_id(_alloc)
  , goal(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
   ::actionlib_msgs::GoalID_<ContainerAllocator>  goal_id;

  typedef  ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator>  _goal_type;
   ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator>  goal;


  typedef boost::shared_ptr< ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct JointAnglesWithSpeedActionGoal
typedef  ::nao_msgs::JointAnglesWithSpeedActionGoal_<std::allocator<void> > JointAnglesWithSpeedActionGoal;

typedef boost::shared_ptr< ::nao_msgs::JointAnglesWithSpeedActionGoal> JointAnglesWithSpeedActionGoalPtr;
typedef boost::shared_ptr< ::nao_msgs::JointAnglesWithSpeedActionGoal const> JointAnglesWithSpeedActionGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace nao_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9b722e9749aa53fc0e8ca7aa12e95efb";
  }

  static const char* value(const  ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x9b722e9749aa53fcULL;
  static const uint64_t static_value2 = 0x0e8ca7aa12e95efbULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/JointAnglesWithSpeedActionGoal";
  }

  static const char* value(const  ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
JointAnglesWithSpeedGoal goal\n\
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
MSG: nao_msgs/JointAnglesWithSpeedGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# goal: a registered body pose name\n\
nao_msgs/JointAnglesWithSpeed joint_angles\n\
\n\
================================================================================\n\
MSG: nao_msgs/JointAnglesWithSpeed\n\
Header header\n\
\n\
# A list of joint names, corresponding to their names in the Nao docs.\n\
# This must be either the same lenght of joint_angles or 1 if it's a\n\
# keyword such as 'Body' (for all angles)\n\
string[] joint_names\n\
float32[] joint_angles\n\
\n\
#fraction of max joint velocity [0:1]\n\
float32 speed\n\
\n\
# Absolute angle(=0, default) or relative change\n\
uint8 relative\n\
\n\
";
  }

  static const char* value(const  ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.goal_id);
    stream.next(m.goal);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct JointAnglesWithSpeedActionGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::nao_msgs::JointAnglesWithSpeedActionGoal_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
s << std::endl;
    Printer< ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};


} // namespace message_operations
} // namespace ros

#endif // NAO_MSGS_MESSAGE_JOINTANGLESWITHSPEEDACTIONGOAL_H
