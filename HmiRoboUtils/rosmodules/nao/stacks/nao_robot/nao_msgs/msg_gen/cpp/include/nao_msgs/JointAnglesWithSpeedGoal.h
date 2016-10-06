/* Auto-generated by genmsg_cpp for file /home/rob/ros_workspace/rosmodules/nao/stacks/nao_robot/nao_msgs/msg/JointAnglesWithSpeedGoal.msg */
#ifndef NAO_MSGS_MESSAGE_JOINTANGLESWITHSPEEDGOAL_H
#define NAO_MSGS_MESSAGE_JOINTANGLESWITHSPEEDGOAL_H
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

#include "nao_msgs/JointAnglesWithSpeed.h"

namespace nao_msgs
{
template <class ContainerAllocator>
struct JointAnglesWithSpeedGoal_ {
  typedef JointAnglesWithSpeedGoal_<ContainerAllocator> Type;

  JointAnglesWithSpeedGoal_()
  : joint_angles()
  {
  }

  JointAnglesWithSpeedGoal_(const ContainerAllocator& _alloc)
  : joint_angles(_alloc)
  {
  }

  typedef  ::nao_msgs::JointAnglesWithSpeed_<ContainerAllocator>  _joint_angles_type;
   ::nao_msgs::JointAnglesWithSpeed_<ContainerAllocator>  joint_angles;


  typedef boost::shared_ptr< ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct JointAnglesWithSpeedGoal
typedef  ::nao_msgs::JointAnglesWithSpeedGoal_<std::allocator<void> > JointAnglesWithSpeedGoal;

typedef boost::shared_ptr< ::nao_msgs::JointAnglesWithSpeedGoal> JointAnglesWithSpeedGoalPtr;
typedef boost::shared_ptr< ::nao_msgs::JointAnglesWithSpeedGoal const> JointAnglesWithSpeedGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace nao_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d19a898a40aae87b37b0f91c9e90f46c";
  }

  static const char* value(const  ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd19a898a40aae87bULL;
  static const uint64_t static_value2 = 0x37b0f91c9e90f46cULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/JointAnglesWithSpeedGoal";
  }

  static const char* value(const  ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
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
";
  }

  static const char* value(const  ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.joint_angles);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct JointAnglesWithSpeedGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::nao_msgs::JointAnglesWithSpeedGoal_<ContainerAllocator> & v) 
  {
    s << indent << "joint_angles: ";
s << std::endl;
    Printer< ::nao_msgs::JointAnglesWithSpeed_<ContainerAllocator> >::stream(s, indent + "  ", v.joint_angles);
  }
};


} // namespace message_operations
} // namespace ros

#endif // NAO_MSGS_MESSAGE_JOINTANGLESWITHSPEEDGOAL_H
