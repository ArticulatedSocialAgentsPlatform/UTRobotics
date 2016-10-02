/* Auto-generated by genmsg_cpp for file /home/rob/ros_workspace/rosmodules/nao/stacks/nao_robot/nao_msgs/msg/JointTrajectoryGoal.msg */
#ifndef NAO_MSGS_MESSAGE_JOINTTRAJECTORYGOAL_H
#define NAO_MSGS_MESSAGE_JOINTTRAJECTORYGOAL_H
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

#include "trajectory_msgs/JointTrajectory.h"

namespace nao_msgs
{
template <class ContainerAllocator>
struct JointTrajectoryGoal_ {
  typedef JointTrajectoryGoal_<ContainerAllocator> Type;

  JointTrajectoryGoal_()
  : trajectory()
  , relative(0)
  {
  }

  JointTrajectoryGoal_(const ContainerAllocator& _alloc)
  : trajectory(_alloc)
  , relative(0)
  {
  }

  typedef  ::trajectory_msgs::JointTrajectory_<ContainerAllocator>  _trajectory_type;
   ::trajectory_msgs::JointTrajectory_<ContainerAllocator>  trajectory;

  typedef uint8_t _relative_type;
  uint8_t relative;


  typedef boost::shared_ptr< ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct JointTrajectoryGoal
typedef  ::nao_msgs::JointTrajectoryGoal_<std::allocator<void> > JointTrajectoryGoal;

typedef boost::shared_ptr< ::nao_msgs::JointTrajectoryGoal> JointTrajectoryGoalPtr;
typedef boost::shared_ptr< ::nao_msgs::JointTrajectoryGoal const> JointTrajectoryGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace nao_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d206ff6714336d8099e6c534bc7eb204";
  }

  static const char* value(const  ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd206ff6714336d80ULL;
  static const uint64_t static_value2 = 0x99e6c534bc7eb204ULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/JointTrajectoryGoal";
  }

  static const char* value(const  ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
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
MSG: trajectory_msgs/JointTrajectoryPoint\n\
float64[] positions\n\
float64[] velocities\n\
float64[] accelerations\n\
duration time_from_start\n\
";
  }

  static const char* value(const  ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.trajectory);
    stream.next(m.relative);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct JointTrajectoryGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::nao_msgs::JointTrajectoryGoal_<ContainerAllocator> & v) 
  {
    s << indent << "trajectory: ";
s << std::endl;
    Printer< ::trajectory_msgs::JointTrajectory_<ContainerAllocator> >::stream(s, indent + "  ", v.trajectory);
    s << indent << "relative: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.relative);
  }
};


} // namespace message_operations
} // namespace ros

#endif // NAO_MSGS_MESSAGE_JOINTTRAJECTORYGOAL_H

