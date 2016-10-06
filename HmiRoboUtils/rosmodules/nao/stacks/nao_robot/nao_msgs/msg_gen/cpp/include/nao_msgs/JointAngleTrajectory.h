/* Auto-generated by genmsg_cpp for file /home/rob/ros_workspace/rosmodules/nao/stacks/nao_robot/nao_msgs/msg/JointAngleTrajectory.msg */
#ifndef NAO_MSGS_MESSAGE_JOINTANGLETRAJECTORY_H
#define NAO_MSGS_MESSAGE_JOINTANGLETRAJECTORY_H
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

namespace nao_msgs
{
template <class ContainerAllocator>
struct JointAngleTrajectory_ {
  typedef JointAngleTrajectory_<ContainerAllocator> Type;

  JointAngleTrajectory_()
  : header()
  , joint_names()
  , joint_angles()
  , times()
  , relative(0)
  {
  }

  JointAngleTrajectory_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , joint_names(_alloc)
  , joint_angles(_alloc)
  , times(_alloc)
  , relative(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _joint_names_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  joint_names;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _joint_angles_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  joint_angles;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _times_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  times;

  typedef uint8_t _relative_type;
  uint8_t relative;


  typedef boost::shared_ptr< ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::JointAngleTrajectory_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct JointAngleTrajectory
typedef  ::nao_msgs::JointAngleTrajectory_<std::allocator<void> > JointAngleTrajectory;

typedef boost::shared_ptr< ::nao_msgs::JointAngleTrajectory> JointAngleTrajectoryPtr;
typedef boost::shared_ptr< ::nao_msgs::JointAngleTrajectory const> JointAngleTrajectoryConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace nao_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::JointAngleTrajectory_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5ec9aa90e61498421ea53db10da7f8dd";
  }

  static const char* value(const  ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5ec9aa90e6149842ULL;
  static const uint64_t static_value2 = 0x1ea53db10da7f8ddULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/JointAngleTrajectory";
  }

  static const char* value(const  ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
\n\
# A list of joint names, corresponding to their names in the Nao docs.\n\
# This must be either the same lenght of joint_angles or 1 if it's a\n\
# keyword such as 'Body' (for all angles)\n\
string[] joint_names\n\
float32[] joint_angles\n\
float32[] times\n\
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

  static const char* value(const  ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.joint_names);
    stream.next(m.joint_angles);
    stream.next(m.times);
    stream.next(m.relative);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct JointAngleTrajectory_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::nao_msgs::JointAngleTrajectory_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "joint_names[]" << std::endl;
    for (size_t i = 0; i < v.joint_names.size(); ++i)
    {
      s << indent << "  joint_names[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.joint_names[i]);
    }
    s << indent << "joint_angles[]" << std::endl;
    for (size_t i = 0; i < v.joint_angles.size(); ++i)
    {
      s << indent << "  joint_angles[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.joint_angles[i]);
    }
    s << indent << "times[]" << std::endl;
    for (size_t i = 0; i < v.times.size(); ++i)
    {
      s << indent << "  times[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.times[i]);
    }
    s << indent << "relative: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.relative);
  }
};


} // namespace message_operations
} // namespace ros

#endif // NAO_MSGS_MESSAGE_JOINTANGLETRAJECTORY_H
