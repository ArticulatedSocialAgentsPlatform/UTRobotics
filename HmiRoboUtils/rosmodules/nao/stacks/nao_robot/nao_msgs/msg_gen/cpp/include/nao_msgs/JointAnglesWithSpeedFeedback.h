/* Auto-generated by genmsg_cpp for file /home/rob/ros_workspace/rosmodules/nao/stacks/nao_robot/nao_msgs/msg/JointAnglesWithSpeedFeedback.msg */
#ifndef NAO_MSGS_MESSAGE_JOINTANGLESWITHSPEEDFEEDBACK_H
#define NAO_MSGS_MESSAGE_JOINTANGLESWITHSPEEDFEEDBACK_H
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


namespace nao_msgs
{
template <class ContainerAllocator>
struct JointAnglesWithSpeedFeedback_ {
  typedef JointAnglesWithSpeedFeedback_<ContainerAllocator> Type;

  JointAnglesWithSpeedFeedback_()
  {
  }

  JointAnglesWithSpeedFeedback_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct JointAnglesWithSpeedFeedback
typedef  ::nao_msgs::JointAnglesWithSpeedFeedback_<std::allocator<void> > JointAnglesWithSpeedFeedback;

typedef boost::shared_ptr< ::nao_msgs::JointAnglesWithSpeedFeedback> JointAnglesWithSpeedFeedbackPtr;
typedef boost::shared_ptr< ::nao_msgs::JointAnglesWithSpeedFeedback const> JointAnglesWithSpeedFeedbackConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace nao_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/JointAnglesWithSpeedFeedback";
  }

  static const char* value(const  ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# no feedback currently \n\
\n\
";
  }

  static const char* value(const  ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct JointAnglesWithSpeedFeedback_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::nao_msgs::JointAnglesWithSpeedFeedback_<ContainerAllocator> & v) 
  {
  }
};


} // namespace message_operations
} // namespace ros

#endif // NAO_MSGS_MESSAGE_JOINTANGLESWITHSPEEDFEEDBACK_H

