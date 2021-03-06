/* Auto-generated by genmsg_cpp for file /home/rob/ros_workspace/rosmodules/nao/stacks/nao_robot/nao_msgs/msg/RunBehaviorGoal.msg */
#ifndef NAO_MSGS_MESSAGE_RUNBEHAVIORGOAL_H
#define NAO_MSGS_MESSAGE_RUNBEHAVIORGOAL_H
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
struct RunBehaviorGoal_ {
  typedef RunBehaviorGoal_<ContainerAllocator> Type;

  RunBehaviorGoal_()
  : behavior()
  {
  }

  RunBehaviorGoal_(const ContainerAllocator& _alloc)
  : behavior(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _behavior_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  behavior;


  typedef boost::shared_ptr< ::nao_msgs::RunBehaviorGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::RunBehaviorGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RunBehaviorGoal
typedef  ::nao_msgs::RunBehaviorGoal_<std::allocator<void> > RunBehaviorGoal;

typedef boost::shared_ptr< ::nao_msgs::RunBehaviorGoal> RunBehaviorGoalPtr;
typedef boost::shared_ptr< ::nao_msgs::RunBehaviorGoal const> RunBehaviorGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::nao_msgs::RunBehaviorGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::nao_msgs::RunBehaviorGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace nao_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::RunBehaviorGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::RunBehaviorGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::RunBehaviorGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "03729983c4b9be7a4f2b56846a7ccbdc";
  }

  static const char* value(const  ::nao_msgs::RunBehaviorGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x03729983c4b9be7aULL;
  static const uint64_t static_value2 = 0x4f2b56846a7ccbdcULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::RunBehaviorGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/RunBehaviorGoal";
  }

  static const char* value(const  ::nao_msgs::RunBehaviorGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::RunBehaviorGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Goal [behavior]: name of the behavior to be executed\n\
# Result [noErrors]: true if behavior finished executing correctly, false otherwise\n\
# Feedback: none as NaoQI API cannot be queried in this respect\n\
string behavior\n\
\n\
";
  }

  static const char* value(const  ::nao_msgs::RunBehaviorGoal_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_msgs::RunBehaviorGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.behavior);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RunBehaviorGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nao_msgs::RunBehaviorGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::nao_msgs::RunBehaviorGoal_<ContainerAllocator> & v) 
  {
    s << indent << "behavior: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.behavior);
  }
};


} // namespace message_operations
} // namespace ros

#endif // NAO_MSGS_MESSAGE_RUNBEHAVIORGOAL_H

