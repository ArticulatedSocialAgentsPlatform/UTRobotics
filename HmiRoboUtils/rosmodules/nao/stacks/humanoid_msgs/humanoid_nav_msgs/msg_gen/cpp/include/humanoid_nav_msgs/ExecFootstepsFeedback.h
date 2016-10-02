/* Auto-generated by genmsg_cpp for file /home/rob/ros_workspace/rosmodules/nao/stacks/humanoid_msgs/humanoid_nav_msgs/msg/ExecFootstepsFeedback.msg */
#ifndef HUMANOID_NAV_MSGS_MESSAGE_EXECFOOTSTEPSFEEDBACK_H
#define HUMANOID_NAV_MSGS_MESSAGE_EXECFOOTSTEPSFEEDBACK_H
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

#include "humanoid_nav_msgs/StepTarget.h"

namespace humanoid_nav_msgs
{
template <class ContainerAllocator>
struct ExecFootstepsFeedback_ {
  typedef ExecFootstepsFeedback_<ContainerAllocator> Type;

  ExecFootstepsFeedback_()
  : executed_footsteps()
  {
  }

  ExecFootstepsFeedback_(const ContainerAllocator& _alloc)
  : executed_footsteps(_alloc)
  {
  }

  typedef std::vector< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >::other >  _executed_footsteps_type;
  std::vector< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >::other >  executed_footsteps;


  typedef boost::shared_ptr< ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ExecFootstepsFeedback
typedef  ::humanoid_nav_msgs::ExecFootstepsFeedback_<std::allocator<void> > ExecFootstepsFeedback;

typedef boost::shared_ptr< ::humanoid_nav_msgs::ExecFootstepsFeedback> ExecFootstepsFeedbackPtr;
typedef boost::shared_ptr< ::humanoid_nav_msgs::ExecFootstepsFeedback const> ExecFootstepsFeedbackConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace humanoid_nav_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5dfde2cb244d6c76567d3c52c40a988c";
  }

  static const char* value(const  ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5dfde2cb244d6c76ULL;
  static const uint64_t static_value2 = 0x567d3c52c40a988cULL;
};

template<class ContainerAllocator>
struct DataType< ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "humanoid_nav_msgs/ExecFootstepsFeedback";
  }

  static const char* value(const  ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Define a feedback message\n\
humanoid_nav_msgs/StepTarget[] executed_footsteps\n\
\n\
\n\
================================================================================\n\
MSG: humanoid_nav_msgs/StepTarget\n\
# Target for a single stepping motion of a humanoid's leg\n\
\n\
geometry_msgs/Pose2D pose   # step pose as relative offset to last leg\n\
uint8 leg                   # which leg to use (left/right, see below)\n\
\n\
uint8 right=0               # right leg constant\n\
uint8 left=1                # left leg constant\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
";
  }

  static const char* value(const  ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.executed_footsteps);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ExecFootstepsFeedback_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::humanoid_nav_msgs::ExecFootstepsFeedback_<ContainerAllocator> & v) 
  {
    s << indent << "executed_footsteps[]" << std::endl;
    for (size_t i = 0; i < v.executed_footsteps.size(); ++i)
    {
      s << indent << "  executed_footsteps[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::humanoid_nav_msgs::StepTarget_<ContainerAllocator> >::stream(s, indent + "    ", v.executed_footsteps[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // HUMANOID_NAV_MSGS_MESSAGE_EXECFOOTSTEPSFEEDBACK_H

