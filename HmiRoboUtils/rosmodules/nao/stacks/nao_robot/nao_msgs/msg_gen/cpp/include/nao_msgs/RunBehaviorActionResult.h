/* Auto-generated by genmsg_cpp for file /home/rob/ros_workspace/rosmodules/nao/stacks/nao_robot/nao_msgs/msg/RunBehaviorActionResult.msg */
#ifndef NAO_MSGS_MESSAGE_RUNBEHAVIORACTIONRESULT_H
#define NAO_MSGS_MESSAGE_RUNBEHAVIORACTIONRESULT_H
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
#include "actionlib_msgs/GoalStatus.h"
#include "nao_msgs/RunBehaviorResult.h"

namespace nao_msgs
{
template <class ContainerAllocator>
struct RunBehaviorActionResult_ {
  typedef RunBehaviorActionResult_<ContainerAllocator> Type;

  RunBehaviorActionResult_()
  : header()
  , status()
  , result()
  {
  }

  RunBehaviorActionResult_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , status(_alloc)
  , result(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::actionlib_msgs::GoalStatus_<ContainerAllocator>  _status_type;
   ::actionlib_msgs::GoalStatus_<ContainerAllocator>  status;

  typedef  ::nao_msgs::RunBehaviorResult_<ContainerAllocator>  _result_type;
   ::nao_msgs::RunBehaviorResult_<ContainerAllocator>  result;


  typedef boost::shared_ptr< ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RunBehaviorActionResult
typedef  ::nao_msgs::RunBehaviorActionResult_<std::allocator<void> > RunBehaviorActionResult;

typedef boost::shared_ptr< ::nao_msgs::RunBehaviorActionResult> RunBehaviorActionResultPtr;
typedef boost::shared_ptr< ::nao_msgs::RunBehaviorActionResult const> RunBehaviorActionResultConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace nao_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ceba2a60bf895ecbe102b353cb4b589c";
  }

  static const char* value(const  ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xceba2a60bf895ecbULL;
  static const uint64_t static_value2 = 0xe102b353cb4b589cULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/RunBehaviorActionResult";
  }

  static const char* value(const  ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
RunBehaviorResult result\n\
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
MSG: nao_msgs/RunBehaviorResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
bool noErrors\n\
\n\
";
  }

  static const char* value(const  ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.status);
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RunBehaviorActionResult_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::nao_msgs::RunBehaviorActionResult_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "status: ";
s << std::endl;
    Printer< ::actionlib_msgs::GoalStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
    s << indent << "result: ";
s << std::endl;
    Printer< ::nao_msgs::RunBehaviorResult_<ContainerAllocator> >::stream(s, indent + "  ", v.result);
  }
};


} // namespace message_operations
} // namespace ros

#endif // NAO_MSGS_MESSAGE_RUNBEHAVIORACTIONRESULT_H

