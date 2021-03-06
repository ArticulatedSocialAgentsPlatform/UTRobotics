/* Auto-generated by genmsg_cpp for file /home/rob/ros_workspace/rosmodules/nao/stacks/nao_robot/nao_msgs/msg/TactileTouch.msg */
#ifndef NAO_MSGS_MESSAGE_TACTILETOUCH_H
#define NAO_MSGS_MESSAGE_TACTILETOUCH_H
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
struct TactileTouch_ {
  typedef TactileTouch_<ContainerAllocator> Type;

  TactileTouch_()
  : button(0)
  , state(0)
  {
  }

  TactileTouch_(const ContainerAllocator& _alloc)
  : button(0)
  , state(0)
  {
  }

  typedef uint8_t _button_type;
  uint8_t button;

  typedef uint8_t _state_type;
  uint8_t state;

  enum { buttonFront = 1 };
  enum { buttonMiddle = 2 };
  enum { buttonRear = 3 };
  enum { stateReleased = 0 };
  enum { statePressed = 1 };

  typedef boost::shared_ptr< ::nao_msgs::TactileTouch_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::TactileTouch_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct TactileTouch
typedef  ::nao_msgs::TactileTouch_<std::allocator<void> > TactileTouch;

typedef boost::shared_ptr< ::nao_msgs::TactileTouch> TactileTouchPtr;
typedef boost::shared_ptr< ::nao_msgs::TactileTouch const> TactileTouchConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::nao_msgs::TactileTouch_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::nao_msgs::TactileTouch_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace nao_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::TactileTouch_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::TactileTouch_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::TactileTouch_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b75165bf9dfed26d50ad4e3162304225";
  }

  static const char* value(const  ::nao_msgs::TactileTouch_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb75165bf9dfed26dULL;
  static const uint64_t static_value2 = 0x50ad4e3162304225ULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::TactileTouch_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/TactileTouch";
  }

  static const char* value(const  ::nao_msgs::TactileTouch_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::TactileTouch_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# A message for Nao's tactile interface (toucht buttons on the head)\n\
\n\
uint8 button            # which of the three segments is touched\n\
uint8 state             # pressed or released, see below\n\
\n\
uint8 buttonFront=1\n\
uint8 buttonMiddle=2\n\
uint8 buttonRear=3\n\
\n\
uint8 stateReleased=0\n\
uint8 statePressed=1\n\
\n\
";
  }

  static const char* value(const  ::nao_msgs::TactileTouch_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nao_msgs::TactileTouch_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_msgs::TactileTouch_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.button);
    stream.next(m.state);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct TactileTouch_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nao_msgs::TactileTouch_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::nao_msgs::TactileTouch_<ContainerAllocator> & v) 
  {
    s << indent << "button: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.button);
    s << indent << "state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.state);
  }
};


} // namespace message_operations
} // namespace ros

#endif // NAO_MSGS_MESSAGE_TACTILETOUCH_H

