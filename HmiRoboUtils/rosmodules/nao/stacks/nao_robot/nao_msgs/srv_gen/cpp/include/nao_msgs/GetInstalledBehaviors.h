/* Auto-generated by genmsg_cpp for file /home/rob/ros_workspace/rosmodules/nao/stacks/nao_robot/nao_msgs/srv/GetInstalledBehaviors.srv */
#ifndef NAO_MSGS_SERVICE_GETINSTALLEDBEHAVIORS_H
#define NAO_MSGS_SERVICE_GETINSTALLEDBEHAVIORS_H
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

#include "ros/service_traits.h"




namespace nao_msgs
{
template <class ContainerAllocator>
struct GetInstalledBehaviorsRequest_ {
  typedef GetInstalledBehaviorsRequest_<ContainerAllocator> Type;

  GetInstalledBehaviorsRequest_()
  {
  }

  GetInstalledBehaviorsRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct GetInstalledBehaviorsRequest
typedef  ::nao_msgs::GetInstalledBehaviorsRequest_<std::allocator<void> > GetInstalledBehaviorsRequest;

typedef boost::shared_ptr< ::nao_msgs::GetInstalledBehaviorsRequest> GetInstalledBehaviorsRequestPtr;
typedef boost::shared_ptr< ::nao_msgs::GetInstalledBehaviorsRequest const> GetInstalledBehaviorsRequestConstPtr;


template <class ContainerAllocator>
struct GetInstalledBehaviorsResponse_ {
  typedef GetInstalledBehaviorsResponse_<ContainerAllocator> Type;

  GetInstalledBehaviorsResponse_()
  : behaviors()
  {
  }

  GetInstalledBehaviorsResponse_(const ContainerAllocator& _alloc)
  : behaviors(_alloc)
  {
  }

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _behaviors_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  behaviors;


  typedef boost::shared_ptr< ::nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct GetInstalledBehaviorsResponse
typedef  ::nao_msgs::GetInstalledBehaviorsResponse_<std::allocator<void> > GetInstalledBehaviorsResponse;

typedef boost::shared_ptr< ::nao_msgs::GetInstalledBehaviorsResponse> GetInstalledBehaviorsResponsePtr;
typedef boost::shared_ptr< ::nao_msgs::GetInstalledBehaviorsResponse const> GetInstalledBehaviorsResponseConstPtr;

struct GetInstalledBehaviors
{

typedef GetInstalledBehaviorsRequest Request;
typedef GetInstalledBehaviorsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct GetInstalledBehaviors
} // namespace nao_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/GetInstalledBehaviorsRequest";
  }

  static const char* value(const  ::nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
";
  }

  static const char* value(const  ::nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "715783c8c6eb28fc2e1c05184add75ec";
  }

  static const char* value(const  ::nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x715783c8c6eb28fcULL;
  static const uint64_t static_value2 = 0x2e1c05184add75ecULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/GetInstalledBehaviorsResponse";
  }

  static const char* value(const  ::nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string[] behaviors\n\
\n\
\n\
";
  }

  static const char* value(const  ::nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GetInstalledBehaviorsRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.behaviors);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GetInstalledBehaviorsResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<nao_msgs::GetInstalledBehaviors> {
  static const char* value() 
  {
    return "715783c8c6eb28fc2e1c05184add75ec";
  }

  static const char* value(const nao_msgs::GetInstalledBehaviors&) { return value(); } 
};

template<>
struct DataType<nao_msgs::GetInstalledBehaviors> {
  static const char* value() 
  {
    return "nao_msgs/GetInstalledBehaviors";
  }

  static const char* value(const nao_msgs::GetInstalledBehaviors&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "715783c8c6eb28fc2e1c05184add75ec";
  }

  static const char* value(const nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/GetInstalledBehaviors";
  }

  static const char* value(const nao_msgs::GetInstalledBehaviorsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "715783c8c6eb28fc2e1c05184add75ec";
  }

  static const char* value(const nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/GetInstalledBehaviors";
  }

  static const char* value(const nao_msgs::GetInstalledBehaviorsResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // NAO_MSGS_SERVICE_GETINSTALLEDBEHAVIORS_H

