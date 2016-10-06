/* Auto-generated by genmsg_cpp for file /home/rob/ros_workspace/rosmodules/nao/stacks/nao_common/nao_remote/srv/SetTransform.srv */
#ifndef NAO_REMOTE_SERVICE_SETTRANSFORM_H
#define NAO_REMOTE_SERVICE_SETTRANSFORM_H
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

#include "geometry_msgs/Transform.h"



namespace nao_remote
{
template <class ContainerAllocator>
struct SetTransformRequest_ {
  typedef SetTransformRequest_<ContainerAllocator> Type;

  SetTransformRequest_()
  : offset()
  {
  }

  SetTransformRequest_(const ContainerAllocator& _alloc)
  : offset(_alloc)
  {
  }

  typedef  ::geometry_msgs::Transform_<ContainerAllocator>  _offset_type;
   ::geometry_msgs::Transform_<ContainerAllocator>  offset;


  typedef boost::shared_ptr< ::nao_remote::SetTransformRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_remote::SetTransformRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetTransformRequest
typedef  ::nao_remote::SetTransformRequest_<std::allocator<void> > SetTransformRequest;

typedef boost::shared_ptr< ::nao_remote::SetTransformRequest> SetTransformRequestPtr;
typedef boost::shared_ptr< ::nao_remote::SetTransformRequest const> SetTransformRequestConstPtr;


template <class ContainerAllocator>
struct SetTransformResponse_ {
  typedef SetTransformResponse_<ContainerAllocator> Type;

  SetTransformResponse_()
  {
  }

  SetTransformResponse_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::nao_remote::SetTransformResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_remote::SetTransformResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetTransformResponse
typedef  ::nao_remote::SetTransformResponse_<std::allocator<void> > SetTransformResponse;

typedef boost::shared_ptr< ::nao_remote::SetTransformResponse> SetTransformResponsePtr;
typedef boost::shared_ptr< ::nao_remote::SetTransformResponse const> SetTransformResponseConstPtr;

struct SetTransform
{

typedef SetTransformRequest Request;
typedef SetTransformResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SetTransform
} // namespace nao_remote

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_remote::SetTransformRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_remote::SetTransformRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_remote::SetTransformRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "67035ddf415a9bb64191f0e45b060e35";
  }

  static const char* value(const  ::nao_remote::SetTransformRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x67035ddf415a9bb6ULL;
  static const uint64_t static_value2 = 0x4191f0e45b060e35ULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_remote::SetTransformRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_remote/SetTransformRequest";
  }

  static const char* value(const  ::nao_remote::SetTransformRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_remote::SetTransformRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
geometry_msgs/Transform offset\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Transform\n\
# This represents the transform between two coordinate frames in free space.\n\
\n\
Vector3 translation\n\
Quaternion rotation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::nao_remote::SetTransformRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nao_remote::SetTransformRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_remote::SetTransformResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_remote::SetTransformResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_remote::SetTransformResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::nao_remote::SetTransformResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_remote::SetTransformResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_remote/SetTransformResponse";
  }

  static const char* value(const  ::nao_remote::SetTransformResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_remote::SetTransformResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
";
  }

  static const char* value(const  ::nao_remote::SetTransformResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nao_remote::SetTransformResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_remote::SetTransformRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.offset);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetTransformRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_remote::SetTransformResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetTransformResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<nao_remote::SetTransform> {
  static const char* value() 
  {
    return "67035ddf415a9bb64191f0e45b060e35";
  }

  static const char* value(const nao_remote::SetTransform&) { return value(); } 
};

template<>
struct DataType<nao_remote::SetTransform> {
  static const char* value() 
  {
    return "nao_remote/SetTransform";
  }

  static const char* value(const nao_remote::SetTransform&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nao_remote::SetTransformRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "67035ddf415a9bb64191f0e45b060e35";
  }

  static const char* value(const nao_remote::SetTransformRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nao_remote::SetTransformRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_remote/SetTransform";
  }

  static const char* value(const nao_remote::SetTransformRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nao_remote::SetTransformResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "67035ddf415a9bb64191f0e45b060e35";
  }

  static const char* value(const nao_remote::SetTransformResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nao_remote::SetTransformResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_remote/SetTransform";
  }

  static const char* value(const nao_remote::SetTransformResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // NAO_REMOTE_SERVICE_SETTRANSFORM_H
