/* Auto-generated by genmsg_cpp for file /home/rob/ros_workspace/rosmodules/nao/stacks/nao_robot/nao_msgs/msg/WordRecognized.msg */
#ifndef NAO_MSGS_MESSAGE_WORDRECOGNIZED_H
#define NAO_MSGS_MESSAGE_WORDRECOGNIZED_H
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
struct WordRecognized_ {
  typedef WordRecognized_<ContainerAllocator> Type;

  WordRecognized_()
  : words()
  , confidence_values()
  {
  }

  WordRecognized_(const ContainerAllocator& _alloc)
  : words(_alloc)
  , confidence_values(_alloc)
  {
  }

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _words_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  words;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _confidence_values_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  confidence_values;


  typedef boost::shared_ptr< ::nao_msgs::WordRecognized_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::WordRecognized_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct WordRecognized
typedef  ::nao_msgs::WordRecognized_<std::allocator<void> > WordRecognized;

typedef boost::shared_ptr< ::nao_msgs::WordRecognized> WordRecognizedPtr;
typedef boost::shared_ptr< ::nao_msgs::WordRecognized const> WordRecognizedConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::nao_msgs::WordRecognized_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::nao_msgs::WordRecognized_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace nao_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::WordRecognized_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_msgs::WordRecognized_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::WordRecognized_<ContainerAllocator> > {
  static const char* value() 
  {
    return "29134437cd61021f75f35f21b72b7eab";
  }

  static const char* value(const  ::nao_msgs::WordRecognized_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x29134437cd61021fULL;
  static const uint64_t static_value2 = 0x75f35f21b72b7eabULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::WordRecognized_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_msgs/WordRecognized";
  }

  static const char* value(const  ::nao_msgs::WordRecognized_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::WordRecognized_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# Message emitted by Nao speech recognition.\n\
# It contains the list of words recognized and confidence values\n\
# Both arrays are of the same length\n\
string[] words\n\
float32[] confidence_values\n\
\n\
";
  }

  static const char* value(const  ::nao_msgs::WordRecognized_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_msgs::WordRecognized_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.words);
    stream.next(m.confidence_values);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct WordRecognized_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nao_msgs::WordRecognized_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::nao_msgs::WordRecognized_<ContainerAllocator> & v) 
  {
    s << indent << "words[]" << std::endl;
    for (size_t i = 0; i < v.words.size(); ++i)
    {
      s << indent << "  words[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.words[i]);
    }
    s << indent << "confidence_values[]" << std::endl;
    for (size_t i = 0; i < v.confidence_values.size(); ++i)
    {
      s << indent << "  confidence_values[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.confidence_values[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // NAO_MSGS_MESSAGE_WORDRECOGNIZED_H

