/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-fuerte-cob-common-0.4.5/debian/ros-fuerte-cob-common/opt/ros/fuerte/stacks/cob_common/brics_actuator/msg/JointConstraint.msg */
#ifndef BRICS_ACTUATOR_MESSAGE_JOINTCONSTRAINT_H
#define BRICS_ACTUATOR_MESSAGE_JOINTCONSTRAINT_H
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

#include "brics_actuator/JointValue.h"

namespace brics_actuator
{
template <class ContainerAllocator>
struct JointConstraint_ {
  typedef JointConstraint_<ContainerAllocator> Type;

  JointConstraint_()
  : type()
  , value()
  {
  }

  JointConstraint_(const ContainerAllocator& _alloc)
  : type(_alloc)
  , value(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  type;

  typedef  ::brics_actuator::JointValue_<ContainerAllocator>  _value_type;
   ::brics_actuator::JointValue_<ContainerAllocator>  value;


  typedef boost::shared_ptr< ::brics_actuator::JointConstraint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::brics_actuator::JointConstraint_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct JointConstraint
typedef  ::brics_actuator::JointConstraint_<std::allocator<void> > JointConstraint;

typedef boost::shared_ptr< ::brics_actuator::JointConstraint> JointConstraintPtr;
typedef boost::shared_ptr< ::brics_actuator::JointConstraint const> JointConstraintConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::brics_actuator::JointConstraint_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::brics_actuator::JointConstraint_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace brics_actuator

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::brics_actuator::JointConstraint_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::brics_actuator::JointConstraint_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::brics_actuator::JointConstraint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f77db04949b26b64f80564df22f00ecb";
  }

  static const char* value(const  ::brics_actuator::JointConstraint_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf77db04949b26b64ULL;
  static const uint64_t static_value2 = 0xf80564df22f00ecbULL;
};

template<class ContainerAllocator>
struct DataType< ::brics_actuator::JointConstraint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "brics_actuator/JointConstraint";
  }

  static const char* value(const  ::brics_actuator::JointConstraint_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::brics_actuator::JointConstraint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string type  		#smaller, greater, equal or <, >, =\n\
JointValue value\n\
\n\
================================================================================\n\
MSG: brics_actuator/JointValue\n\
time timeStamp 		#time of the data \n\
string joint_uri\n\
string unit 		#if empy expects si units, you can use boost::unit\n\
float64 value\n\
\n\
";
  }

  static const char* value(const  ::brics_actuator::JointConstraint_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::brics_actuator::JointConstraint_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.type);
    stream.next(m.value);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct JointConstraint_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::brics_actuator::JointConstraint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::brics_actuator::JointConstraint_<ContainerAllocator> & v) 
  {
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type);
    s << indent << "value: ";
s << std::endl;
    Printer< ::brics_actuator::JointValue_<ContainerAllocator> >::stream(s, indent + "  ", v.value);
  }
};


} // namespace message_operations
} // namespace ros

#endif // BRICS_ACTUATOR_MESSAGE_JOINTCONSTRAINT_H

