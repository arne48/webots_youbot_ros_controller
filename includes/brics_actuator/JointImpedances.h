/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-fuerte-cob-common-0.4.5/debian/ros-fuerte-cob-common/opt/ros/fuerte/stacks/cob_common/brics_actuator/msg/JointImpedances.msg */
#ifndef BRICS_ACTUATOR_MESSAGE_JOINTIMPEDANCES_H
#define BRICS_ACTUATOR_MESSAGE_JOINTIMPEDANCES_H
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

#include "brics_actuator/Poison.h"
#include "brics_actuator/JointValue.h"
#include "brics_actuator/JointValue.h"

namespace brics_actuator
{
template <class ContainerAllocator>
struct JointImpedances_ {
  typedef JointImpedances_<ContainerAllocator> Type;

  JointImpedances_()
  : poisonStamp()
  , dampings()
  , stiffnesses()
  {
  }

  JointImpedances_(const ContainerAllocator& _alloc)
  : poisonStamp(_alloc)
  , dampings(_alloc)
  , stiffnesses(_alloc)
  {
  }

  typedef  ::brics_actuator::Poison_<ContainerAllocator>  _poisonStamp_type;
   ::brics_actuator::Poison_<ContainerAllocator>  poisonStamp;

  typedef std::vector< ::brics_actuator::JointValue_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::brics_actuator::JointValue_<ContainerAllocator> >::other >  _dampings_type;
  std::vector< ::brics_actuator::JointValue_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::brics_actuator::JointValue_<ContainerAllocator> >::other >  dampings;

  typedef std::vector< ::brics_actuator::JointValue_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::brics_actuator::JointValue_<ContainerAllocator> >::other >  _stiffnesses_type;
  std::vector< ::brics_actuator::JointValue_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::brics_actuator::JointValue_<ContainerAllocator> >::other >  stiffnesses;


  typedef boost::shared_ptr< ::brics_actuator::JointImpedances_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::brics_actuator::JointImpedances_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct JointImpedances
typedef  ::brics_actuator::JointImpedances_<std::allocator<void> > JointImpedances;

typedef boost::shared_ptr< ::brics_actuator::JointImpedances> JointImpedancesPtr;
typedef boost::shared_ptr< ::brics_actuator::JointImpedances const> JointImpedancesConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::brics_actuator::JointImpedances_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::brics_actuator::JointImpedances_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace brics_actuator

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::brics_actuator::JointImpedances_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::brics_actuator::JointImpedances_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::brics_actuator::JointImpedances_<ContainerAllocator> > {
  static const char* value() 
  {
    return "14fdbb76121290f17525acd99e7242c6";
  }

  static const char* value(const  ::brics_actuator::JointImpedances_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x14fdbb76121290f1ULL;
  static const uint64_t static_value2 = 0x7525acd99e7242c6ULL;
};

template<class ContainerAllocator>
struct DataType< ::brics_actuator::JointImpedances_<ContainerAllocator> > {
  static const char* value() 
  {
    return "brics_actuator/JointImpedances";
  }

  static const char* value(const  ::brics_actuator::JointImpedances_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::brics_actuator::JointImpedances_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Poison poisonStamp\n\
JointValue[] dampings\n\
JointValue[] stiffnesses\n\
\n\
================================================================================\n\
MSG: brics_actuator/Poison\n\
string originator 		# node id\n\
string description 		# encoding still an issue\n\
float32 qos			# reliability of the channel\n\
				# 0..1 where 1 means healthy\n\
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

  static const char* value(const  ::brics_actuator::JointImpedances_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::brics_actuator::JointImpedances_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.poisonStamp);
    stream.next(m.dampings);
    stream.next(m.stiffnesses);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct JointImpedances_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::brics_actuator::JointImpedances_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::brics_actuator::JointImpedances_<ContainerAllocator> & v) 
  {
    s << indent << "poisonStamp: ";
s << std::endl;
    Printer< ::brics_actuator::Poison_<ContainerAllocator> >::stream(s, indent + "  ", v.poisonStamp);
    s << indent << "dampings[]" << std::endl;
    for (size_t i = 0; i < v.dampings.size(); ++i)
    {
      s << indent << "  dampings[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::brics_actuator::JointValue_<ContainerAllocator> >::stream(s, indent + "    ", v.dampings[i]);
    }
    s << indent << "stiffnesses[]" << std::endl;
    for (size_t i = 0; i < v.stiffnesses.size(); ++i)
    {
      s << indent << "  stiffnesses[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::brics_actuator::JointValue_<ContainerAllocator> >::stream(s, indent + "    ", v.stiffnesses[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // BRICS_ACTUATOR_MESSAGE_JOINTIMPEDANCES_H
