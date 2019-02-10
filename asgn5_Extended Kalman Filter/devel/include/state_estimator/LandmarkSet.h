// Generated by gencpp from file state_estimator/LandmarkSet.msg
// DO NOT EDIT!


#ifndef STATE_ESTIMATOR_MESSAGE_LANDMARKSET_H
#define STATE_ESTIMATOR_MESSAGE_LANDMARKSET_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <state_estimator/Landmark.h>

namespace state_estimator
{
template <class ContainerAllocator>
struct LandmarkSet_
{
  typedef LandmarkSet_<ContainerAllocator> Type;

  LandmarkSet_()
    : landmarks()  {
    }
  LandmarkSet_(const ContainerAllocator& _alloc)
    : landmarks(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::state_estimator::Landmark_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::state_estimator::Landmark_<ContainerAllocator> >::other >  _landmarks_type;
  _landmarks_type landmarks;





  typedef boost::shared_ptr< ::state_estimator::LandmarkSet_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::state_estimator::LandmarkSet_<ContainerAllocator> const> ConstPtr;

}; // struct LandmarkSet_

typedef ::state_estimator::LandmarkSet_<std::allocator<void> > LandmarkSet;

typedef boost::shared_ptr< ::state_estimator::LandmarkSet > LandmarkSetPtr;
typedef boost::shared_ptr< ::state_estimator::LandmarkSet const> LandmarkSetConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::state_estimator::LandmarkSet_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::state_estimator::LandmarkSet_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace state_estimator

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'state_estimator': ['/home/dian/ros_wkspace_asgn5/src/state_estimator/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::state_estimator::LandmarkSet_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::state_estimator::LandmarkSet_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::state_estimator::LandmarkSet_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::state_estimator::LandmarkSet_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::state_estimator::LandmarkSet_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::state_estimator::LandmarkSet_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::state_estimator::LandmarkSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2e42ef07fd19a8de8b0770685a8090aa";
  }

  static const char* value(const ::state_estimator::LandmarkSet_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2e42ef07fd19a8deULL;
  static const uint64_t static_value2 = 0x8b0770685a8090aaULL;
};

template<class ContainerAllocator>
struct DataType< ::state_estimator::LandmarkSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "state_estimator/LandmarkSet";
  }

  static const char* value(const ::state_estimator::LandmarkSet_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::state_estimator::LandmarkSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "state_estimator/Landmark[] landmarks\n\
\n\
================================================================================\n\
MSG: state_estimator/Landmark\n\
# The x coordinate of this landmark\n\
float64 x\n\
\n\
# The y coordinate of this landmark\n\
float64 y\n\
\n\
";
  }

  static const char* value(const ::state_estimator::LandmarkSet_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::state_estimator::LandmarkSet_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.landmarks);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LandmarkSet_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::state_estimator::LandmarkSet_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::state_estimator::LandmarkSet_<ContainerAllocator>& v)
  {
    s << indent << "landmarks[]" << std::endl;
    for (size_t i = 0; i < v.landmarks.size(); ++i)
    {
      s << indent << "  landmarks[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::state_estimator::Landmark_<ContainerAllocator> >::stream(s, indent + "    ", v.landmarks[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // STATE_ESTIMATOR_MESSAGE_LANDMARKSET_H
