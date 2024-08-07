// Generated by gencpp from file dynamixel_sdk_examples/SyncSetPosition.msg
// DO NOT EDIT!


#ifndef DYNAMIXEL_SDK_EXAMPLES_MESSAGE_SYNCSETPOSITION_H
#define DYNAMIXEL_SDK_EXAMPLES_MESSAGE_SYNCSETPOSITION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dynamixel_sdk_examples
{
template <class ContainerAllocator>
struct SyncSetPosition_
{
  typedef SyncSetPosition_<ContainerAllocator> Type;

  SyncSetPosition_()
    : id0(0)
    , id1(0)
    , id2(0)
    , position0(0)
    , position1(0)
    , position2(0)  {
    }
  SyncSetPosition_(const ContainerAllocator& _alloc)
    : id0(0)
    , id1(0)
    , id2(0)
    , position0(0)
    , position1(0)
    , position2(0)  {
  (void)_alloc;
    }



   typedef uint8_t _id0_type;
  _id0_type id0;

   typedef uint8_t _id1_type;
  _id1_type id1;

   typedef uint8_t _id2_type;
  _id2_type id2;

   typedef int32_t _position0_type;
  _position0_type position0;

   typedef int32_t _position1_type;
  _position1_type position1;

   typedef int32_t _position2_type;
  _position2_type position2;





  typedef boost::shared_ptr< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> const> ConstPtr;

}; // struct SyncSetPosition_

typedef ::dynamixel_sdk_examples::SyncSetPosition_<std::allocator<void> > SyncSetPosition;

typedef boost::shared_ptr< ::dynamixel_sdk_examples::SyncSetPosition > SyncSetPositionPtr;
typedef boost::shared_ptr< ::dynamixel_sdk_examples::SyncSetPosition const> SyncSetPositionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator1> & lhs, const ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator2> & rhs)
{
  return lhs.id0 == rhs.id0 &&
    lhs.id1 == rhs.id1 &&
    lhs.id2 == rhs.id2 &&
    lhs.position0 == rhs.position0 &&
    lhs.position1 == rhs.position1 &&
    lhs.position2 == rhs.position2;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator1> & lhs, const ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dynamixel_sdk_examples

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1f8428b176ccf88bc7f2c826e9b347e6";
  }

  static const char* value(const ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1f8428b176ccf88bULL;
  static const uint64_t static_value2 = 0xc7f2c826e9b347e6ULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dynamixel_sdk_examples/SyncSetPosition";
  }

  static const char* value(const ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 id0\n"
"uint8 id1\n"
"uint8 id2\n"
"int32 position0\n"
"int32 position1\n"
"int32 position2\n"
;
  }

  static const char* value(const ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id0);
      stream.next(m.id1);
      stream.next(m.id2);
      stream.next(m.position0);
      stream.next(m.position1);
      stream.next(m.position2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SyncSetPosition_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dynamixel_sdk_examples::SyncSetPosition_<ContainerAllocator>& v)
  {
    s << indent << "id0: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id0);
    s << indent << "id1: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id1);
    s << indent << "id2: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id2);
    s << indent << "position0: ";
    Printer<int32_t>::stream(s, indent + "  ", v.position0);
    s << indent << "position1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.position1);
    s << indent << "position2: ";
    Printer<int32_t>::stream(s, indent + "  ", v.position2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DYNAMIXEL_SDK_EXAMPLES_MESSAGE_SYNCSETPOSITION_H
