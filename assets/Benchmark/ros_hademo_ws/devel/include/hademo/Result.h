// Generated by gencpp from file hademo/Result.msg
// DO NOT EDIT!


#ifndef HADEMO_MESSAGE_RESULT_H
#define HADEMO_MESSAGE_RESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <hademo/ResultInfo.h>
#include <hademo/ResultInfo.h>
#include <hademo/ResultInfo.h>
#include <hademo/ResultInfo.h>
#include <hademo/ResultInfo.h>
#include <hademo/ResultInfo.h>
#include <hademo/ResultInfo.h>
#include <hademo/ResultInfo.h>
#include <hademo/ResultInfo.h>

namespace hademo
{
template <class ContainerAllocator>
struct Result_
{
  typedef Result_<ContainerAllocator> Type;

  Result_()
    : franka_0()
    , franka_1()
    , franka_2()
    , aliengo_0()
    , aliengo_1()
    , aliengo_2()
    , quadrotor_0()
    , quadrotor_1()
    , quadrotor_2()  {
    }
  Result_(const ContainerAllocator& _alloc)
    : franka_0(_alloc)
    , franka_1(_alloc)
    , franka_2(_alloc)
    , aliengo_0(_alloc)
    , aliengo_1(_alloc)
    , aliengo_2(_alloc)
    , quadrotor_0(_alloc)
    , quadrotor_1(_alloc)
    , quadrotor_2(_alloc)  {
  (void)_alloc;
    }



   typedef  ::hademo::ResultInfo_<ContainerAllocator>  _franka_0_type;
  _franka_0_type franka_0;

   typedef  ::hademo::ResultInfo_<ContainerAllocator>  _franka_1_type;
  _franka_1_type franka_1;

   typedef  ::hademo::ResultInfo_<ContainerAllocator>  _franka_2_type;
  _franka_2_type franka_2;

   typedef  ::hademo::ResultInfo_<ContainerAllocator>  _aliengo_0_type;
  _aliengo_0_type aliengo_0;

   typedef  ::hademo::ResultInfo_<ContainerAllocator>  _aliengo_1_type;
  _aliengo_1_type aliengo_1;

   typedef  ::hademo::ResultInfo_<ContainerAllocator>  _aliengo_2_type;
  _aliengo_2_type aliengo_2;

   typedef  ::hademo::ResultInfo_<ContainerAllocator>  _quadrotor_0_type;
  _quadrotor_0_type quadrotor_0;

   typedef  ::hademo::ResultInfo_<ContainerAllocator>  _quadrotor_1_type;
  _quadrotor_1_type quadrotor_1;

   typedef  ::hademo::ResultInfo_<ContainerAllocator>  _quadrotor_2_type;
  _quadrotor_2_type quadrotor_2;





  typedef boost::shared_ptr< ::hademo::Result_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hademo::Result_<ContainerAllocator> const> ConstPtr;

}; // struct Result_

typedef ::hademo::Result_<std::allocator<void> > Result;

typedef boost::shared_ptr< ::hademo::Result > ResultPtr;
typedef boost::shared_ptr< ::hademo::Result const> ResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hademo::Result_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hademo::Result_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hademo::Result_<ContainerAllocator1> & lhs, const ::hademo::Result_<ContainerAllocator2> & rhs)
{
  return lhs.franka_0 == rhs.franka_0 &&
    lhs.franka_1 == rhs.franka_1 &&
    lhs.franka_2 == rhs.franka_2 &&
    lhs.aliengo_0 == rhs.aliengo_0 &&
    lhs.aliengo_1 == rhs.aliengo_1 &&
    lhs.aliengo_2 == rhs.aliengo_2 &&
    lhs.quadrotor_0 == rhs.quadrotor_0 &&
    lhs.quadrotor_1 == rhs.quadrotor_1 &&
    lhs.quadrotor_2 == rhs.quadrotor_2;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hademo::Result_<ContainerAllocator1> & lhs, const ::hademo::Result_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hademo

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hademo::Result_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hademo::Result_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hademo::Result_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hademo::Result_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hademo::Result_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hademo::Result_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hademo::Result_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6a97bcb6cbe98951369cc3755518f34c";
  }

  static const char* value(const ::hademo::Result_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6a97bcb6cbe98951ULL;
  static const uint64_t static_value2 = 0x369cc3755518f34cULL;
};

template<class ContainerAllocator>
struct DataType< ::hademo::Result_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hademo/Result";
  }

  static const char* value(const ::hademo::Result_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hademo::Result_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hademo/ResultInfo franka_0\n"
"hademo/ResultInfo franka_1\n"
"hademo/ResultInfo franka_2\n"
"hademo/ResultInfo aliengo_0\n"
"hademo/ResultInfo aliengo_1\n"
"hademo/ResultInfo aliengo_2\n"
"hademo/ResultInfo quadrotor_0\n"
"hademo/ResultInfo quadrotor_1\n"
"hademo/ResultInfo quadrotor_2\n"
"\n"
"================================================================================\n"
"MSG: hademo/ResultInfo\n"
"bool has_result\n"
"bool success\n"
"string info\n"
;
  }

  static const char* value(const ::hademo::Result_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hademo::Result_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.franka_0);
      stream.next(m.franka_1);
      stream.next(m.franka_2);
      stream.next(m.aliengo_0);
      stream.next(m.aliengo_1);
      stream.next(m.aliengo_2);
      stream.next(m.quadrotor_0);
      stream.next(m.quadrotor_1);
      stream.next(m.quadrotor_2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Result_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hademo::Result_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hademo::Result_<ContainerAllocator>& v)
  {
    s << indent << "franka_0: ";
    s << std::endl;
    Printer< ::hademo::ResultInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.franka_0);
    s << indent << "franka_1: ";
    s << std::endl;
    Printer< ::hademo::ResultInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.franka_1);
    s << indent << "franka_2: ";
    s << std::endl;
    Printer< ::hademo::ResultInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.franka_2);
    s << indent << "aliengo_0: ";
    s << std::endl;
    Printer< ::hademo::ResultInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.aliengo_0);
    s << indent << "aliengo_1: ";
    s << std::endl;
    Printer< ::hademo::ResultInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.aliengo_1);
    s << indent << "aliengo_2: ";
    s << std::endl;
    Printer< ::hademo::ResultInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.aliengo_2);
    s << indent << "quadrotor_0: ";
    s << std::endl;
    Printer< ::hademo::ResultInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.quadrotor_0);
    s << indent << "quadrotor_1: ";
    s << std::endl;
    Printer< ::hademo::ResultInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.quadrotor_1);
    s << indent << "quadrotor_2: ";
    s << std::endl;
    Printer< ::hademo::ResultInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.quadrotor_2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HADEMO_MESSAGE_RESULT_H
