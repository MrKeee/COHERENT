// Generated by gencpp from file hademo/Func_and_Args.msg
// DO NOT EDIT!


#ifndef HADEMO_MESSAGE_FUNC_AND_ARGS_H
#define HADEMO_MESSAGE_FUNC_AND_ARGS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <hademo/Args.h>

namespace hademo
{
template <class ContainerAllocator>
struct Func_and_Args_
{
  typedef Func_and_Args_<ContainerAllocator> Type;

  Func_and_Args_()
    : has_func(false)
    , func_name()
    , args()  {
    }
  Func_and_Args_(const ContainerAllocator& _alloc)
    : has_func(false)
    , func_name(_alloc)
    , args(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _has_func_type;
  _has_func_type has_func;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _func_name_type;
  _func_name_type func_name;

   typedef  ::hademo::Args_<ContainerAllocator>  _args_type;
  _args_type args;





  typedef boost::shared_ptr< ::hademo::Func_and_Args_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hademo::Func_and_Args_<ContainerAllocator> const> ConstPtr;

}; // struct Func_and_Args_

typedef ::hademo::Func_and_Args_<std::allocator<void> > Func_and_Args;

typedef boost::shared_ptr< ::hademo::Func_and_Args > Func_and_ArgsPtr;
typedef boost::shared_ptr< ::hademo::Func_and_Args const> Func_and_ArgsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hademo::Func_and_Args_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hademo::Func_and_Args_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hademo::Func_and_Args_<ContainerAllocator1> & lhs, const ::hademo::Func_and_Args_<ContainerAllocator2> & rhs)
{
  return lhs.has_func == rhs.has_func &&
    lhs.func_name == rhs.func_name &&
    lhs.args == rhs.args;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hademo::Func_and_Args_<ContainerAllocator1> & lhs, const ::hademo::Func_and_Args_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hademo

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hademo::Func_and_Args_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hademo::Func_and_Args_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hademo::Func_and_Args_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hademo::Func_and_Args_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hademo::Func_and_Args_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hademo::Func_and_Args_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hademo::Func_and_Args_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2cf85b88f235eeb7b23bd9784805fafa";
  }

  static const char* value(const ::hademo::Func_and_Args_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2cf85b88f235eeb7ULL;
  static const uint64_t static_value2 = 0xb23bd9784805fafaULL;
};

template<class ContainerAllocator>
struct DataType< ::hademo::Func_and_Args_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hademo/Func_and_Args";
  }

  static const char* value(const ::hademo::Func_and_Args_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hademo::Func_and_Args_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool has_func\n"
"string func_name\n"
"hademo/Args args\n"
"\n"
"================================================================================\n"
"MSG: hademo/Args\n"
"bool has_args\n"
"string attached_prim_path\n"
"std_msgs/Float64MultiArray waypoint_pos\n"
"std_msgs/Float64MultiArray waypoint_ori\n"
"int32 waypoint_ind\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Float64MultiArray\n"
"# Please look at the MultiArrayLayout message definition for\n"
"# documentation on all multiarrays.\n"
"\n"
"MultiArrayLayout  layout        # specification of data layout\n"
"float64[]         data          # array of data\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/MultiArrayLayout\n"
"# The multiarray declares a generic multi-dimensional array of a\n"
"# particular data type.  Dimensions are ordered from outer most\n"
"# to inner most.\n"
"\n"
"MultiArrayDimension[] dim # Array of dimension properties\n"
"uint32 data_offset        # padding elements at front of data\n"
"\n"
"# Accessors should ALWAYS be written in terms of dimension stride\n"
"# and specified outer-most dimension first.\n"
"# \n"
"# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n"
"#\n"
"# A standard, 3-channel 640x480 image with interleaved color channels\n"
"# would be specified as:\n"
"#\n"
"# dim[0].label  = \"height\"\n"
"# dim[0].size   = 480\n"
"# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n"
"# dim[1].label  = \"width\"\n"
"# dim[1].size   = 640\n"
"# dim[1].stride = 3*640 = 1920\n"
"# dim[2].label  = \"channel\"\n"
"# dim[2].size   = 3\n"
"# dim[2].stride = 3\n"
"#\n"
"# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/MultiArrayDimension\n"
"string label   # label of given dimension\n"
"uint32 size    # size of given dimension (in type units)\n"
"uint32 stride  # stride of given dimension\n"
;
  }

  static const char* value(const ::hademo::Func_and_Args_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hademo::Func_and_Args_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.has_func);
      stream.next(m.func_name);
      stream.next(m.args);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Func_and_Args_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hademo::Func_and_Args_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hademo::Func_and_Args_<ContainerAllocator>& v)
  {
    s << indent << "has_func: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.has_func);
    s << indent << "func_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.func_name);
    s << indent << "args: ";
    s << std::endl;
    Printer< ::hademo::Args_<ContainerAllocator> >::stream(s, indent + "  ", v.args);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HADEMO_MESSAGE_FUNC_AND_ARGS_H
