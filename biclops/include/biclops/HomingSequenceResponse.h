/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/yeshi/catkin_ws/src/biclops/srv/HomingSequence.srv
 *
 */


#ifndef BICLOPS_MESSAGE_HOMINGSEQUENCERESPONSE_H
#define BICLOPS_MESSAGE_HOMINGSEQUENCERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace biclops
{
template <class ContainerAllocator>
struct HomingSequenceResponse_
{
  typedef HomingSequenceResponse_<ContainerAllocator> Type;

  HomingSequenceResponse_()
    {
    }
  HomingSequenceResponse_(const ContainerAllocator& _alloc)
    {
    }






  typedef boost::shared_ptr< ::biclops::HomingSequenceResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::biclops::HomingSequenceResponse_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct HomingSequenceResponse_

typedef ::biclops::HomingSequenceResponse_<std::allocator<void> > HomingSequenceResponse;

typedef boost::shared_ptr< ::biclops::HomingSequenceResponse > HomingSequenceResponsePtr;
typedef boost::shared_ptr< ::biclops::HomingSequenceResponse const> HomingSequenceResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::biclops::HomingSequenceResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::biclops::HomingSequenceResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace biclops

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/groovy/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::biclops::HomingSequenceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::biclops::HomingSequenceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::biclops::HomingSequenceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::biclops::HomingSequenceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::biclops::HomingSequenceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::biclops::HomingSequenceResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::biclops::HomingSequenceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::biclops::HomingSequenceResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::biclops::HomingSequenceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "biclops/HomingSequenceResponse";
  }

  static const char* value(const ::biclops::HomingSequenceResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::biclops::HomingSequenceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
";
  }

  static const char* value(const ::biclops::HomingSequenceResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::biclops::HomingSequenceResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct HomingSequenceResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::biclops::HomingSequenceResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::biclops::HomingSequenceResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // BICLOPS_MESSAGE_HOMINGSEQUENCERESPONSE_H
