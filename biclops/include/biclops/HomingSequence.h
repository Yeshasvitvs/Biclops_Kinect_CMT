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
 * Auto-generated by gensrv_cpp from file /home/yeshi/catkin_ws/src/biclops/srv/HomingSequence.srv
 *
 */


#ifndef BICLOPS_MESSAGE_HOMINGSEQUENCE_H
#define BICLOPS_MESSAGE_HOMINGSEQUENCE_H

#include <ros/service_traits.h>


#include <biclops/HomingSequenceRequest.h>
#include <biclops/HomingSequenceResponse.h>


namespace biclops
{

struct HomingSequence
{

typedef HomingSequenceRequest Request;
typedef HomingSequenceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct HomingSequence
} // namespace biclops


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::biclops::HomingSequence > {
  static const char* value()
  {
    return "1d00cd540af97efeb6b1589112fab63e";
  }

  static const char* value(const ::biclops::HomingSequence&) { return value(); }
};

template<>
struct DataType< ::biclops::HomingSequence > {
  static const char* value()
  {
    return "biclops/HomingSequence";
  }

  static const char* value(const ::biclops::HomingSequence&) { return value(); }
};


// service_traits::MD5Sum< ::biclops::HomingSequenceRequest> should match 
// service_traits::MD5Sum< ::biclops::HomingSequence > 
template<>
struct MD5Sum< ::biclops::HomingSequenceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::biclops::HomingSequence >::value();
  }
  static const char* value(const ::biclops::HomingSequenceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::biclops::HomingSequenceRequest> should match 
// service_traits::DataType< ::biclops::HomingSequence > 
template<>
struct DataType< ::biclops::HomingSequenceRequest>
{
  static const char* value()
  {
    return DataType< ::biclops::HomingSequence >::value();
  }
  static const char* value(const ::biclops::HomingSequenceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::biclops::HomingSequenceResponse> should match 
// service_traits::MD5Sum< ::biclops::HomingSequence > 
template<>
struct MD5Sum< ::biclops::HomingSequenceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::biclops::HomingSequence >::value();
  }
  static const char* value(const ::biclops::HomingSequenceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::biclops::HomingSequenceResponse> should match 
// service_traits::DataType< ::biclops::HomingSequence > 
template<>
struct DataType< ::biclops::HomingSequenceResponse>
{
  static const char* value()
  {
    return DataType< ::biclops::HomingSequence >::value();
  }
  static const char* value(const ::biclops::HomingSequenceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // BICLOPS_MESSAGE_HOMINGSEQUENCE_H
