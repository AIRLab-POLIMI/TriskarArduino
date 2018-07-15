/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>

#include <ModuleConfiguration.hpp>

#include <core/triskar_kinematics/ForwardConfiguration.hpp>
#include <core/triskar_msgs/Velocity.hpp>
#include <core/sensor_msgs/Delta_f32.hpp>

namespace core {
namespace triskar_kinematics {
class Forward:
   public mw::CoreNode,
   public mw::CoreConfigurable<ForwardConfiguration>
{
public:
   Forward(
      const char*          name,
      os::Thread::Priority priority = os::Thread::PriorityEnum::NORMAL
   );
   virtual
   ~Forward();

private:
   bool
   onPrepareMW();

   bool
   onLoop();

private:
   template<int index>
   static bool
   callback(
      const core::sensor_msgs::Delta_f32& msg,
      void*                               context
   )
   {
	   Forward* _this = static_cast<Forward*>(context);

	   _this->_speed[index] = msg.value;

	   return true;
   }



private:
   mw::Subscriber<core::sensor_msgs::Delta_f32, 2>  _subscriber[3];
   mw::Publisher<triskar_msgs::Velocity> _publisher;

   float _speed[3];
};
}
}
