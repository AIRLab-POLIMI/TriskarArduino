/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <Module.hpp>

#include <core/triskar_kinematics/Forward.hpp>
#include <core/utils/math/Constants.hpp>
#include <core/utils/math/Conversions.hpp>

#include <core/triskar_msgs/Velocity.hpp>
#include <core/triskar_msgs/Speeds.hpp>

#include <cmath>

using namespace core::utils::math::constants;
using namespace std;

namespace core {
namespace triskar_kinematics {
Forward::Forward(
   const char*          name,
   os::Thread::Priority priority
) :
   CoreNode::CoreNode(name, priority),
   CoreConfigurable::CoreConfigurable(name)
{
   _workingAreaSize = 1024;
}

Forward::~Forward()
{
   teardown();
}

bool
Forward::onPrepareMW()
{
   _subscriber[0].set_callback(Forward::callback<0>);
   _subscriber[1].set_callback(Forward::callback<1>);
   _subscriber[2].set_callback(Forward::callback<2>);

   this->subscribe(_subscriber[0], configuration().input_0);
   this->subscribe(_subscriber[1], configuration().input_1);
   this->subscribe(_subscriber[2], configuration().input_2);

   this->advertise(_publisher, configuration().output);

   return true;
}

bool
Forward::onLoop()
{
   triskar_msgs::Velocity* velocity;

   const float R = configuration().wheel_radius;
   const float L = configuration().center_distance;


   float dx = 0.5*R*(_speed[0] - _speed[1])/cos(pi<float>()/6.0);
   float dy = R*(_speed[0]+_speed[1]-2.0*_speed[2])/3.0;
   float dphi = R*(_speed[0]+_speed[1]+_speed[2])/3.0/L;

   if (this->spin(ModuleConfiguration::SUBSCRIBER_SPIN_TIME)) {
      if (_publisher.alloc(velocity)) {

         /// PUBLISH THE RESULTS
         velocity->linear[0] = dx;
         velocity->linear[1] = dy;
         velocity->angular = dphi;

         _publisher.publish(velocity);
      }
   }

   return true;
} // Forward::onLoop

}
}
