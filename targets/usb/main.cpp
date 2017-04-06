#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/led/Subscriber.hpp>
#include <core/led/Publisher.hpp>
#include <core/triskar_kinematics/Forward.hpp>
#include <core/triskar_kinematics/Inverse.hpp>

#include "rosserial.hpp"

// --- BOARD IMPL -------------------------------------------------------------

// --- MISC -------------------------------------------------------------------

// *** DO NOT MOVE ***
Module module;

// --- NODES ------------------------------------------------------------------
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);
core::led::Publisher  led_publisher("led_publisher");
core::triskar_kinematics::Forward forward("forward", core::os::Thread::PriorityEnum::NORMAL);
core::triskar_kinematics::Inverse inverse("inverse", core::os::Thread::PriorityEnum::NORMAL);
rosserial::RosSerialPublisher ros_node("ros", core::os::Thread::PriorityEnum::NORMAL);

/*
 * Application entry point.
 */
extern "C" {
   int main( void )
   {
	  const float wheelRadius = 0.035f;
	  const float centerDistance = 0.100f;

      module.initialize();

      // Led publisher node
      core::led::PublisherConfiguration led_publisher_configuration;
      led_publisher_configuration.topic = "led";
      led_publisher_configuration.led   = 1;
      led_publisher.setConfiguration(led_publisher_configuration);

      // Led subscriber node
      core::led::SubscriberConfiguration led_subscriber_configuration;
      led_subscriber_configuration.topic = "led";
      led_subscriber.setConfiguration(led_subscriber_configuration);

      //Forward kinematics configuration
      core::triskar_kinematics::ForwardConfiguration forward_conf;
      forward_conf.input_0 = "encoder_0";
      forward_conf.input_1 = "encoder_1";
      forward_conf.input_2 = "encoder_2";
      forward_conf.output = "vel";
      forward_conf.wheel_radius = wheelRadius;
      forward_conf.center_distance = centerDistance;
      forward.setConfiguration(forward_conf);

      //Inverse kinematics configuration
      core::triskar_kinematics::InverseConfiguration inverse_conf;
      inverse_conf.output_0 = "speed_0";
      inverse_conf.output_1 = "speed_1";
      inverse_conf.output_2 = "speed_2";
      inverse_conf.velocity_input = "cmd_vel";
      inverse_conf.wheel_radius = wheelRadius;
      inverse_conf.center_distance = centerDistance;
      inverse.setConfiguration(inverse_conf);

      // Add nodes to the node manager (== board)...
      module.add(led_subscriber);
      module.add(led_publisher);
      module.add(forward);
      module.add(inverse);
      module.add(ros_node);

      // ... and let's play!
      module.setup();
      module.run();

      // Is everything going well?
      for (;;) {
         if (!module.isOk()) {
            module.halt("This must not happen!");
         }

         core::os::Thread::sleep(core::os::Time::ms(500));
      }

      return core::os::Thread::OK;
   } // main
}
