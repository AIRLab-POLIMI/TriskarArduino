/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */
 
/* USB Module template file
 *
 */

#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- BOARD IMPL -------------------------------------------------------------

// --- MODULE -----------------------------------------------------------------
Module module;

// *** DO NOT MOVE THE CODE ABOVE THIS COMMENT *** //

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/led/Publisher.hpp>
#include <core/led/Subscriber.hpp>

// --- TYPES ------------------------------------------------------------------

// --- CONFIGURATIONS ---------------------------------------------------------
core::led::PublisherConfiguration  led_publisher_configuration_default;
core::led::SubscriberConfiguration led_subscriber_configuration_default;

// --- NODES ------------------------------------------------------------------
core::led::Publisher  led_publisher("led_pub", core::os::Thread::PriorityEnum::LOWEST);
core::led::Subscriber led_subscriber("led_sub", core::os::Thread::PriorityEnum::LOWEST);

// --- DEVICE CONFIGURATION ---------------------------------------------------

// --- MAIN -------------------------------------------------------------------
extern "C" {
    int
    main(
        void
    )
    {
        module.initialize();

        // Device configurations

        // Default configuration
        led_publisher_configuration_default.topic  = "led";
        led_publisher_configuration_default.led    = 1;
        led_subscriber_configuration_default.topic = "led";

        // Add configurable objects to the configuration manager...
        module.configurations().add(led_publisher, led_publisher_configuration_default);
        module.configurations().add(led_subscriber, led_subscriber_configuration_default);

        // ... and load the configuration
        module.configurations().loadFrom(module.configurationStorage());

        // Add nodes to the node manager...
        module.nodes().add(led_publisher);
        module.nodes().add(led_subscriber);

        // ... and let's play!
        module.nodes().setup();
        module.nodes().run();

        // Is everything going well?
        for (;;) {
            if (!module.nodes().areOk()) {
                module.halt("This must not happen!");
            }

            core::os::Thread::sleep(core::os::Time::ms(500));

            // Remember to feed the (watch)dog!
            module.keepAlive();
        }

        return core::os::Thread::OK;
    } // main
}
