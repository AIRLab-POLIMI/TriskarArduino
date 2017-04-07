#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/os/Callback.hpp>

#include <ModuleConfiguration.hpp>

//Core msgs
#include <core/sensor_msgs/Proximity.hpp>
#include <core/sensor_msgs/Delta_f32.hpp>
#include <core/triskar_msgs/Velocity.hpp>

#include "ch.h"
#include "hal.h"
#include <usbcfg.h>

#include "shell.h"

namespace serialconsole {
class SerialConsole: public core::mw::CoreNode {
public:
	SerialConsole(const char* name,
	         core::os::Thread::Priority priority = core::os::Thread::PriorityEnum::NORMAL);

public:
	static void
	cmd_run(BaseSequentialStream *chp, int argc, char *argv[]);

	/*static void
	cmd_proximity(BaseSequentialStream *chp, int argc, char *argv[]);

	static void
	cmd_encoder(BaseSequentialStream *chp, int argc, char *argv[]);

public:
	static bool
	twistCallback(const core::triskar_msgs::Velocity& msg,
				   void* node);

	static bool
	proximityCallback(const core::sensor_msgs::Proximity& msg,
					   void* node);

	static bool
	encoderCallback_0(const core::sensor_msgs::Delta_f32& msg,
					   void* node);

	static bool
	encoderCallback_1(const core::sensor_msgs::Delta_f32& msg,
						   void* node);

	static bool
	encoderCallback_2(const core::sensor_msgs::Delta_f32& msg,
						   void* node);*/
private:
	void
	setpointCallback(float vx, float vy, float wz);

private:
      bool
      onPrepareMW();

      bool
	  onStart();

      bool
      onLoop();

private:
    //Nova Core
    core::mw::Subscriber<core::triskar_msgs::Velocity, 5> _subscriberTwist;
    core::mw::Subscriber<core::sensor_msgs::Proximity, 5> _subscriberProximity;

    core::mw::Subscriber<core::sensor_msgs::Delta_f32, 5> _subscriberEncoder[3];

	core::mw::Publisher<core::triskar_msgs::Velocity> _cmd_publisher;

	bool twist;
	bool proximity;
	bool encoder[3];

	//Shell
	thread_t* usb_shelltp;
	static const ShellCommand commands[];
	static const ShellConfig usb_shell_cfg;

	static std::function<void(float, float, float)> consoleCallbackRun;

	//Loop
	core::os::Time _stamp;

};

}

