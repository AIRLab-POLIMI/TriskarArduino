#include "SerialConsole.hpp"

#include "chprintf.h"
#include <stdlib.h>

#include <Module.hpp>

static const char* setpointName = "cmd_vel";
static const char* twist_name = "vel";
static const char* enc_name = "enc";
static const char* proximity_name = "proximity";

namespace serialconsole {

std::function<void(float, float, float)> SerialConsole::consoleCallbackRun;

const ShellCommand SerialConsole::commands[] =
{
		{ "r", SerialConsole::cmd_run },
		/*{ "p", SerialConsole::cmd_proximity },
		{ "e", SerialConsole::cmd_encoder },*/
		{ NULL, NULL }
};

ShellConfig SerialConsole::usb_shell_cfg = { nullptr, commands };

SerialConsole::SerialConsole(const char* name,
		core::os::IOChannel& serial,
		core::os::Thread::Priority priority) :
		serial(serial),
		CoreNode::CoreNode(name, priority)
{
	usb_shelltp = nullptr;

	usb_shell_cfg.sc_channel = (BaseSequentialStream *)serial.rawChannel();

	_workingAreaSize = 1024;
	//twist = false;
	//proximity = false;
	setpoint = false;

	/*for(unsigned int i = 0; i < 3; i++)
		encoder[i] = false;*/

	v_x = 0;
    v_y = 0;
    w_z = 0;
}

void SerialConsole::cmd_run(BaseSequentialStream *chp, int argc, char *argv[])
{
	if (argc != 3)
	{
		chprintf(chp, "Usage: r <v_x> <v_y> <w_z>\r\n");
		return;
	}

	float vx = atof(argv[0]);
	float vy = atof(argv[1]);
	float wz = atof(argv[2]);

	SerialConsole::consoleCallbackRun(vx, vy, wz);

}

/*void SerialConsole::cmd_proximity(BaseSequentialStream *chp, int argc, char *argv[])
{

}

void SerialConsole::cmd_encoder(BaseSequentialStream *chp, int argc, char *argv[])
{

}

bool SerialConsole::twistCallback(const core::triskar_msgs::Velocity& msg,
		void* node)
{
	return true;
}

bool SerialConsole::proximityCallback(const core::sensor_msgs::Proximity& msg,
		void* node)
{

	return true;
}

bool SerialConsole::encoderCallback_0(const core::sensor_msgs::Delta_f32& msg,
		void* node)
{

	return true;
}

bool SerialConsole::encoderCallback_1(const core::sensor_msgs::Delta_f32& msg,
		void* node)
{

	return true;
}

bool SerialConsole::encoderCallback_2(const core::sensor_msgs::Delta_f32& msg,
		void* node)
{

	return true;
}*/


void SerialConsole::setpointCallback(float vx, float vy, float wz)
{
	v_x = vx;
	v_y = vy;
	w_z = wz;

	setpoint = true;
}

bool SerialConsole::onPrepareMW()
{
	consoleCallbackRun = std::bind(&SerialConsole::setpointCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	/*_subscriberTwist.set_callback(twistCallback);
	subscribe(_subscriberTwist, twist_name);

	_subscriberProximity.set_callback(proximityCallback);
	subscribe(_subscriberProximity, proximity_name);

	_subscriberEncoder[0].set_callback(encoderCallback_0);
	subscribe(_subscriberEncoder[0], "encoder_0");

	_subscriberEncoder[1].set_callback(encoderCallback_1);
	subscribe(_subscriberEncoder[1], "encoder_1");

	_subscriberEncoder[2].set_callback(encoderCallback_2);
	subscribe(_subscriberEncoder[2], "encoder_2");*/

	advertise(_cmd_publisher, setpointName);

	return true;
}


bool SerialConsole::onStart()
{
	_stamp = core::os::Time::now();

	return true;
}

bool SerialConsole::onLoop()
{
	core::os::Thread::sleep_until(_stamp + core::os::Time::ms(10));

	if(setpoint)
	{
		core::triskar_msgs::Velocity* msgp;

		if (_cmd_publisher.alloc(msgp))
		{
			msgp->linear[0] = v_x;
			msgp->linear[1] = v_y;
			msgp->angular = w_z;

			_cmd_publisher.publish(*msgp);
		}

		setpoint = false;
	}


	if(this->spin(core::os::Time::ms(1)))
	{
		/*if(twist)
		{
			//chprintf(PORT, "TWIST: %f, %f, %f\n", );
			twist = false;
		}

		if(proximity)
		{
			//chprintf(PORT, "PROX: %f, %f, %f, %f, %f, %f, %f, %f\n", );
			proximity = false;
		}

		if(encoder[0] && encoder[1] && encoder[2])
		{
			//chprintf(PORT, "ENC: %f, %f, %f\n", );

			for(unsigned int i = 0; i < 3; i++)
				encoder[i] = false;
		}*/
	}


	if (!usb_shelltp)
		usb_shelltp = shellCreate(&usb_shell_cfg, 4096, NORMALPRIO - 1);
	else if (chThdTerminatedX(usb_shelltp)) {
		chThdRelease(usb_shelltp); /* Recovers memory of the previous shell.   */
		usb_shelltp = NULL; /* Triggers spawning of a new shell.        */
	}


	_stamp = core::os::Time::now();


	return true;
}

}

