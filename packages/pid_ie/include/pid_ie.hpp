#ifndef _PID_EI_EI_HPP_
#define _PID_EI_EI_HPP_

#include <float.h>

namespace core
{
namespace pid_ie
{
    
class PID_IE {
private:
	// Parameters
	float _k;
	float _a;
	float _b1;
	float _b2;
	float _min;
	float _max;

	// Status
	float _ui;
	float _ud;
	float _errorOld;
	float _outputOld;

	// Inputs
	bool _auto;
	bool _fHigh;
	bool _fLow;
	float _control;
	float _setpoint;

	// Outputs
	bool _sHigh;
	bool _sLow;
	bool _flagsChanged;

public:
	PID_IE(void);
	void config(float k, float ti, float td, float ts, float N, float min,
			float max);
	void set(float setpoint);
	void reset(void);
	float get_setpoint(void);
	float update(float measure);
	float getLastOutput();

	// methods to set extra inputs
	void setAuto(bool autoMode);
	void setControl(float control);
	void setFreezeHigh(bool fHigh);
	void setFreezeLow(bool fLow);

	// methods to get outputs
	bool flagsChanged();
	bool isSaturatedHigh();
	bool isSaturatedLow();
};

}
}

#endif /* _PID_EI_EI_HPP_ */
