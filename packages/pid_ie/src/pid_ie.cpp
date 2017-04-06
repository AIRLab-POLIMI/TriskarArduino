#include <core/pid_ie/pid_ie.hpp>

namespace core
{

namespace pid_ie
{
    
PID_IE::PID_IE(void) {
	// Parameters
	_k = 0;
	_a = 0;
	_b1 = 0;
	_b2 = 0;
	_min = -FLT_MAX;
	_max = FLT_MAX;

	// Status
	_ui = 0;
	_ud = 0;
	_errorOld = 0;
	_outputOld = 0;

	// Inputs
	_auto = true;
	_fHigh = false;
	_fLow = false;
	_control = 0;
	_setpoint = 0;

	// Outputs
	_sHigh = false;
	_sLow = false;
	_flagsChanged = false;

}

void PID_IE::config(float k, float ti, float td, float ts, float N, float min =
		-FLT_MAX, float max = FLT_MAX) {

	_k = k;
	_a = (ti == 0) ? 0 : k * (ts / ti);
	_b1 = (td == 0) ? 0 : td / (N * ts + td);
	_b2 = (td == 0) ? 0 : k * N * _b1;
	_min = min;
	_max = max;
}

void PID_IE::set(float setpoint) {

	_setpoint = setpoint;
}

void PID_IE::reset(void) {

	// Status
	_ui = 0;
	_ud = 0;
	_errorOld = 0;
	_outputOld = 0;

	// Inputs
	_auto = true;
	_fHigh = false;
	_fLow = false;
	_control = 0.0;
	_setpoint = 0.0;

	// Outputs
	_sHigh = false;
	_sLow = false;
	_flagsChanged = true; //different from init
}

float PID_IE::get_setpoint(void) {
	return _setpoint;
}

float PID_IE::getLastOutput() {
	return _outputOld;
}

float PID_IE::update(float measure) {
	float error;
	float output;

	/* calculate error */
	error = _setpoint - measure;

	/* proportional term */
	float up = _k * error;

	/* derivative term */
	_ud = _b1 * _ud + _b2 * (error - _errorOld);

	/* compute control */
	if (_auto) {
		output = up + _ui + _ud;
	} else {
		output = _control;
	}

	/* saturation filter */
	if (output > _max) {
		output = _max;

		//update flags
		_sHigh = true;
		_flagsChanged = true;

	} else if (output < _min) {
		output = _min;

		//update flags
		_sLow = true;
		_flagsChanged = true;

	} else if ((_fLow && output < _outputOld)
			|| (_fHigh && output > _outputOld)) {
		output = _outputOld;

		//update flags
		_sHigh = _fHigh;
		_sLow = _fLow;
		_flagsChanged = true;
	} else {

		if (_auto) {
			/* integral term - auto */
			_ui = _ui + _a * error;
		} else {
			/* integral term - manual */
			_ui = output - up - _ud;
		}

		// update flags
		_flagsChanged = _sHigh || _sLow;
		_sHigh = false;
		_sLow = false;
	}

	// Update the rest of the state
	_errorOld = error;
	_outputOld = output;

	return output;
}

void PID_IE::setAuto(bool autoMode) {
	_auto = autoMode;
}

void PID_IE::setControl(float control) {
	_control = control;
}

void PID_IE::setFreezeHigh(bool fHigh) {
	_fHigh = fHigh;
}

void PID_IE::setFreezeLow(bool fLow) {
	_fLow = fLow;
}

bool PID_IE::flagsChanged() {
	bool flagsChanged = _flagsChanged;
	_flagsChanged = false;
	return flagsChanged;
}

bool PID_IE::isSaturatedHigh() {
	return _sHigh;
}

bool PID_IE::isSaturatedLow() {
	return _sLow;
} 

}

}
