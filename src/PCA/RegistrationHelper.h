#ifndef REGISTRATIONHELPER_H
#define REGISTRATIONHELPER_H

#include "MultilinearModel.h"

class RegistrationHelper
{
public:
	enum Resolution
	{
		HIGH_RESOLUTION,
		LOW_RESOLUTION
	};

private:

	Tensor m_lowResTensor;
};

#endif