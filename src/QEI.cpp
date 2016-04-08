#include <sensors/QEI.hpp>
#include <Module.hpp>

#ifdef USE_SENSOR_QEI

#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include <Core/Utils/Math/Constants.hpp>

namespace sensors {
	QEI::QEI(
			Core::HW::QEI& qei
	) : _qei(qei) {}

	QEI::~QEI()
	{}

	bool
	QEI::probe()
	{
		return true;
	} // QEI::init

	QEI_Delta::QEI_Delta(
			QEI& device
	) : _timestamp(Core::MW::Time::IMMEDIATE), _device(device)
	{
		configuration.period = 20.0f;
		configuration.ticks  = 0.0f;
	}

	QEI_Delta::~QEI_Delta()
	{}

	bool
	QEI_Delta::init()
	{
		CORE_ASSERT(configuration.ticks != 0);
		return true;
	} // QEI::init

	bool
	QEI_Delta::start()
	{
		_device._qei.enable();
		_timestamp = Core::MW::Time::IMMEDIATE;
		return true;
	}

	bool
	QEI_Delta::stop()
	{
		_device._qei.disable();
		return true;
	}

	void
	QEI_Delta::get(
			DataType& data
	)
	{
		data.value = _device._qei.getDelta() / (float)configuration.ticks * (2.0f * Core::Utils::Math::Constants::pi<float>());
	} // QEI::update

	bool
	QEI_Delta::waitUntilReady()
	{
		// FIXME !!!!!
		/*
		   if (_timestamp != Core::MW::Time::IMMEDIATE) {
		   Core::MW::Thread::sleep_until(_timestamp + Core::MW::Time::ms(configuration.period));
		   }

		   _timestamp = Core::MW::Time::now();
		 */

		chThdSleepMilliseconds((uint16_t)configuration.period);

		return true;
	}
}
#endif // ifdef USE_SENSOR_QEI
