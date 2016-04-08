#include <sensors/L3GD20H.hpp>
#include <Module.hpp>

#ifdef USE_SENSOR_L3GD20H

#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include <functional>

#define L3GD20H_WHO_AM_I    0x0F
#define L3GD20H_CTRL1       0x20
#define L3GD20H_CTRL2       0x21
#define L3GD20H_CTRL3       0x22
#define L3GD20H_CTRL4       0x23
#define L3GD20H_CTRL5       0x24
#define L3GD20H_REFERENCE   0x25
#define L3GD20H_OUT_TEMP    0x26
#define L3GD20H_STATUS      0x27
#define L3GD20H_OUT_X_L     0x28
#define L3GD20H_OUT_X_H     0x29
#define L3GD20H_OUT_Y_L     0x2A
#define L3GD20H_OUT_Y_H     0x2B
#define L3GD20H_OUT_Z_L     0x2C
#define L3GD20H_OUT_Z_H     0x2D
#define L3GD20H_FIFO_CTRL   0x2E
#define L3GD20H_FIFO_SRC    0x2F
#define L3GD20H_IG_CFG      0x30
#define L3GD20H_IG_SRC      0x31
#define L3GD20H_IG_TSH_XH   0x32
#define L3GD20H_IG_TSH_XL   0x33
#define L3GD20H_IG_TSH_YH   0x34
#define L3GD20H_IG_TSH_YL   0x35
#define L3GD20H_IG_TSH_ZH   0x36
#define L3GD20H_IG_TSH_ZL   0x37
#define L3GD20H_IG_DURATION 0x38
#define L3GD20H_LOW_ODR     0x39

#define GYRO_DATA_READY 123

namespace sensors {
	L3GD20H::L3GD20H(
			Core::HW::SPIDevice&  spi,
			Core::HW::EXTChannel& ext
	) : _spi(spi), _ext(ext) {}

	L3GD20H_Gyro::L3GD20H_Gyro(
			L3GD20H& device
	) : _runner(nullptr), _device(device) {}

	L3GD20H::~L3GD20H()
	{}

	L3GD20H_Gyro::~L3GD20H_Gyro()
	{}

	bool
	L3GD20H::probe()
	{
		uint8_t who_am_i;

		Core::MW::Thread::sleep(Core::MW::Time::ms(500));

		_spi.acquireBus();
		who_am_i = readRegister(L3GD20H_WHO_AM_I);

		if (who_am_i != 0xD7) {
			_spi.releaseBus();
			return false;
		}

		writeRegister(L3GD20H_CTRL1, 0x7F);
		writeRegister(L3GD20H_CTRL2, 0x00);
		writeRegister(L3GD20H_CTRL3, 0x08);
		writeRegister(L3GD20H_CTRL4, 0x10);
		writeRegister(L3GD20H_CTRL5, 0x00);
		_spi.releaseBus();

		return true;
	} // l3gd20h::init

	bool
	L3GD20H_Gyro::init()
	{
		_device._ext.setCallback([this]() {
			chSysLockFromISR();

			if (_runner != nullptr) {
			   _timestamp.now();
			   Core::MW::Thread::wake(*(this->_runner), GYRO_DATA_READY);
			   _runner = nullptr;
			}

			chSysUnlockFromISR();
		});

		return true;
	} // l3gd20h::init

	bool
	L3GD20H_Gyro::start()
	{
		_device._ext.enable();

		return true;
	}

	bool
	L3GD20H_Gyro::stop()
	{
		_device._ext.disable();
		Core::MW::Thread::wake(*(this->_runner), 0); // Wake up consumer thread...

		return true;
	}

	uint8_t
	L3GD20H::readRegister(
			uint8_t reg
	)
	{
		uint8_t txbuf[2];
		uint8_t rxbuf[2];

		_spi.select();
		txbuf[0] = 0x80 | reg;
		txbuf[1] = 0xff;
		_spi.exchange(2, txbuf, rxbuf);
		_spi.deselect();
		return rxbuf[1];
	}

	void
	L3GD20H::writeRegister(
			uint8_t reg,
			uint8_t value
	)
	{
		uint8_t txbuf[2];

		switch (reg) {
		  case L3GD20H_WHO_AM_I:
		  case L3GD20H_OUT_TEMP:
		  case L3GD20H_STATUS:
		  case L3GD20H_OUT_X_L:
		  case L3GD20H_OUT_X_H:
		  case L3GD20H_OUT_Y_L:
		  case L3GD20H_OUT_Y_H:
		  case L3GD20H_OUT_Z_L:
		  case L3GD20H_OUT_Z_H:
			  /* Read only registers cannot be written, the command is ignored.*/
			  return;

		  case L3GD20H_CTRL1:
		  case L3GD20H_CTRL2:
		  case L3GD20H_CTRL3:
		  case L3GD20H_CTRL4:
		  case L3GD20H_CTRL5:
		  case L3GD20H_REFERENCE:
		  case L3GD20H_FIFO_CTRL:
		  case L3GD20H_FIFO_SRC:
		  case L3GD20H_IG_CFG:
		  case L3GD20H_IG_SRC:
		  case L3GD20H_IG_TSH_XH:
		  case L3GD20H_IG_TSH_XL:
		  case L3GD20H_IG_TSH_YH:
		  case L3GD20H_IG_TSH_YL:
		  case L3GD20H_IG_TSH_ZH:
		  case L3GD20H_IG_TSH_ZL:
		  case L3GD20H_IG_DURATION:
		  case L3GD20H_LOW_ODR:
			  _spi.select();
			  txbuf[0] = reg;
			  txbuf[1] = value;
			  _spi.send(2, txbuf);
			  _spi.deselect();
			  break;
		  default:
			  /* Reserved register must not be written, according to the datasheet
			     this could permanently damage the device.*/
			  chDbgAssert(FALSE, "L3GD20HWriteRegister(), #1 reserved register");
			  break;
		} // switch
	} // l3gd20h::writeRegister

	bool
	L3GD20H_Gyro::update()
	{
		int16_t tmp[3];

		//XXX da fare lettura sequenziale!
		_device._spi.acquireBus();
		tmp[0]  = (_device.readRegister(L3GD20H_OUT_X_H) & 0xFF) << 8;
		tmp[0] |= _device.readRegister(L3GD20H_OUT_X_L) & 0xFF;
		tmp[1]  = (_device.readRegister(L3GD20H_OUT_Y_H) & 0xFF) << 8;
		tmp[1] |= _device.readRegister(L3GD20H_OUT_Y_L) & 0xFF;
		tmp[2]  = (_device.readRegister(L3GD20H_OUT_Z_H) & 0xFF) << 8;
		tmp[2] |= _device.readRegister(L3GD20H_OUT_Z_L) & 0xFF;
		_device._spi.releaseBus();

		//data.t = _timestamp; // TODO: Fix
		_data.x = tmp[0];
		_data.y = tmp[1];
		_data.z = tmp[2];
	} // l3gd20h::update

	void
	L3GD20H_Gyro::get(
			DataType& data
	)
	{
		data = _data;
	} // l3gd20h::get

	bool
	L3GD20H_Gyro::waitUntilReady()
	{
		chSysLock();
		_runner = &Core::MW::Thread::self();
		Core::MW::Thread::Return msg = Core::MW::Thread::sleep();
		chSysUnlock();

		return msg == GYRO_DATA_READY;
	}
}
#endif // ifdef USE_SENSOR_L3GD20H
