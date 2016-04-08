#pragma once

#include <Configuration.hpp>

#ifdef USE_SENSOR_LSM303D

#include <Core/HW/SPI.hpp>
#include <Core/HW/EXT.hpp>
#include <Core/MW/CoreSensor.hpp>
#include <Core/MW/Thread.hpp>

namespace sensors {
	class LSM303D
	{
public:
		LSM303D(
				Core::HW::SPIDevice&  spi,
				Core::HW::EXTChannel& extA,
				Core::HW::EXTChannel& extM
		);

		virtual
		~LSM303D();

public:
		bool
		probe();

		uint8_t
		readRegister(
				uint8_t reg
		);

		void
		writeRegister(
				uint8_t reg,
				uint8_t value
		);


public:
		Core::HW::SPIDevice&  _spi;
		Core::HW::EXTChannel& _extA;
		Core::HW::EXTChannel& _extM;
	};

	class LSM303D_Acc:
		public Core::MW::CoreSensor<Configuration::LSM303D_ACC_DATATYPE>
	{
public:
		LSM303D_Acc(
				LSM303D& device
		);

		virtual
		~LSM303D_Acc();

private:
public:
		bool
		init();

		bool
		start();

		bool
		stop();

		bool
		waitUntilReady();

		bool
		update();

		void
		get(
				DataType& data
		);


protected:
		Core::MW::Thread* _runner;
		Core::MW::Time    _timestamp;

private:
		LSM303D& _device;
		Configuration::LSM303D_ACC_DATATYPE _data;
	};

	class LSM303D_Mag:
		public Core::MW::CoreSensor<Configuration::LSM303D_MAG_DATATYPE>
	{
public:
		LSM303D_Mag(
				LSM303D& device
		);

		virtual
		~LSM303D_Mag();

private:
public:
		bool
		init();

		bool
		start();

		bool
		stop();

		bool
		waitUntilReady();

		bool
		update();

		void
		get(
				DataType& data
		);


protected:
		Core::MW::Thread* _runner;
		Core::MW::Time    _timestamp;

private:
		LSM303D& _device;
		Configuration::LSM303D_MAG_DATATYPE _data;
	};
}
#endif // ifdef USE_SENSOR_LSM303D
