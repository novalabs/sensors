/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <sensors/LSM303D.hpp>
#include <Module.hpp>

#ifdef USE_SENSOR_LSM303D

#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include <functional>

#define LSM303D_TEMP_OUT_L      0x05
#define LSM303D_TEMP_OUT_H      0x06
#define LSM303D_STATUS_M      0x07
#define LSM303D_OUT_X_L_M     0x08
#define LSM303D_OUT_X_H_M     0x09
#define LSM303D_OUT_Y_L_M     0x0A
#define LSM303D_OUT_Y_H_M     0x0B
#define LSM303D_OUT_Z_L_M     0x0C
#define LSM303D_OUT_Z_H_M     0x0D
#define LSM303D_WHO_AM_I      0x0F
#define LSM303D_CTRL_M        0x12
#define LSM303D_INT_SRC_M     0x13
#define LSM303D_THS_L_M       0x14
#define LSM303D_THS_H_M       0x15
#define LSM303D_OFFSET_X_L_M    0x16
#define LSM303D_OFFSET_X_H_M    0x17
#define LSM303D_OFFSET_Y_L_M    0x18
#define LSM303D_OFFSET_Y_H_M    0x19
#define LSM303D_OFFSET_Z_L_M    0x1A
#define LSM303D_OFFSET_Z_H_M    0x1B
#define LSM303D_REFERENCE_X     0x1C
#define LSM303D_REFERENCE_Y     0x1D
#define LSM303D_REFERENCE_Z     0x1E
#define LSM303D_CTRL0       0x1F
#define LSM303D_CTRL1       0x20
#define LSM303D_CTRL2       0x21
#define LSM303D_CTRL3       0x22
#define LSM303D_CTRL4       0x23
#define LSM303D_CTRL5       0x24
#define LSM303D_CTRL6       0x25
#define LSM303D_CTRL7       0x26
#define LSM303D_STATUS_A      0x27
#define LSM303D_OUT_X_L_A     0x28
#define LSM303D_OUT_X_H_A     0x29
#define LSM303D_OUT_Y_L_A     0x2A
#define LSM303D_OUT_Y_H_A     0x2B
#define LSM303D_OUT_Z_L_A     0x2C
#define LSM303D_OUT_Z_H_A     0x2D
#define LSM303D_FIFO_CTRL     0x2E
#define LSM303D_FIFO_SRC      0x2F
#define LSM303D_IG_CFG1       0x30
#define LSM303D_IG_SRC1       0x31
#define LSM303D_IG_THS1       0x32
#define LSM303D_IG_DUR1       0x33
#define LSM303D_IG_CFG2       0x34
#define LSM303D_IG_SRC2       0x35
#define LSM303D_IG_THS2       0x36
#define LSM303D_IG_DUR2       0x37
#define LSM303D_CLICK_CFG     0x38
#define LSM303D_CLICK_SRC     0x39
#define LSM303D_CLICK_THS     0x3A
#define LSM303D_TIME_LIMIT      0x3B
#define LSM303D_TIME_LATENCY    0x3C
#define LSM303D_TIME_WINDOW     0x3D
#define LSM303D_ACT_THS       0x3E
#define LSM303D_ACT_DUR       0x3F

#define LSM303D_ACC_DATA_READY    123
#define LSM303D_MAG_DATA_READY    456


namespace sensors {
   LSM303D::LSM303D(
      Core::HW::SPIDevice&  spi,
      Core::HW::EXTChannel& extA,
      Core::HW::EXTChannel& extM
   ) : _spi(spi), _extA(extA), _extM(extM) {}

   LSM303D::~LSM303D()
   {}

   uint8_t
   LSM303D::readRegister(
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
   LSM303D::writeRegister(
      uint8_t reg,
      uint8_t value
   )
   {
      uint8_t txbuf[2];

      switch (reg) {
        case LSM303D_TEMP_OUT_L:
        case LSM303D_TEMP_OUT_H:
        case LSM303D_STATUS_M:
        case LSM303D_OUT_X_L_M:
        case LSM303D_OUT_X_H_M:
        case LSM303D_OUT_Y_L_M:
        case LSM303D_OUT_Y_H_M:
        case LSM303D_OUT_Z_L_M:
        case LSM303D_OUT_Z_H_M:
        case LSM303D_WHO_AM_I:
        case LSM303D_INT_SRC_M:
        case LSM303D_STATUS_A:
        case LSM303D_OUT_X_L_A:
        case LSM303D_OUT_X_H_A:
        case LSM303D_OUT_Y_L_A:
        case LSM303D_OUT_Y_H_A:
        case LSM303D_OUT_Z_L_A:
        case LSM303D_OUT_Z_H_A:
        case LSM303D_FIFO_SRC:
        case LSM303D_IG_SRC1:
        case LSM303D_IG_SRC2:
        case LSM303D_CLICK_SRC:
           /* Read only registers cannot be written, the command is ignored.*/
           return;

        case LSM303D_CTRL_M:
        case LSM303D_THS_L_M:
        case LSM303D_THS_H_M:
        case LSM303D_OFFSET_X_L_M:
        case LSM303D_OFFSET_X_H_M:
        case LSM303D_OFFSET_Y_L_M:
        case LSM303D_OFFSET_Y_H_M:
        case LSM303D_OFFSET_Z_L_M:
        case LSM303D_OFFSET_Z_H_M:
        case LSM303D_REFERENCE_X:
        case LSM303D_REFERENCE_Y:
        case LSM303D_REFERENCE_Z:
        case LSM303D_CTRL0:
        case LSM303D_CTRL1:
        case LSM303D_CTRL2:
        case LSM303D_CTRL3:
        case LSM303D_CTRL4:
        case LSM303D_CTRL5:
        case LSM303D_CTRL6:
        case LSM303D_CTRL7:
        case LSM303D_FIFO_CTRL:
        case LSM303D_IG_CFG1:
        case LSM303D_IG_THS1:
        case LSM303D_IG_DUR1:
        case LSM303D_IG_CFG2:
        case LSM303D_IG_THS2:
        case LSM303D_IG_DUR2:
        case LSM303D_CLICK_CFG:
        case LSM303D_CLICK_THS:
        case LSM303D_TIME_LIMIT:
        case LSM303D_TIME_LATENCY:
        case LSM303D_TIME_WINDOW:
        case LSM303D_ACT_THS:
        case LSM303D_ACT_DUR:
           _spi.select();
           txbuf[0] = reg;
           txbuf[1] = value;
           _spi.send(2, txbuf);
           _spi.deselect();
           break;
        default:
           /* Reserved register must not be written, according to the datasheet
              this could permanently damage the device.*/
           chDbgAssert(FALSE, "lsm303dWriteRegister(), #1 reserved register");
           break;
      } // switch
   } // lsm303d::writeRegister

   bool
   LSM303D::probe()
   {
      uint8_t who_am_i;

      Core::MW::Thread::sleep(Core::MW::Time::ms(500));

      // FIXME: interrupt lines will be rearranged!
      _spi.acquireBus();
      who_am_i = readRegister(LSM303D_WHO_AM_I);

      if (who_am_i != 0x49) {
         _spi.releaseBus();
         return false;
      }

      writeRegister(LSM303D_CTRL0, 0x00);
      writeRegister(LSM303D_CTRL2, 0x40); // 194Hz anti-alias filter BW
      writeRegister(LSM303D_CTRL3, 0x04); // Acc data-ready on INT1
      writeRegister(LSM303D_CTRL4, 0x04); // Mag data-ready on INT2
      writeRegister(LSM303D_CTRL5, 0xF4); // Temp sensor enabled, magnetometer high-res, Mag 100Hz
      writeRegister(LSM303D_CTRL6, 0x00); // Mag FS ±2 gauss
      writeRegister(LSM303D_CTRL7, 0x00);
      writeRegister(LSM303D_CTRL1, 0x67); // Acc 100Hz, axes enabled, no data update until MSB and LSB read
      _spi.releaseBus();

      Core::MW::Thread::sleep(Core::MW::Time::ms(250));

      return true;
   } // lsm303d::init

   LSM303D_Acc::LSM303D_Acc(
      LSM303D& device
   ) : _runner(nullptr), _device(device) {}

   LSM303D_Acc::~LSM303D_Acc()
   {}

   bool
   LSM303D_Acc::init()
   {
      _device._extA.setCallback([this]() {
         chSysLockFromISR();

         if (_runner != nullptr) {
            _timestamp.now();
            Core::MW::Thread::wake(*(this->_runner), LSM303D_ACC_DATA_READY);
            _runner = nullptr;
         }

         chSysUnlockFromISR();
      });

      return true;
   } // lsm303d::init

   bool
   LSM303D_Acc::start()
   {
      _device._extA.enable();
      update();

      return true;
   }

   bool
   LSM303D_Acc::stop()
   {
      _device._extA.disable();
      Core::MW::Thread::wake(*(this->_runner), 0); // Wake up consumer thread...

      return true;
   }

   bool
   LSM303D_Acc::update()
   {
      uint8_t  txbuf;
      uint8_t  rxbuf[6];
      int16_t* tmp = (int16_t*)(&rxbuf[0]);

      txbuf = 0x80 | 0x40 | LSM303D_OUT_X_L_A;
      _device._spi.acquireBus();
      _device._spi.select();
      _device._spi.send(1, &txbuf);
      _device._spi.receive(6, &rxbuf);
      /*
         tmp[0]  = (_device.readRegister(LSM303D_OUT_X_H_A) & 0xFF) << 8;
         tmp[0] |= _device.readRegister(LSM303D_OUT_X_L_A) & 0xFF;
         tmp[1]  = (_device.readRegister(LSM303D_OUT_Y_H_A) & 0xFF) << 8;
         tmp[1] |= _device.readRegister(LSM303D_OUT_Y_L_A) & 0xFF;
         tmp[2]  = (_device.readRegister(LSM303D_OUT_Z_H_A) & 0xFF) << 8;
         tmp[2] |= _device.readRegister(LSM303D_OUT_Z_L_A) & 0xFF;
       */
      _device._spi.deselect();
      _device._spi.releaseBus();

      //data.t = _timestamp; // TODO: Fix
      _data.x = tmp[0] >> 4;
      _data.y = tmp[1] >> 4;
      _data.z = tmp[2] >> 4;

      return true;
   } // lsm303d::update

   void
   LSM303D_Acc::get(
      DataType& data
   )
   {
      data = _data;
   } // lsm303d::get

   bool
   LSM303D_Acc::waitUntilReady()
   {
      chSysLock();
      _runner = &Core::MW::Thread::self();
      Core::MW::Thread::Return msg = Core::MW::Thread::sleep();
      chSysUnlock();

      return msg == LSM303D_ACC_DATA_READY;
   }

   LSM303D_Mag::LSM303D_Mag(
      LSM303D& device
   ) : _runner(nullptr), _device(device) {}

   LSM303D_Mag::~LSM303D_Mag()
   {}

   bool
   LSM303D_Mag::init()
   {
      _device._extM.setCallback([this]() {
         chSysLockFromISR();

         if (_runner != nullptr) {
            _timestamp.now();
            Core::MW::Thread::wake(*(this->_runner), LSM303D_MAG_DATA_READY);
            _runner = nullptr;
         }

         chSysUnlockFromISR();
      });

      return true;
   } // lsm303d::init

   bool
   LSM303D_Mag::start()
   {
      _device._extM.enable();

      return true;
   }

   bool
   LSM303D_Mag::stop()
   {
      _device._extM.disable();
      Core::MW::Thread::wake(*(this->_runner), 0); // Wake up consumer thread...

      return true;
   }

   bool
   LSM303D_Mag::update()
   {
      uint8_t  txbuf;
      uint8_t  rxbuf[6];
      int16_t* tmp = (int16_t*)(&rxbuf[0]);

      txbuf = 0x80 | 0x40 | LSM303D_OUT_X_L_M;
      _device._spi.acquireBus();
      _device._spi.select();
      _device._spi.send(1, &txbuf);
      _device._spi.receive(6, &rxbuf);
      _device._spi.deselect();
      _device._spi.releaseBus();

      //data.t = _timestamp; //TODO: Fix
      _data.x = tmp[0] >> 4;
      _data.y = tmp[1] >> 4;
      _data.z = tmp[2] >> 4;

      return true;
   } // lsm303d::update

   void
   LSM303D_Mag::get(
      DataType& data
   )
   {
      data = _data;
   } // lsm303d::get

   bool
   LSM303D_Mag::waitUntilReady()
   {
      chSysLock();
      _runner = &Core::MW::Thread::self();
      Core::MW::Thread::Return msg = Core::MW::Thread::sleep();
      chSysUnlock();

      return msg == LSM303D_MAG_DATA_READY;
   }
}
#endif // ifdef USE_SENSOR_LSM303D
