/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <Configuration.hpp>

#ifdef USE_SENSOR_QEI

#include <Core/HW/QEI.hpp>
#include <Core/MW/CoreSensor.hpp>
#include <Core/MW/Thread.hpp>
#include <sensors/QEI_DeltaConfiguration.hpp>

namespace sensors {
   class QEI
   {
public:
      QEI(
         Core::HW::QEI& qei
      );

      virtual
      ~QEI();

public:
      bool
      probe();


public:
      Core::HW::QEI& _qei;
   };

   class QEI_Delta:
      public Core::MW::CoreSensor<Configuration::QEI_DELTA_DATATYPE>
   {
public:
      QEI_Delta(
         QEI& device
      );

      virtual
      ~QEI_Delta();

public:
      QEI_DeltaConfiguration configuration;

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
      Core::MW::Time _timestamp;

private:
      QEI& _device;
   };
}
#endif // ifdef USE_SENSOR_QEI
