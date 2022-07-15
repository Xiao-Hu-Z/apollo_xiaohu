# Apollo 3.5 Quick Start Guide

The following guide serves as a user manual for launching the Apollo 3.5
software and hardware stack on vehicle.

The Apollo 3.5 Quick Start Guide focuses on new features available in Apollo
3.5. For general Apollo concepts, please refer to
[Apollo 1.0 Quick Start](apollo_1_0_quick_start.md).

## Contents

- [Calibration Guide](#calibration-guide)
- [Hardware and Software Installation](#hardware-and-software-installation)
- [Dreamview Usage Table](#dreamview-usage-table)
- [Onboard Test](#onboard-test)

## Calibration Guide

For the vehicle's onboard testing make sure you have calibrated all the sensors.
For sensor calibration, please refer to
[Apollo 2.0 Sensor Calibration Guide](apollo_2_0_sensor_calibration_guide.md)
before you proceed.

## Hardware and Software Installation

Please refer to
[Apollo 3.5 Hardware and System Installation Guide](apollo_3_5_hardware_system_installation_guide.md)
for the steps to install the hardware components and the system software, as
well as
[Apollo Software Installation Guide](apollo_software_installation_guide.md).

## Dreamview Usage Table

For questions regarding Dreamview icons refer to the
[Dreamview Usage Table](../specs/dreamview_usage_table.md).

## Onboard Test

1. Plug-in an external hard-drive to any available USB port in the host machine.

2. Turn on the vehicle, and then the host machine.

3. Launch Docker Release Container.

4. Launch DreamView.

   Note\: Use your favorite browser to access Dreamview web service in your host
   machine browser with URL <http://localhost:8888>.

   ![launch_dreamview](images/dreamview_2_5.png)

5. Select Mode, Vehicle and Map.

   ![setup_profile](images/dreamview_2_5_setup_profile.png)

   Note\: You'll be required to setup profile before doing anything else. Click
   the dropdown menu to select **Navigation** mode, the HDMap and vehicle you
   want to use. The lists are defined in
   [HMI config file](https://raw.githubusercontent.com/ApolloAuto/apollo/r3.0.0/modules/dreamview/conf/hmi.conf).

   Note\: It's also possible to change the profile on the right panel of the
   HMI, but just remember to click `Reset All` on the top-right corner to
   restart the system.

6. Start the Modules.

   Click the `Setup` button.

   ![start_modules](images/dreamview_2_5_setup.png)

   Go to **Module Controller** tab, check if all modules and hardware are ready.
   (Note\: In your offline environment, the hardware modules such as GPS,
   CANBus, Velodyne, Camera and Radar cannot be brought up.) (Note\: You may
   need to drive around a bit to get a good GPS signal.)

   ![controller](images/dreamview_2_5_module_controller.png)

7. Under `Default Routing` select your desired route.

8. Under Tasks click `Start Auto`. (Note: Be cautious when starting the
   autonomous driving, you should now be in autonomous mode.)

   ![start_auto](images/dreamview_2_5_start_auto.png)

9. After the autonomous testing is complete, under Tasks click `Reset All`,
   close all windows and shutdown the machine.

10. Remove the hard drive.
