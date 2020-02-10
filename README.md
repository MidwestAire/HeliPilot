# HeliPilot Project

HeliPilot is a fork of the ArduPilot project specific to commercially flown unmanned helicopters. 
Release versioning is based on the year and month of release. HeliPilot provides support only for
single-rotor conventional helicopters with focus on stability and extensive testing of the code 
for reliability. Testing is conducted on microcontrollers running the NuttX RTOS sub-system and
autopilots running a linux-based sub-system. Credit is given below to the ArduPilot project's 
resources, developers and maintainers.

## Build the Code ##

To set up the build environment and run the simulator change to the HeliPilot directory and run
the build-setup.sh setup script from your terminal with ./build-setup.sh Follow the prompts in the script.

On Unbuntu 18.04 LTS and newer desktop systems, an icon will be created on the desktop to launch the 
build for HeliPilot. All other systems must run the firmware-build.sh script from the Tools/firmare-build
directory in terminal.

## Running the Simulator ##

After cloning the code and running the build-setup the HeliPilot simulator can be run by changing to
the HeliPilot directory and run ./sim in your terminal.

There is a locations.txt file located in your home directory in the .config/helipilot folder. This 
file can be modified to add custom locations for the SIM and have your simulator flight start from 
a custom location with ./sim -l Default. Change Default to the name of your custom location as entered 
into the locations.txt file.

On Unbuntu 18.04 LTS and newer desktop systems, an icon will be created on the desktop to launch the HeliPilot 
simulator. All other systems must run the sim program from the HeliPilot root directory in terminal.

## License ##

The HeliPilot project is compliant with and licenced under the GNU General Public License, version 3.

- [Full Text](https://github.com/ChristopherOlson/HeliPilot/blob/HeliPilot-master/COPYING.txt)

# The ArduPilot project is made up of:

- ArduCopter (or APM:Copter) : [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter), [wiki](http://ardupilot.org/copter/index.html)

- ArduPlane (or APM:Plane) : [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane), [wiki](http://ardupilot.org/plane/index.html)

- ArduRover (or APMrover2) : [code](https://github.com/ArduPilot/ardupilot/tree/master/APMrover2), [wiki](http://ardupilot.org/rover/index.html)

- ArduSub (or APM:Sub) : [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduSub), [wiki](http://ardusub.com/)

- Antenna Tracker : [code](https://github.com/ArduPilot/ardupilot/tree/master/AntennaTracker), [wiki](http://ardupilot.org/antennatracker/index.html)

## Contributors ##

- [Github statistics](https://github.com/ArduPilot/ardupilot/graphs/contributors)

## Maintainers ##

Ardupilot is comprised of several parts, vehicles and boards. The list below
contains the people that regularly contribute to the project and are responsible
for reviewing patches on their specific area. See [CONTRIBUTING.md](.github/CONTRIBUTING.md) for more information.

- [Andrew Tridgell](https://github.com/tridge):
  - ***Vehicle***: Plane, AntennaTracker
  - ***Board***: APM1, APM2, Pixhawk, Pixhawk2, PixRacer
- [Randy Mackay](https://github.com/rmackay9):
  - ***Vehicle***: Copter, AntennaTracker
- [Grant Morphett](https://github.com/gmorph):
  - ***Vehicle***: Rover
- [Tom Pittenger](https://github.com/magicrub):
  - ***Vehicle***: Plane
- [Chris Olson](https://github.com/ChristopherOlson) and [Bill Geyer](https://github.com/bnsgeyer):
  - ***Vehicle***: TradHeli
- [Paul Riseborough](https://github.com/priseborough):
  - ***Subsystem***: AP_NavEKF2
  - ***Subsystem***: AP_NavEKF3
- [Lucas De Marchi](https://github.com/lucasdemarchi):
  - ***Subsystem***: Linux
- [Peter Barker](https://github.com/peterbarker):
  - ***Subsystem***: DataFlash
  - ***Subsystem***: Tools
- [Michael du Breuil](https://github.com/WickedShell):
  - ***Subsystem***: SMBus Batteries
  - ***Subsystem***: GPS
- [Francisco Ferreira](https://github.com/oxinarf):
  - ***Bug Master***
- [Matthias Badaire](https://github.com/badzz):
  - ***Subsystem***: FRSky
- [Eugene Shamaev](https://github.com/EShamaev):
  - ***Subsystem***: CAN bus
  - ***Subsystem***: UAVCAN
- [Víctor Mayoral Vilches](https://github.com/vmayoral):
  - ***Board***: PXF, Erle-Brain 2, PXFmini
- [Mirko Denecke](https://github.com/mirkix):
  - ***Board***: BBBmini, BeagleBone Blue, PocketPilot
- [Georgii Staroselskii](https://github.com/staroselskii):
  - ***Board***: NavIO
- [Emile Castelnuovo](https://github.com/emilecastelnuovo):
  - ***Board***: VRBrain
- [Julien BERAUD](https://github.com/jberaud):
  - ***Board***: Bebop & Bebop 2
- [Matt Lawrence](https://github.com/Pedals2Paddles):
  - ***Vehicle***: 3DR Solo & Solo based vehicles
- [Gustavo José de Sousa](https://github.com/guludo):
  - ***Subsystem***: Build system
- [Craig Elder](https://github.com/CraigElder):
  - ***Administration***: ArduPilot Technical Community Manager
- [Jacob Walser](https://github.com/jaxxzer):
  - ***Vehicle***: Sub
