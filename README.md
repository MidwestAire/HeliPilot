# HeliPilot Project

HeliPilot is a fork of the ArduPilot project specific to commercially flown unmanned helicopters. Release versioning is based on the year and month of release. HeliPilot provides support only for single-rotor conventional helicopters with focus on stability and extensive testing of the code for reliability. Testing is conducted on microcontrollers running the NuttX RTOS sub-system and autopilots running a linux-based sub-system. Credit is given below to the ArduPilot project's resources, developers and maintainers.

## Building and Working with the code ##

The HeliPilot code repository can be cloned and compiled with normal git tools and waf (included as a submodule) on Linux, Mac and Windows (with Windows Subsystem for Linux). It can also be cloned, built and run the simulator with the Desktop Suite for Linux found in the releases.

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
