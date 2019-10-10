Open Source Overview
============================
Rail Crossing Violation Warning 
v 1.0
<Application description>
<Primary functions>
<Installation and removal instructions>
Example:
The Software Solution (SS) Software is designed to test different strategies for producing, transmitting, and storing Connected Vehicle information. The SS reads in and uses vehicle trajectory information or Vissim output, roadside equipment (RSE) location information, cellular or event region information and strategy information to produce a series of snapshots that a connected vehicle would produce. Vehicles can be equipped to generate and transmit Probe Data Messages (PDMs), Basic Safety Messages (BSMs), Cooperative Awareness Messages (CAMs) or ITS SPOT messages which can be transmitted by Dedicated Short Range Communication (DSRC) and/or via cellular. The SS program version 2 build 3 or 2.3 includes simulated communication disruptions between vehicles and roadside equipment. As soon as a vehicle equipped to transmit via DSRC is in range of a RSE, it will download all of its snapshot information directly with a probabilistic uncertainty of the data being lost. Similarly, if the vehicle is equipped to transmit via cellular, it will download all its snapshot information directly but those snapshots might be lost or delayed due to user-defined loss rate and latency. In SS 2.3, BSMs and PDMs can also be made to transmit at user-defined intervals.

License information
-------------------
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this
file except in compliance with the License.
You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed under
the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
KIND, either express or implied. See the License for the specific language governing
permissions and limitations under the License.

System Requirements
-------------------------
See system requirements for V2I Hub

Documentation
-------------

Prerequisites

AIOUSB Needs to be downloaded and installed

$ wget http://accesio.com/files/packages/AIOUSB-Latest.tar.gz
$ tar -xvf AIOUSB-Latest.tar.gz
$ cd AIOUSB-1.117.1/AIOUSB
$ mkdir bin
$ cd bin
$ cmake ..
$ make
$ sudo make install
$ sudo apt-get install fxload

Compilation RCVWPlugin

$ cd RCVW/Plugins/RCVWPlugin/Debug
$ cmake ..
$ make package

- This will create a RCVWPlugin-1.0.0.zip that can be installed using the V2I Hub admin portal.

Compilation HRIStatusPlugin
$ cd /RCVW/Plugins/HRIStatusPlugin/Debug
$ cmake ..
$ make package

- This will create a HRIStatusPlugin-1.0.0.zip that can be installed using the V2I Hub admin portal.


Installation
See V2I Hub Sample Setup Guide for complete installation instructions

To install a plugin on V2I Hub
- Login to the V2I Hub admin portal
- Navigate to the Installed Plugins page
- Click the + button at the bottom left of the table to install a new plugin
- If you are updating an existing plugin, click the upload button for the plugin you wish to update
- Click browse and select the plugin zip file and click open, then click add
- This will install a new plugin
- Click the pencil icon and click the check box next to Disabled to enable the plugin.  Click save.
- Disable to edit the Configuration Parameters for the plugin. 
