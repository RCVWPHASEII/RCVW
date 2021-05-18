# Project Description

Rail Crossing Violation Warning 

Plugins for V2I Hub version 3.3 that implement both Roadside and In-Vehicle operations of RCVW.

# Prerequisites

See system requirements for V2I Hub

# Usage

## Building
```
$ cd RCVW/Plugins/
$ cmake .
$ make
```
This will create a bin directory that contains the plugin executable, as well as a directory for each plugin.  However, a V2I Hub plugin must be packaged in a ZIP file to be installed to a system.  In order to package up any one of the plugins from the v2i-hub directory, do the following:
```
$ ln -s ../bin <PluginName>/bin
$ zip <PluginName>.zip <PluginName>/bin/<PluginName> <PluginName>/manifest.json
```
## Execution
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

# Version History and Retention
Version 1.1 - May, 2021 - RCVW Phase II

# License
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

# Contact Information

Contact Name: Jared Withers (FRA) 
Contact Information: Jared.Withers@dot.gov, 202-493-6014
