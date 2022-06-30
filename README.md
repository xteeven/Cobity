

# Cobity:  A Library Supporting Easy Dynamic Cobot control using Unity

Cobify is a simple Unity 3D Plugin that enables quick robot prototyping for Haptics in VR without using ROS bridges, just plug and play.


## Requirements (Hardware):

- Kinova ultra-lightweight Gen3
- Kinova ultra-lightweight Gen3 Lite

## Not supported yet:

- Universal Robots UR# Series

## Requirements (Software):

- Kinova Kortex API
- Protocol Buffer 

# Usage

The unityPackage includes all the required files to run the code in your project. Just import it and add it to you scene.

![interface](/pictures/UnityInterface.PNG)

Move the robot by introducing values in the target position field. Please be aware that these values represent meters (m).


## Important

Before using the unity world-coordinates please align your real robot with the virtual one, so the measurements from the robot encoder are consistent with the virtual model.

# Build Instructions

Building the library manually is required if you want to alter any of the provided functions. In order to do so, please follow these simple instructions:

Disclaimer: The current project was developed and and tested only using MS Visual Studio 2019 (and consequently Windows).

Download this repository to you computer, make sure that you have correctly installed and built the Kortex Examples

Then, use the GUI (ccmake or cmake-gui) to set the Kortex API path, or specify it on the cmake command line:

>cmake -DKORTEX_DIR:STRING=<PATH_TO_KORTEX_LIBRARY>

Please note that you have to specify the full path until "include" libraries, for example:

>cmake -DKORTEX_DIR:STRING="Users/<USER_NAME>/.conan/data/kortex_api_cpp/<LIB_VERSION>/kortex/stable/package/<PACKAGE_CODE>"

then run

>cmake install

If you have any feature request please contact us via the GitHub issues tracker.

## Citing our work
This work can be cited as follows:
<pre>@inproceedings{villa2022cobity,
title = {Cobity: A Plug-And-Play Toolbox to Deliver Haptics in Virtual Reality},
author = {Villa, Steeven and Mayer, Sven},
url = {https://github.com/xteeven/Cobity},
doi = {10.1145/3543758.3543775},
year = {2022},
booktitle = {Proceedings of Mensch Und Computer 2022},
publisher = {Association for Computing Machinery},
address = {Darmstadt, Germany},
series = {MuC'22}
}
</pre>
