# ros_tiago_soar
Project aims to use the Soar cognitive architecture to control the TIAGo robot using ROS. 

Gazebo is used for simulations.

This code runs on ubuntu 14 because of limitations of ROS/Gazebo/TIAGo. Image to be provided.
ROS Indigo (Gazebo 2.2.3 is installed when following the steps on the wiki)
Python 2.7.6
Soar 9.6.0

The utils folder has the basic code used for interations between python, ROS and Soar. 
python_ros_gazebo.py --> Integration between python, ros and gazebo.
python_soar.py --> Integration between pyton and Soar.

Before running any command that depends on Gazebo, it's necessary to start the environment (in a new terminal) as stated on section [Testing the simulation installation|http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/TiagoSimulation]
```
source /home/user/tiago_public_ws/devel/setup.bash
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel
```

More information on the [ROS and TIAGo wiki|http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation].


Install Soar 9.6.0

```
sudo apt-get install build-essential swig openjdk-7-jdk python-all-dev
wget http://soar.eecs.umich.edu/downloads/SoarSuite/SoarSuite_9.6.0-source.zip
```

Extract SoarSuite_9.6.0-source.zip, go to folder and type:
```
python scons/scons.py all
```

To get the PATH_TO_SOAR go to the folder extract and type in terminal: pwd
After that put the PATH_TO_SOAR in the export, and copy each line and put in ~/.bashrc

```
export LD_LIBRARY_PATH=PATH_TO_SOAR/out:$LD_LIBRARY_PATH
export CPATH=$HOME/SoarSuite/out/include:$CPATH
export SOAR_HOME=$HOME/SoarSuite/out
```

After that type in terminal:

```
source ~/.bashrc
```

If you have any doubt or error to install Soar 9.6.0, to to this link:

[Soar 9.6.0 Building on Linux|https://soar.eecs.umich.edu/articles/articles/building-soar/81-building-on-linux]

ps: Before run, you need to change Soar path in main.py

