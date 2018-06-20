
## Source code for detect objects in Gazebo with Tiago robot

Run the world inside this repository: https://github.com/RaphaBrito/TIAGo/tree/master/install

1. Run Model

		# World with objects on table to detect		
		roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel world:=objects_on_table

		# RoboCIn World
		roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=titanium world:=robocinV3


2. Teleop Tiago to a place with objects to detect

		rosrun key_teleop key_teleop.py

3. Get raw image from Tiago camera (Util for debug)

		rosrun image_view image_view image:=/xtion/rgb/image_raw


4. To open find_object_2d with gui and select the objects to detect
		
		4.1. Go to path roscd find_object_2d/launch and open file, and change the parameter to remap image:	
		<remap from="image" to="/xtion/rgb/image_raw"/>
	
		4.2. Run launch with gui
		roslaunch find_object_2d find_object_2d_gui.launch 

5. To open find_object_2d withouth gui (Load descriptors and detect)

		roslaunch find_object_2d find_object_2d.launch

6. Run detect (Print boundbox in a cv::window)

		rosrun detect detect.py


## Load descriptors

1. Put objects inside
	
		/home/$USER/objects

2. Go to path roscd find_object_2d/launch and open file (to remap image and load descriptors):

		subl find_object_2d.launch
		
		# Change that parameters and save file:
		<remap from="image" to="/xtion/rgb/image_raw"/>
		<param name="gui" value="false" type="bool"/>
		<param name="objects_path" value="~/objects" type="str"/>
