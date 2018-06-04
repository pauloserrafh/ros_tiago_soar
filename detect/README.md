## Source code for detect objects in Gazebo with Tiago robot

Run the world inside this repository: https://github.com/RaphaBrito/TIAGo/tree/master/install

1. Run Model

		roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=titanium world:=robocinV3

2. Teleop Tiago to a place with objects to detect

		rosrun key_teleop key_teleop.py

3. Get raw image from Tiago camera (Util for debug)

		rosrun image_view image_view image:=/xtion/rgb/image_raw
