Workspace Name : R2 Workspace

articubot_one :

  - This package is made with reference from a youtube series https://www.youtube.com/watch?v=OWeLUSzxMsw&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT.
  - We adjusted the urdf files according to our design, but not the whole robot as we have not added the gripper, its servo , its motor those are remianing

  - Currently it works properly - it launches simulation as well as launches robot and we can control the robot using -- teleop_twist_keyboard

	- In this series, he mentioned one ROS-Arduino Bridge (a github repo), it was intended for the arduino board that we are using along with Jetson Nano, so we have uploaded that code in our arduino uno
	- This bridge code was intended for L289N Motor driver, so we adjusted it for our motor driver - Cytron MDD10A

diffdrive_arduino :
	- This package was directly cloned form git as mentioned in above series and it worked perfectly.
 	(So, actually we have no idea what happend in it)


serial :
	-  This packages is also prebuilt and we cloned it direclty and it worked.

yolov8_ros :
	- This package is made by us - it uses the yolov8 model to detect specified objects like - red ball, blue ball, silo and purple ball
	- It subscribes to the '/image_raw' of our camera and publishes detection message in it there is Class_id, confidence and objects coordinates

autonomy_package 
	- This is currently under development, in it we want to add fucntionality of bot to move autonomusly towards the specified ball and then pick it up and move to silo and drop in it.



Now the things we have TODO - 

- Autonomously navigate bot from start zone to area 3
- move to the ball - pick it up - find correct silo (as ball positions in silo matters) - place the ball in silo - and repeat till interrupted or till winning condition
  	- so for this - we need gripper to work - Problem is - how to connect servo and motor and use them ?
	- what logics to use or algorithms for it to navigate with less collison


