# "Hello World!" for ROS2 written, using classes

## ::::: instructions :::::
- move ros2 packages under _ros2-packages_ to directories with _src/_ under it
- compile with _colcon build_ and activate by running _setup.bash_ under _install_ directory

## ::::: Updates :::::
- [January 20, 2020]
	- added Hello World! for ROS2 packages
	- repository made public

- [February 5, 2020]
	- added publisher ("Hello World!") for ROS2 packages written in C++

- [February 6, 2020]
	- added subscriber for ROS2 package "Hello World!" written in C++
- [June 22, 2020]
	- removed every existed ROS2 package before June 22nd, 2020 and overwrote them with the current packages
		- [further plans]:
			- add C++ version for every packages under _ros2\_packages_ directory written in python3
- [June 28, 2020]
	- added C++ version for the <i>hello\_action\_test\_py3</i> package
		- Note:
			- The C++ version (<i>hello\_action\_test\_cpp</i>) does not contain enough comment to explain the entire code yet. Will be added in further update.
- [June 29, 2020]
	- added C++ version for <i>hello\_test\_pub\_py3</i> and <i>hello\_test\_sub\_py3</i>

- [July 24, 2020]
	- pre-asigned data types of some local variables in <i>send_goal</i> function in action client of hello\_action\_test\_cpp

- [October 3, 2020]
    - Renamed <i>ros2-packages</i> to <i>hello_ws</i>
    - Moved all packages under <i>hello_ws/src</i>
