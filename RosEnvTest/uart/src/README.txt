
Open a terminal and navigate to your ROS package's msg directory. For example, if your package is named my_package, you can use the following command:
Copy
cd ~/catkin_ws/src/my_package/msg
Replace my_package with the actual name of your package and catkin_ws with your workspace name.

Create a new file called MergedCoordinates.msg in the msg directory using a text editor. For example, you can use the following command to create and open the file in the Nano editor:
Copy
nano MergedCoordinates.msg
In the text editor, add the following content to the MergedCoordinates.msg file:
Copy
float32 x
float32 y
float32 z
This content defines a custom message type with three fields: x, y, and z, each of type float32. You can modify the fields and their types as per your requirements.

Save the file and exit the text editor. In Nano, you can press Ctrl + X, then Y to save the file.
Once the MergedCoordinates.msg file is created, you need to update the package's CMakeLists.txt file to include the message file. Open the CMakeLists.txt file for your package in a text editor.
In the CMakeLists.txt file, locate the find_package(catkin REQUIRED ...) line, and add message_generation to the list of required packages. It should look like this:
cmake
Copy
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
Below the find_package(...) line, add the following lines to specify the messages to be generated:
cmake
Copy
add_message_files(
  FILES
  MergedCoordinates.msg
)
Still in the CMakeLists.txt file, add the generate_messages(...) command after the add_message_files(...) command:
cmake
Copy
generate_messages(
  DEPENDENCIES
  std_msgs
)
Save the CMakeLists.txt file and exit the text editor.
Build your ROS package using catkin_make or your preferred build system. In the terminal, navigate to your workspace root directory and run the build command. For example:
Copy
cd ~/catkin_ws
catkin_make
Replace catkin_ws with your workspace name.
