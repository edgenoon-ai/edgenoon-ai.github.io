---
layout: single
title: "Create your own ROS Service in Python"
author: bartoszptak
excerpt: "Services allow you to create a request/response paradigm. How to make your own Service in ROS Melodic using Python, create your own Service message type and configure the project."
modified: 2020-05-09
tags: [ros-melodic, python, service]
category: [robot-operating-system]
image: "assets/images/posts/2020/05/create-own-ros-service-python/bumper-cars-4390958_640.jpg"
---

Hey! I would like to show you how to make your own Service in ROS Melodic using Python, how to create your own Service message type and configure the project to be able to use it in the implementation. In this example, I use the topic `/map` for collision detection. It comes from another node. The map is of the type [OccupancyGrid](http://docs.ros.org/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html).

[ROS Services wiki says:](http://wiki.ros.org/Services) 
> The publish/ ubscribe model is a very flexible communication paradigm, but its many-to-many one-way transport is not appropriate for RPC request/reply interactions, which are often required in a distributed system. **Request/reply is done via a Service, which is defined by a pair of messages: one for the request and one for the reply.** A providing ROS node offers a service under a string name, and a client calls the service by sending the request message and awaiting the reply. Client libraries usually present this interaction to the programmer as if it were a remote procedure call.

### Project tree
The structure of the project is presented below. All `__init__.py` files are empty. In the root directory, I created the `srv/` folder, which is a place for my own types of Services messages.

```bash
.
├── CMakeLists.txt
├── launch
│   ├── collision_detector.launch
├── package.xml
├── setup.py
├── src
│   ├── __init__.py
│   ├── project_name
│   │   ├── collision_detector.py
│   │   └── __init__.py
└── srv
    ├── Coll.srv
    └── __init__.py

```

### Own service message
Create `Coll.srv` in the `srv/ ` directory. This file is divided into two parts. Above `---` request messages are defined and below are response types. Messages can be of the simple type like `uint32` or `string`. But they can also consist of complex types like `geometry_msgs/Pose`.

```
geometry_msgs/Pose pose
---
std_msgs/Bool coll
```

### Add dependencies
Add the `message_generation` dependency to the `package.xml` file.

```xml
<package>
    ...
    <build_depend>message_generation</build_depend>
</package>
```

### Init python project
Create a `setup.py` file that stores information about Python files in our project. Thanks to this, you don't have to specify all project files in `CMakeLists.txt`.

```python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['project_name'],
    package_dir={'': 'src'}
)

setup(**setup_args)
```

### Set CMake settings
The most difficult part is to properly configure `CMakeLists.txt` to build our message and be visible in the python code. The first step is to search for Python libraries:

```make
find_package(PythonLibs REQUIRED)
```

Then search for the components that are used in the Service message and `message_generation` to compile it.
```make
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    nav_msgs
    message_generation
    )
```

Add `.srv` files - they are located in the `srv/` directory.
```make
add_service_files(FILES
      Coll.srv
    )
```

Initialize python in Catkin and then generate messages. You must specify library dependencies that are used.
```make
catkin_python_setup()

# must be after catkin_python_setup()
generate_messages(DEPENDENCIES 
    std_msgs 
    nav_msgs
    )
```

Finally provide the path to the python setup file.
```make
catkin_install_python(PROGRAMS
    setup.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
```

### Create service driver
The following code presents the contents of the `collision_detector.py` file, which is an implementation of our Service. Comments in the code should explain the construction.

```python
#!/usr/bin/env python3
import rospy

from nav_msgs.msg import OccupancyGrid
# catkin_make automatically created a request and response object
from project_name.srv import Coll, CollResponse


class CollisionDetector:
    def __init__(self):
        self.map = None

        # the map subscriber from other node
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # create a Coll service and define callback
        self.collision_serv = rospy.Service('collision', Coll, self.pose_callback)

    def map_callback(self, occupancy):
        # just assign OccupancyMap
        self.map = occupancy

    def pose_callback(self, pose):
        # read a Pose message
        position = pose.pose.position
        orientation = pose.pose.orientation

        # perform your collision detection function
        is_collision = self.check_collision(self.map, position, orientation)

        # create a response object, set the value and return
        response = CollResponse()
        response.coll.data = is_collision
        return response

if __name__ == "__main__":
    rospy.init_node('collision_detector')
    cd = CollisionDetector()
    rospy.spin()
```

### Admire your results!
If everything is ready, start your nodes and check if the service is visible. Use `rosservice list`:
```bash
/collision
/collision_detector/get_loggers
/collision_detector/set_logger_level
```

Call the collision checking service with:
```bash
rosservice call /collision "pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 
```

Then you will get feedback with the result:
```bash
coll: 
  data: False
```
**Nice, there is no collision!**


