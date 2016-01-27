Tritech Micron ROS Package
==========================

This ROS package configures and communicates with the Tritech Micron sonar.
**This has only been tested on ROS Indigo and Jade over RS232.**

Setting up
----------

You must clone this repository as `tritech_micron` into your catkin workspace:

```bash
git clone https://github.com/mcgill-robotics/ros-tritech-micron.git tritech_micron
```

Dependencies
------------
Before proceeding, make sure to install all the dependencies by running:

```bash
sudo pip install -r requirements.txt
```

Compiling
---------

You **must** compile this package before being able to run it. You can do so
by running:

```bash
catkin_make
```

from the root of your workspace.

Running
-------
To run, simply connect the Tritech Micron sonar over RS232 and launch the
package with:

```bash
roslaunch launch/tritech_micron.launch port:=</path/to/sonar> frame:=<frame_id>
```

`port` and `frame` are run-time ROS launch arguments:
- `port`: Serial port to read from, default: `/dev/sonar`.
- `frame`: `tf` frame to stamp the messages with, default: `robot`.

The package will keep trying to connect to the sonar until it is successful.

The `tritech_micron` node will output to the following ROS topics:
- `~scan`: `PointCloud` message. Scan data of the current heading only.
- `~heading`: `PoseStamped` message. Current heading of the sonar.
- `~config`: `TritechMicronConfig` message. Sonar config published on change.

Configuring
-----------
To configure the Tritech Micron sonar, take a look at the parameters defined
in [Scan.cfg](cfg/Scan.cfg).

These paramaters can also be updated on the fly with ROS `dynamic_reconfigure`
as such:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Visualizing
-----------
The scan data can be conveniently visualized with `rviz`.
Simply, add the `tritech_micron/scan` topic as a `PointCloud` message to the
view and make sure to set the `Decay Time` parameter to the number of seconds
it takes to run a full scan in order to see the full scan at once instead of
only one slice.

If done properly, you should be able to see something like this:
![Tritech Micron scan data](https://cloud.githubusercontent.com/assets/723610/10464518/1f73efda-71b8-11e5-8654-8dc300471692.png)
