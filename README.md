# Tritech Micron ROS Package

[status]: https://dev.mcgillrobotics.com/buildStatus/icon?job=ros-tritech-micron/master
[url]: https://dev.mcgillrobotics.com/job/ros-tritech-micron/job/master
[![status]][url]

This ROS package configures and communicates with the Tritech Micron sonar.
**This has only been tested on ROS Indigo, Jade, and Kinetic over RS232.**

## Setting up

You must clone this repository as `tritech_micron` into your `catkin`
workspace's `src` directory:

```bash
roscd
cd src
git clone https://github.com/mcgill-robotics/ros-tritech-micron.git tritech_micron
```

## Dependencies

Before proceeding, make sure to install all the dependencies by running:

```bash
rosdep update
rosdep install tritech_micron
```

## Compiling

You **must** compile this package before being able to run it. You can do so
by running:

```bash
catkin_make
```

from the root of your workspace.

## Running

To run, simply connect the Tritech Micron sonar over RS232 and launch the
package with:

```bash
roslaunch tritech_micron tritech_micron.launch port:=</path/to/sonar>
```

The following run-time ROS launch arguments are available:

- `port`: Serial port to read from, default: `/dev/sonar`.
- `baudrate`: Serial port baudrate, default: `115200`.
- `frame`: `tf` frame to stamp the messages with, default: `sonar`.

The package will keep trying to connect to the sonar until it is successful.

The `tritech_micron` node will output to the following ROS topics:

- `~scan`: `PointCloud` message. Scan data of the current heading only.
- `~heading`: `PoseStamped` message. Current heading of the sonar.
- `~config`: `TritechMicronConfig` message. Sonar config published on change.

## Configuring

To configure the Tritech Micron sonar, take a look at the parameters defined
in [Scan.cfg](cfg/Scan.cfg).

These paramaters can also be updated on the fly with ROS `dynamic_reconfigure`
as such:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

## Visualizing

The scan data can be conveniently visualized with `rviz`.
Simply, add the `tritech_micron/scan` topic as a `PointCloud` message to the
view and make sure to set the `Decay Time` parameter to the number of seconds
it takes to run a full scan in order to see the full scan at once instead of
only one slice.

If done properly, you should be able to see something like this:
![Tritech Micron scan data](https://cloud.githubusercontent.com/assets/723610/10464518/1f73efda-71b8-11e5-8654-8dc300471692.png)

## Contributing

Contributions are welcome. Simply open an issue or pull request on the matter,
and it will be accepted as long as it does not complicate the code base too
much.

As for style guides, we follow the ROS Python Style Guide for ROS-specifics and
the Google Python Style Guide for everything else.

### Linting

We use [YAPF](https://github.com/google/yapf) for all Python formatting needs.
You can auto-format your changes with the following command:

```bash
yapf --recursive --in-place --parallel .
```

We also use [catkin_lint](https://github.com/fkie/catkin_lint) for all `catkin`
specifics. You can lint your changes as follows:

```bash
catkin lint --explain -W2 .
```

## License

See [LICENSE](LICENSE).

