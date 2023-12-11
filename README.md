# `turtle_control` package
The `turtle_control` package uses the `turtlesim` package to to render waypoints as X's using a turtle pen, and then traverse those waypoints using a motion planner while recording its running error.

***The intention behind doing this project was to gain familiarity with the [`turtlesim`](https://wiki.ros.org/turtlesim) package, making asynchronous calls, and maintaining future objects.***

## Quickstart Guide
1. Use `ros2 launch turtle_control waypoint.launch.xml` to run the `waypoint` node.
2. Call the `/load` service to load waypoints for the turtle to follow.
3. Call the `/toggle` service to start and stop the turtle.
4. Here is a video of the turtle in action.
[HW1-Video.webm](https://github.com/ME495-EmbeddedSystems/homework1-GogiPuttar/assets/59332714/a06e8d02-6e67-442f-90f8-173628ffdefe)

Author: [Aditya Nair](https://github.com/GogiPuttar)
