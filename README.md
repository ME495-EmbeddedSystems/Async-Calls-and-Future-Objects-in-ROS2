# `turtle_control` package
The `turtle_control` package uses the `turtlesim` package to to render waypoints as X's using a turtle pen, and then traverse those waypoints using a motion planner while recording its running error.

***The intention behind doing this project was to gain familiarity with the [`turtlesim`](https://wiki.ros.org/turtlesim) package, making asynchronous calls, and maintaining future objects.***

## Quickstart Guide
1. Use `ros2 launch turtle_control waypoint.launch.xml` to run the `waypoint` node.
2. Call the `/load` service to load waypoints for the turtle to follow.
3. Call the `/toggle` service to start and stop the turtle.
https://private-user-images.githubusercontent.com/59332714/273350260-a06e8d02-6e67-442f-90f8-173628ffdefe.webm?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTEiLCJleHAiOjE3MDIyNzAzNzcsIm5iZiI6MTcwMjI3MDA3NywicGF0aCI6Ii81OTMzMjcxNC8yNzMzNTAyNjAtYTA2ZThkMDItNmU2Ny00NDJmLTkwZjgtMTczNjI4ZmZkZWZlLndlYm0_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBSVdOSllBWDRDU1ZFSDUzQSUyRjIwMjMxMjExJTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDIzMTIxMVQwNDQ3NTdaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT04MzVjMWY1ODVkMThiYjY1OTNkNzBhYWE2ZTk2NjUxM2FmNTBjOWQzYzc5ZjYyMWE0Y2I4MzFkMzcwOTM4Y2JhJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZhY3Rvcl9pZD0wJmtleV9pZD0wJnJlcG9faWQ9MCJ9._Y98wF5Z_bwyWw4S-D0aZKoPyVFk1V9lMrV4f7s8p9I

Author: [Aditya Nair](https://github.com/GogiPuttar)
