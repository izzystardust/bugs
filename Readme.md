Bug0 Algorithm for ROS with Prompts
===================================

Dependencies:
-------------

- None

Running:
--------

`rosrun bugs bug.py destX destY`

The robot will give feedback and prompt the user for decisions on the command
line in certain cases.

Stage should be running before the bug program is deployed.

Known bugs:
-----------

- In some cases, can hit corner of obstacle without detection.
- A case in wall following can lead to the robot sitting in a corner and twitching. A literal corner case, if you will.
- Not well tested for other corner cases. Shit's weird, yo.

Evaluation:
-----------

A full writeup can be found in the documentation folder.
