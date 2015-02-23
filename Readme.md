Bug Algorithms for ROS
======================

Dependencies:
-------------

`pip install pycsp greenlet`

Running:
--------

`rosrun bugs bug.py bug[0|1|2] destX destY`

where 0, 1, or 2 is the bug algorithm of interest.

Stage should be running before the bug program is deployed.

Known bugs:
-----------

- In some cases, can hit corner of obstacle without detection.
- A case in wall following can lead to the robot sitting in a corner and twitching. A literal corner case, if you will.
