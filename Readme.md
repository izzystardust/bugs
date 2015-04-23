Bug Algorithms for ROS
======================

Dependencies:
-------------

- None

Running:
--------

`rosrun bugs bug.py bug[0|1|2] destX destY`

where 0, 1, or 2 is the bug algorithm of interest.

Stage should be running before the bug program is deployed.

Known bugs:
-----------

- In some cases, can hit corner of obstacle without detection.
- A case in wall following can lead to the robot sitting in a corner and twitching. A literal corner case, if you will.
- Not well tested for other corner cases. Shit's weird, yo.

Analysis:
---------

### Bug0:

Memory: O(1) - at each step, the robot knows only its current state and the sensor readings.
CPU: - at each step, constant time.
Steps: best, O(d), where d is distance. Worst case, nonterminating - can get stuck.

### Bug1:

Memory: O(1)
CPU: at each step, constant time
Steps: best, O(d), worst O(D+1.5*total_perimeter)

### Bug2:

Memory: O(1)
CPU: at each step, constant time
Steps: best, O(d), worst O(D+0.5 * num_obstacles * total_perimeter)

Evaluation:
-----------

The three bug algorithms differ only in how they decide to leave the wall and
return to the path through free space to the goal. To implement this, a single
Bug class was created that contained all of the shared logic of the bug algorithms,
with the main loop:
    while current_location.distance(tx, ty) > delta:
        hit_wall = bug.go_until_obstacle()
        if hit_wall:
            bug.follow_wall()
    print "Arrived at", (tx, ty)

where `follow_wall()` loops until `bug.should_leave_wall()` is true.

Bug0 implements logic to see if the path in the direction of the goal is clear.
Bug1 implements logic to confirm circumnavigation occured and the robot is at the closest point.
Bug2 implements logic to see if the slope of the line to the destination is the same as the slope at impact and the current position is closer.
