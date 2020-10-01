## Initial Basic Movement Algorithm

As the movement must always be vertical or horizontal and the only rotation is that of 90 degrees on corners the robot will calculate its direction on one axis first (horizontal first, chosen at random) and it will translate itself until both the goal and the robot have the same value in that axis, then it will try and rotate 90 degrees and do the same procedure until it reaches its goal

---

### Requirements:

* Rotational movement (of z) must be orthogonal. Meaning possible values of:
    * 0 
    * π/2 or 90
    * π or 180
    * 3 π/2 or 270
* Should only move in vertical or horizontal

### Input Data:

* Goal in a Pose2D format, meaning X, Y and theta (although this could be skipped as the rotation doesn't seem to be totally necesarry)
* Pause/Resume
* Speed Multiplier

---

Posible ways to solve this algorithmn:

* Using the standard passing of messages through /turtleX/cmd_vel topic  
* Implementing TF or other more sofisticated motion planning libraries

I will use a custom (poorly made) algorithm as any solution will have to interact directly with the angular velocities. 
That after every translation it will reset its own rotation to a setted point (in this case 0) to simplify the math involved in the rotation calculation. This should be improved 

## A pseudo-code interpretation of this problem (forcing every initial angle as 0)
1. Get the Goal in X, Y coordinates, and calculate dx and dy (distances in both axis between the goal and the robot )
2. Knowing the current angle change it to make it **0°**
3. If *dx* (we always start with the horizontal translation) is a positive number it is in right angle and it will not change, if it is a negative number it will have a rotation of **180°**, if it is zero it is in a direct line and the next step will be skipped
4. Move horizontally (increasing the x value) until they are in the same place horizontally
5. Knowing the current angle change it to make it **0°**
6. If *dy* is a positive number rotate **90°**, if it is a negative number it will have a rotation of **270°**, if it is zero it is in a direct line and the next step will be skipped
7. Move vertically (increasing the x value) until they are in the same place.
