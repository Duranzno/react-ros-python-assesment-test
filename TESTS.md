# Tests
## In order to refactor the existing algorithm and improve it some tests to facilitate the bug catching will have to be done implementing (rostest)[http://wiki.ros.org/rostest]

* Describe Turtlebot Tests
    * it will move to all 4 Corners from Center (10,10) (0,10) (0,0) (10,10)
        * expect *position topic* be equal to *the goal* within the distance tolerance
    * it will stop and then resume when the *Pause Service* is called
        * expect no changes in the *position topic* when is paused
    * it will change the speed modifier with Dynamic Reconfigure on two instance http://wiki.ros.org/rostest/Nodes#paramtest
        * expect to take less time when speed is higher
* Actionlib Server
    * it will publish the **goal** to the *goal topic* http://wiki.ros.org/rostest/Nodes#publishtest
    * it will **preempt** and close when preempeted
    * it will **preempt** and close when takes longer than the max waiting time
    * it will send **feedback** when the *percentage topic* is updated
    * it will suceed when *percentage topic* reaches 100 and send the **result**
---
## Advice
> In general, for both Python unit and integration tests you want to do the following:
> * Create a class that subclasses from unittest.TestCase
> * Have a bunch of methods with the prefix "test_". Each represents a test case
> * Implement your tests in these methods using the assert*() calls (e.g.self.assertTrue)
> * Integration tests are the same except you create a ROS node handle within the test case and start * > publishing/subscribing. You perform checks in the exact same way as you do with unittest
> * For integration tests, you also want to add a "add_rostest" to your CMakeLists.txt file.
> * catkin_make run_tests in your catkin workspace. 
> [mirzashah](https://answers.ros.org/question/94526/any-good-examples-of-ros-unittests/?answer=94561#post-id-94561)

### Levels of Testing

> * Level 1. Library unit test (**unittest**, gtest): A library unit test should test your code sans
> * Level 2. ROS node unit test (**rostest + unittest/gtest**): Node unit tests start up your node and test its external API, i.e. published topics, subscribed topics, and services.
> * Level 3. ROS nodes integration/regression test (rostest + unittest/gtest): Integration tests start up multiple nodes and test that they all work together as expected.
> * Level 4. Functional testing:At this level, you should test the full robot application as a whole 
[The constructism](https://www.theconstructsim.com/how-to-test-your-ros-programs/)

