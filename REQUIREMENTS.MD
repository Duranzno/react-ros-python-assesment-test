### Challenge

**Backend (Python/C++)**

- ROS Node turtlesim moving to a goal, in vertical or horizontal, _in-place rotations_ allowed at corners.
- **Actionlib** server
  - Goal: **Pose2D**
  - Feedback: _progress (up to me)(percentage?)_
  - Result: _finished (up to me)(isFinished boolean?)_
- **Service** to pause/resu,e motion
  - Update requester in case its paused/resumed
- **Parameter** for _speed (treat it as a modifier for the normal speed)_
  - This should be changeable (**dynamic reconfigure)**
- Create Launchfiles for the Project \*\*\*\*

**Frontend (ROSBridge/ROSlibjs)**

- Replace turtlesim window showing turtle pose and drawing
- _"Click to navigate"_ to send navigation goals
- Pause/Resume navigation & Speed parameter

**Extra Points**

- ForbiddenZone functionality on the backend using a JSON file. Show those zones on frontend.
- IntegrationTests with rostest
- Server Module that records and saves pose. List them and play them in a real environment

**Deliverable**

- Documentation for building/executing project and any dependencies needed.

**Personal Extra Points**

- Docker based deployment so the entire process can be setted up with a docker command
- Communicate daily (daily stand ups) and updating the development through a Notion Folder and the trello board
