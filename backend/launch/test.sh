nodemon --exec "rostest backend run.launch --text" -w src/backend/nodes/go_actionserver.py -w src/backend/nodes/__tests__/publish_goal_integration_test.py
