# TO RUN
```bash
roslaunch project1_turtlebot project1.launch
```

To run a path, we have a few example scripts:
```bash
./square.sh
./notopt_square.sh

# Which these scripts in turn just publish tasks to our topic like so:
rostopic pub -1 /task_lines std_msgs/String "data: '((0, 0), (0, 6))'"
sleep 0.5
# Triggers execution
rostopic pub -1 /task_lines std_msgs/String "data: ''"
```

# COMPLETE
- Document the following
    - The world and launch files, translating the xml into English (1.5 to 2 pages in length, roughly 80 characters per line, 50 lines per page)
    - The robot code, as well as why we used our particular reactive architecture, and how the code embodies that architecture (1.5 to 2 pages in length, roughly 80 characters per line, 50 lines per page)
- Accept keyboard movement commands from a human user.
- Also needs mapping, but he lists it separately for whatever reason in the assignment page
- Gazebo map world file
- Implement the following
    - Halt if collision(s) detected by bumper(s).
    - Escape from (roughly) symmetric obstacles within 1ft in front of the robot.
    - Avoid asymmetric obstacles within 1ft in front of the robot.
    - Turn randomly (uniformly sampled within ±15°) after every 1ft of forward movement.
    - Drive forward.
- Map file
