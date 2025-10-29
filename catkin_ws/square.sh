#!/bin/bash
rostopic pub -1 /task_lines std_msgs/String "data: '((0, 0), (0, 6))'"
sleep 0.5

rostopic pub -1 /task_lines std_msgs/String "data: '((0, 6), (-6, 6))'"
sleep 0.5

rostopic pub -1 /task_lines std_msgs/String "data: '((-6, 6), (-6, 0))'"
sleep 0.5

rostopic pub -1 /task_lines std_msgs/String "data: '((-6, 0), (0, 0))'"
sleep 0.5

# triggers execution
rostopic pub -1 /task_lines std_msgs/String "data: ''"
