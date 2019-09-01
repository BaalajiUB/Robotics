# Robotics

#### Colorizing the Prokudin-Gorskii photo collection
Given the R,G and B images sepately, computed the SSD(Sum of Squared Differences) and NCC(Normalized Cross-Correlation) to find the best possible overlap of images to create a clear color image of the R,G & B images.

Programming language: Python

#### Grid Localization using Bayes Filter:
Given the current 3D occupancy grid and the latest motion of the robot, position of the robot is estimated by using the prior state and the latest motion of the robot to compute the Gaussian distribution of the robot's position in the 3D occupancy grid using Bayes theorem in the 3D environment. The estimated path in ploted and is compared with the expected/actual path.

Programming language: Python

#### Laser-Based Perception and Navigation with Obstacle Avoidance:
Implemented Bug2 line following algorithm along with RANSAC algorithm on the laser sensor data to avoid local obstacles and reach the goal position. Visualized the RANSAC algorithm functionality on rviz.

Programming languages and tools: Python, ROS, rviz.

#### A* Planning:
Planned a path around static obstacles using A* algorithm from start to goal position(global plan) using Eucledian distance of the cell to the goal as the cost.

Programming languages and tools: Python, ROS, rviz.

