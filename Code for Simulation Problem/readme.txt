Name: Ian Syndergaard
Date: 3/18/2023

This folder includes code and an executable for performing inverse kinematics on a planer 3-link robot.

The code requires the following inputs:
  1. Initial pose (three values representing the x-coordinate, y-coordinate, and orientation of the end effector)
  2. Final pose (three values representing the x-coordinate, y-coordinate, and orientation of the end effector)
  3. Number of desired trajectory points
  4. Duration of trajectory
These inputs must be entered in "inputs.csv" in the order described above with each number being separated by a comma.
The inputs may be on different lines, but the separating comma is still required even if values are on different lines.

The code prints six columns to the console:
  1. Theta1
  2. Theta2
  3. Theta3
  4. Theta1_dot
  5. Theta2_dot
  6. Theta3_dot
where values of theta are the joint angles and have units of radians and values of theta_dot are joint velocities and
have units of radians per unit time (whatever unit was used for the duration of the trajectory - input 4). If a pose
is not reachable, the program gives a warning.
Becuase 6 columns are quite wide, it is recommended to maximize the terminal window.

WARNING: For many reachable poses, there are two solutions to the inverse kinematics (elbow up and elbow down). If both
are valid solutions, the program chooses elbow down. However, near some unreachable poses, only one solution is possible.
If only the elbow up solution is possible, the inverse kinematics will command elbow-up angles. This will result in a
sharp discontinuity in the joint angles (requiring near infinite joint velocity to attain). If implemented on a real
robot, the joint velocities should be carefully examined to avoid such trajectories.

If changes are made, the code can be recompiled by the command:
  gcc Inverse_Kinematics_3_Link_Planer_Robot.c -o Inverse_Kinematics_3_Link_Planer_Robot
The executable can be run by double clicking or using the terminal/powershell command (from the appropriate directory):
  ./Inverse_Kinematics_3_Link_Planer_Robot.exe