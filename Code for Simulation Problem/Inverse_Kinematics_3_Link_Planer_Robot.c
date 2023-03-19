/*--------------------------------------------------------------------------//
Name : Ian Syndergaard
Date: 3/18/2023
Description: Take-home problems for Intuitive Surgical Interview. The program
    takes in desired initial and final poses for a planar three-link robot
    (along with the number of desired steps and total time for the move) and
    outputs the joint-space trajectory and velocities. Includes functions for
    inverse kinematics and computing the Jacobian. Requires the accompanying
    "Matrix_Operations.h" header file.
//--------------------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include "Matrix_Operations.h"
#define PI 3.1415926535897932

void inverse_kinematics(double pose[3], double pose_dot[3]);
void jacobian(double J[3][3], double theta[3]);

int main()
{
    double pose_0[3], pose_f[3]; // Initial and final poses
    int N; // Number of points to compute
    double T; // Time to complete the move (in seconds)

    // Stream initial pose, final pose, N, and T from "inputs.csv". The first
    // three inputs are initial x-coordinate, y-coordinate, and orientation.
    // The next three inputs are final x-coordinate, y-coordinate, and
    // orientation. Last two inputs are duration (T) and number of points (N).
    FILE * inputs;
    inputs = fopen("inputs.csv", "r");
    if (inputs == NULL) {
        printf("Error opening file\n");
        return 1;
    }
    fscanf(inputs, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d",
            &pose_0[0], &pose_0[1], &pose_0[2],
            &pose_f[0], &pose_f[1], &pose_f[2],
            &T, &N);
    fclose(inputs);

    // translational & rotational velocities of the end effector
    // calculated to be constant throughout trajectory
    double pose_dot[3] = {  (pose_f[0] - pose_0[0])/T,
                            (pose_f[1] - pose_0[1])/T,
                            (pose_f[2] - pose_0[2])/T};

    // For each data point, the pose of the end effector is computed and fed
    // to the "inverse_kinematics" function (with its derivative).
    double pose[3];
    for (int i=0; i<N; i++) // Loop through each point in the trajectory
    {
        pose[0] = pose_0[0] + i*(pose_f[0]-pose_0[0])/(N-1);
        pose[1] = pose_0[1] + i*(pose_f[1]-pose_0[1])/(N-1);
        pose[2] = pose_0[2] + i*(pose_f[2]-pose_0[2])/(N-1);

        inverse_kinematics(pose, pose_dot);
    }
    
    printf("Press ENTER to finish: ");
    getchar(); // Wait for a keypress to prevent window from closing.
}

void inverse_kinematics(double pose[3], double pose_dot[3])
{
    double x3 = pose[0];    // x coordinate of end effector
    double y3 = pose[1];    // y coordinate of end effector
    double phi = pose[2];   // orientation of end effector

    double theta[3];        // joint angles
    double theta_dot[3];    // joint velocities
    double L[] = {5, 4, 3}; // Link lengths

    // End position of second link
    double x2 = x3 - L[2]*cos(phi);
    double y2 = y3 - L[2]*sin(phi);

    double a = sqrt(x2*x2 + y2*y2); // distance from base to end of second link
    if (a>=1 & a<=9) // Pose is only reachable if 1<=a<=9
    {
        // Calculate joint angles.
        // theta1 and theta2 are calculated using the law of cosines
        theta[1] = PI - acos((L[0]*L[0] + L[1]*L[1] - a*a)/(2*L[0]*L[1]));
        theta[0] = atan2(y2,x2) - acos((a*a + L[0]*L[0] - L[1]*L[1])/(2*a*L[0]));
        theta[2] = phi - theta[0] - theta[1];

        // Use jacobian to find joint velocities (theta_dot = J^-1 * pose_dot)
        double J[3][3];
        jacobian(J,theta);      // Calculates jacobian (J)
        double J_inv[3][3];
        mat3x3_inv(J_inv,J);    // Calculates inverse of jacobian (J_inv)
        mat3x3_vec_mult(theta_dot,J_inv,pose_dot); // (theta_dot = J^-1 * pose_dot)
        
        // Print the three joint angles and three joint velocities at each time step
        printf("theta1: %f\ttheta2: %f\ttheta3: %f\t|", theta[0], theta[1], theta[2]);
        printf("\ttheta1_dot: %f\ttheta2_dot: %f\ttheta3_dot: %f\n", theta_dot[0], theta_dot[1], theta_dot[2]);
    } else // If pose is not reachable
    {
        printf("ERROR - THE POSE IS NOT REACHABLE\n");
    }
}

void jacobian(double J[3][3], double theta[3])
{
    double L[] = {5, 4, 3};
    double t1 = theta[0], t2 = theta[1], t3 = theta[2];

    // Jacobian for a planer 3-link robot. Derivation in  handwritten notes.
    // Rows correspond to x, y, and phi, respectively. Colums correspond
    // to theta1, theta2, and theta3, respectively.
    J[0][0] = -L[0]*sin(t1) - L[1]*sin(t1+t2) - L[2]*sin(t1+t2+t3);
    J[0][1] = -L[1]*sin(t1+t2) - L[2]*sin(t1+t2+t3);
    J[0][2] = -L[2]*sin(t1+t2+t3);
    J[1][0] = L[0]*cos(t1) + L[1]*cos(t1+t2) + L[2]*cos(t1+t2+t3);
    J[1][1] = L[1]*cos(t1+t2) + L[2]*cos(t1+t2+t3);
    J[1][2] = L[2]*cos(t1+t2+t3);
    J[2][0] = 1;
    J[2][1] = 1;
    J[2][2] = 1;
}
