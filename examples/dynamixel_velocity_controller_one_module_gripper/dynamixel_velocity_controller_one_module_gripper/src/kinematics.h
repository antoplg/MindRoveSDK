#include <Arduino.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

// Define a struct for the end effector pose
struct EEPose {
    float height;
    float roll;
    float pitch;
};

// Module parameters (in millimeters and radians)
const float fixed_height = 20.0;
const float radius_base = 100.0;
const float length_leg = 60.0;
const float position_leg_1 = 0;  // PI / 6.0 = 30 degrees
const float position_leg_2 = 4.0 * PI / 6.0; // 5.0 * PI / 6.0 = 150 degrees
const float position_leg_3 = 8.0 * PI / 6.0; // 3.0 * PI / 2.0 = 270 degrees


// Function to extract roll, pitch, and height from a 4x4 transformation matrix
EEPose transformationMatrixToRollPitchHeight(const BLA::Matrix<4, 4>& T) {
    // Extract the rotation submatrix (R) from the transformation matrix (T)
    float R31 = T(2, 0); // Element R31
    float R32 = T(2, 1); // Element R32
    float R33 = T(2, 2); // Element R33
    
    // Define variables
    EEPose pose;

    // Calculate pitch
    pose.pitch = asin(-R31); // Check for potential gimbal lock when pitch is +/-90 degrees

    // Calculate roll if cos(pitch) is not too small
    if (cos(pose.pitch) > 1e-6) {
        pose.roll = atan2(R32, R33);
    } else {
        // Handle gimbal lock
        pose.roll = atan2(-T(1, 2), T(1, 1)); // Alternative computation in case of gimbal lock
    }

    // Extract height (translation in the z-axis)
    pose.height = T(2, 3);
    
    return pose;
}

// Function to create a 4x4 transformation matrix from roll, pitch, and height
BLA::Matrix<4, 4> heightRollPitchToTransformationMatrix(float &height, float &roll, float &pitch) {
    // Initialize matrix with zeros
    BLA::Matrix<4, 4> T;
    T.Fill(0); 

    // Define roll and pitch rotation matrix
    BLA::Matrix<3, 3> Rroll = {
        1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll)};

    BLA::Matrix<3, 3> Rpitch = {
        cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch)};

    // Calculate rotation matrix
    BLA::Matrix<3, 3> R = Rpitch * Rroll;

    // Fill in the rotation matrix part
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            T(i, j) = R(i, j);
        }
    }

    // Fill in the translation part (height)
    T(0, 3) = 0;    // No translation in x
    T(1, 3) = 0;    // No translation in y
    T(2, 3) = height;

    // Homogeneous coordinate
    T(3, 3) = 1.0;

    return T;
}

// Function to calculate forward kinematics
EEPose forwardKinematicsOneModule(float motor_angles[3]) {
    // Calculate the waterbomb centers (3D vectors)
    float center_b1[3] = {
        cos(position_leg_1) * (radius_base + length_leg * cos(motor_angles[0])),
        sin(position_leg_1) * (radius_base + length_leg * cos(motor_angles[0])),
        length_leg * sin(motor_angles[0])
    };

    float center_b2[3] = {
        cos(position_leg_2) * (radius_base + length_leg * cos(motor_angles[1])),
        sin(position_leg_2) * (radius_base + length_leg * cos(motor_angles[1])),
        length_leg * sin(motor_angles[1])
    };

    float center_b3[3] = {
        cos(position_leg_3) * (radius_base + length_leg * cos(motor_angles[2])),
        sin(position_leg_3) * (radius_base + length_leg * cos(motor_angles[2])),
        length_leg * sin(motor_angles[2])
    };

    // Calculate the cross product to get the normal vector of the virtual plane
    float normal_vector[3] = {
        (center_b1[1] - center_b2[1]) * (center_b1[2] - center_b3[2]) - (center_b1[2] - center_b2[2]) * (center_b1[1] - center_b3[1]),
        (center_b1[2] - center_b2[2]) * (center_b1[0] - center_b3[0]) - (center_b1[0] - center_b2[0]) * (center_b1[2] - center_b3[2]),
        (center_b1[0] - center_b2[0]) * (center_b1[1] - center_b3[1]) - (center_b1[1] - center_b2[1]) * (center_b1[0] - center_b3[0])
    };

    // Calculate the magnitude of the normal vector
    float norm_normal_vector = sqrt(pow(normal_vector[0], 2) + pow(normal_vector[1], 2) + pow(normal_vector[2], 2));
    
    float normalized_normal_vector[3];
    // Normalize the normal vector
    for (int i = 0; i < 3; i++) {
        normalized_normal_vector[i] = normal_vector[i] / norm_normal_vector;
    }

    // Calculate the distance between the base plane and the virtual plane
    float distance_vp = ((center_b1[0]) * normal_vector[0] +
                         (center_b1[1]) * normal_vector[1] +
                         (center_b1[2]) * normal_vector[2]) / norm_normal_vector;

    // Calculate the position of the end-effector in the base frame
    float position_ee[3];
    for (int i = 0; i < 3; i++) {
        position_ee[i] = 2 * distance_vp * normalized_normal_vector[i];
    }

    // Get orientation of the end-effector (azimuthal angle and polar angle)
    float alpha = atan2(normalized_normal_vector[1], normalized_normal_vector[0]) + PI;
    float beta = 2 * asin(sqrt(pow(normalized_normal_vector[0], 2) + pow(normalized_normal_vector[1], 2)));

    // Transform angles to roll and pitch
    float pitch = asin(cos(alpha) * sin(beta));
    float roll = asin(sin(alpha) * sin(beta) / -cos(pitch));

    // Fill the output struct
    EEPose output;
    output.height = position_ee[2];
    output.roll = roll;
    output.pitch = pitch;

    return output;
}

// Function to calculate forward kinematics of two modules
EEPose forwardKinematicsTwoModules(float joint_angles[3]) {  
    EEPose fk_one_module = forwardKinematicsOneModule(joint_angles);

    float height = fk_one_module.height + fixed_height;

    // Create transformation matrix for the module
    BLA::Matrix<4, 4> T_one_module = heightRollPitchToTransformationMatrix(height, fk_one_module.roll, fk_one_module.pitch);

    // Compute transformation matrix for two modules (modules have same joint angles)
    BLA::Matrix<4, 4> T_two_modules = T_one_module * T_one_module;

    EEPose fk_two_modules = transformationMatrixToRollPitchHeight(T_two_modules);

    return fk_two_modules;
}


// Function to compute the numerical Jacobian
void numericalJacobian(EEPose (*f)(float[3]), float theta[3], BLA::Matrix<3, 3> &jacobian, float delta = 1e-5) {
    int n = 3;
    EEPose f_plus, f_minus;

    for (int i = 0; i < n; i++) {
        float theta_plus[3];
        float theta_minus[3];

        // Copy the original angles and adjust the ith variable
        for (int j = 0; j < n; j++) {
            theta_plus[j] = theta[j];
            theta_minus[j] = theta[j];
        }
        theta_plus[i] += delta;
        theta_minus[i] -= delta;

        // Compute forward and backward outputs
        f_plus = f(theta_plus);
        f_minus = f(theta_minus);

        // Fill the Jacobian
        jacobian(0, i) = (f_plus.height - f_minus.height) / (2 * delta);
        jacobian(1, i) = (f_plus.roll - f_minus.roll) / (2 * delta);
        jacobian(2, i) = (f_plus.pitch - f_minus.pitch) / (2 * delta);
    }
}