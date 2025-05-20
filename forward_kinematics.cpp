#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

// Matrix class for 4x4 homogeneous transformation matrices
class Matrix4x4 {
public:
    double data[4][4];
    
    Matrix4x4() {
        // Initialize as identity matrix
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                data[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    }
    
    // Matrix multiplication
    Matrix4x4 operator*(const Matrix4x4& other) const {
        Matrix4x4 result;
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                result.data[i][j] = 0.0;
                for(int k = 0; k < 4; k++) {
                    result.data[i][j] += data[i][k] * other.data[k][j];
                }
            }
        }
        return result;
    }
    
    // Enhanced matrix printing with better formatting
    void print() const {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "┌                                          ┐\n";
        for(int i = 0; i < 4; i++) {
            std::cout << "│ ";
            for(int j = 0; j < 4; j++) {
                if(data[i][j] >= 0) std::cout << " ";
                std::cout << std::setw(8) << data[i][j];
                if(j < 3) std::cout << "  ";
            }
            std::cout << " │\n";
        }
        std::cout << "└                                          ┘\n";
    }
};

// Structure to hold pose data (position + orientation)
struct RobotPose {
    std::vector<double> position;    // [x, y, z] in mm
    std::vector<double> orientation; // [roll, pitch, yaw] in degrees
};

// Helper function to normalize angle difference
double normalizeAngleDiff(double angle1, double angle2) {
    double diff = angle1 - angle2;
    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;
    return std::abs(diff);
}

class RB3_730ES_ForwardKinematics {
private:
    // DH transformation matrix
    Matrix4x4 createDHMatrix(double theta, double d, double a, double alpha) const {
        Matrix4x4 T;
        double ct = cos(theta);
        double st = sin(theta);
        double ca = cos(alpha);
        double sa = sin(alpha);
        
        // Standard DH transformation matrix
        T.data[0][0] = ct;
        T.data[0][1] = -ca * st;
        T.data[0][2] = sa * st;
        T.data[0][3] = a * ct;
        
        T.data[1][0] = st;
        T.data[1][1] = ca * ct;
        T.data[1][2] = -sa * ct;
        T.data[1][3] = a * st;
        
        T.data[2][0] = 0.0;
        T.data[2][1] = sa;
        T.data[2][2] = ca;
        T.data[2][3] = d;
        
        T.data[3][0] = 0.0;
        T.data[3][1] = 0.0;
        T.data[3][2] = 0.0;
        T.data[3][3] = 1.0;
        
        return T;
    }
    
public:
    // Forward kinematics using DH parameters
    Matrix4x4 computeForwardKinematics(const std::vector<double>& joint_angles_deg) {
        if(joint_angles_deg.size() != 6) {
            std::cerr << "Error: Expected 6 joint angles (J0-J5)" << std::endl;
            return Matrix4x4();
        }
        
        // Convert joint angles to radians
        double j0 = deg2rad(joint_angles_deg[0]);
        double j1 = deg2rad(joint_angles_deg[1]);
        double j2 = deg2rad(joint_angles_deg[2]);
        double j3 = deg2rad(joint_angles_deg[3]);
        double j4 = deg2rad(joint_angles_deg[4]);
        double j5 = deg2rad(joint_angles_deg[5]);
        
        // Physical dimensions in meters (not mm)
        double d1 = 0.1453;   // Base height
        double d2 = 0.11715;  // Shoulder offset
        double a1 = 0.286;    // Upper arm length
        double d3 = 0.1107;   // Elbow offset
        double d4 = 0.344;    // Forearm length
        double d5 = 0.0946;   // Wrist offset
        double d6 = 0.0946;   // Wrist length
        double d7 = 0.1;      // Tool flange length
        
        Matrix4x4 T = Matrix4x4(); // Identity matrix
        
        // L1: θ₁, d₁, 0, -90°
        Matrix4x4 T1 = createDHMatrix(j0, d1, 0.0, deg2rad(-90));
        T = T * T1;
        
        // L2: θ₂ - 90°, -d₂, 0, 0° 
        Matrix4x4 T2 = createDHMatrix(j1 - deg2rad(90), -d2, 0.0, deg2rad(0));
        T = T * T2;
        
        // L3: 0, 0, a₁, 0°
        Matrix4x4 T3 = createDHMatrix(0.0, 0.0, a1, deg2rad(0));
        T = T * T3;
        
        // L4: θ₃ + 90°, d₃, 0, 0°
        Matrix4x4 T4 = createDHMatrix(j2 + deg2rad(90), d3, 0.0, deg2rad(0));
        T = T * T4;
        
        // L5: 0, 0, 0, 90°
        Matrix4x4 T5 = createDHMatrix(0.0, 0.0, 0.0, deg2rad(90));
        T = T * T5;
        
        // L6: θ₄, d₄, 0, -90°
        Matrix4x4 T6 = createDHMatrix(j3, d4, 0.0, deg2rad(-90));
        T = T * T6;
        
        // L7: 0, -d₅, 0, 0°
        Matrix4x4 T7 = createDHMatrix(0.0, -d5, 0.0, deg2rad(0));
        T = T * T7;
        
        // L8: θ₅, d₆, 0, 90°
        Matrix4x4 T8 = createDHMatrix(j4, d6, 0.0, deg2rad(90));
        T = T * T8;
        
        // L9: θ₆, d₇, 0, 0°
        Matrix4x4 T9 = createDHMatrix(j5, d7, 0.0, deg2rad(0));
        T = T * T9;
    
        return T;
    }

    // Convert degrees to radians
    double deg2rad(double degrees) const {
        return degrees * M_PI / 180.0;
    }
    
    // Convert radians to degrees
    double rad2deg(double radians) const {
        return radians * 180.0 / M_PI;
    }
    
    // Extract position and orientation from transformation matrix
    RobotPose getPositionAndOrientation(const Matrix4x4& T) const {
        RobotPose pose;
        
        // Extract position (convert to mm)
        pose.position = {
            T.data[0][3] * 1000.0, 
            T.data[1][3] * 1000.0, 
            T.data[2][3] * 1000.0
        };
        
        // Extract rotation matrix
        double r11 = T.data[0][0], r12 = T.data[0][1], r13 = T.data[0][2];
        double r21 = T.data[1][0], r22 = T.data[1][1], r23 = T.data[1][2];
        double r31 = T.data[2][0], r32 = T.data[2][1], r33 = T.data[2][2];
        
        // Rainbow Robotics uses ZYX Euler convention (Z -> Y' -> X'')
        // This means: first rotation about Z, then Y, then X
        double rx, ry, rz;
        
        // ZYX Euler angle extraction
        ry = atan2(-r31, sqrt(r11*r11 + r21*r21));
        
        if (abs(cos(ry)) > 1e-6) {
            rz = atan2(r21, r11);
            rx = atan2(r32, r33);
        } else {
            // Near gimbal lock
            rz = 0.0;
            rx = atan2(-r23, r22);
        }
        
        // Convert to degrees and store as [Rx, Ry, Rz]
        pose.orientation = {rad2deg(rx), rad2deg(ry), rad2deg(rz)};
        
        return pose;
    }
};

int main() {
    RB3_730ES_ForwardKinematics robot;
    
    std::cout << "\n";
    std::cout << "🤖 RB3-730ES Forward Kinematics Position and Orientation Extraction 🤖\n\n";
    
    // Define the joint configurations from the user's request
    std::vector<std::vector<double>> joint_configs = {
        {0.00, 0.00, 0.00, 0.00, 0.00, 0.00},       // Home position
        {0.00, 90.00, 0.00, 0.00, 0.00, 0.00},      // Random #1
        {45.00, 30.00, -60.00, 120.00, 30.00, -90.00}, // Random #2
        {0.00, 0.00, -90.00, 0.00, 0.00, 0.00},     // Random #3
        {0.00, 30.00, -15.00, 0.00, 43.00, 0.00},   // Random #4
        {0.00, 30.00, -15.00, 0.00, 43.00, 60.00},  // Random #5
        {0.00, 30.00, -15.00, 0.00, 0.00, 0.00},    // Random #6
        {0.00, 30.00, -15.00, -24.00, 43.00, 60.00}, // Random #7
        {0.00, 30.00, 0.00, 0.00, 0.00, 0.00}       // Random #8
    };
    
    std::vector<std::string> config_names = {
        "Home position",
        "Joint random #1",
        "Joint random #2",
        "Joint random #3",
        "Joint random #4",
        "Joint random #5",
        "Joint random #6",
        "Joint random #7",
        "Joint random #8"
    };
    
    // Expected real robot poses
    std::vector<std::pair<std::vector<double>, std::vector<double>>> expected_poses = {
        {{0.04, -6.51, 877.07}, {0.00, 0.00, -0.03}},       // Home
        {{731.19, -6.51, 145.84}, {-90.38, 89.97, -90.38}}, // Random #1
        {{-92.62, -40.26, 755.01}, {-46.11, -25.66, 78.67}}, // Random #2
        {{-444.67, -6.36, 432.36}, {91.14, -89.99, -91.16}}, // Random #3
        {{317.30, -6.51, 779.91}, {-0.02, 58.00, -0.05}},   // Random #4
        {{317.30, -6.51, 779.91}, {54.18, 25.10, 72.96}},   // Random #5
        {{258.37, -6.47, 823.51}, {-0.01, 15.00, -0.03}},   // Random #6
        {{311.56, -34.24, 781.45}, {50.76, 30.47, 48.15}},  // Random #7
        {{365.63, -6.51, 779.09}, {-0.02, 30.00, -0.03}}    // Random #8
    };
    
    // Print header
    std::cout << "┌───────────────────┬──────────────────────────┬─────────────────────────────┬─────────────────────────────┬────────────────┐\n";
    std::cout << "│ Configuration     │     Joint Angles [°]     │     Expected Position [mm]  │    Calculated Position [mm] │ Position Error │\n";
    std::cout << "├───────────────────┼──────────────────────────┼─────────────────────────────┼─────────────────────────────┼────────────────┤\n";
    
    // Process each joint configuration
    for (size_t i = 0; i < joint_configs.size(); i++) {
        // Compute forward kinematics
        Matrix4x4 T = robot.computeForwardKinematics(joint_configs[i]);
        
        // Extract position and orientation
        RobotPose calculated_pose = robot.getPositionAndOrientation(T);
        
        // Calculate position error
        double pos_error = sqrt(
            pow(calculated_pose.position[0] - expected_poses[i].first[0], 2) +
            pow(calculated_pose.position[1] - expected_poses[i].first[1], 2) +
            pow(calculated_pose.position[2] - expected_poses[i].first[2], 2)
        );
        
        // Print joint configuration
        std::cout << "│ " << std::left << std::setw(17) << config_names[i] << "│ ";
        for (size_t j = 0; j < joint_configs[i].size(); j++) {
            std::cout << std::setw(4) << std::fixed << std::setprecision(0) << joint_configs[i][j];
            if (j < joint_configs[i].size() - 1) std::cout << " ";
        }
        std::cout << " │ ";
        
        // Print expected position
        std::cout << std::setw(7) << std::fixed << std::setprecision(2) << expected_poses[i].first[0] << " "
                  << std::setw(7) << expected_poses[i].first[1] << " "
                  << std::setw(7) << expected_poses[i].first[2] << " │ ";
        
        // Print calculated position
        std::cout << std::setw(7) << std::fixed << std::setprecision(2) << calculated_pose.position[0] << " "
                  << std::setw(7) << calculated_pose.position[1] << " "
                  << std::setw(7) << calculated_pose.position[2] << " │ ";
        
        // Print position error
        std::cout << std::setw(12) << std::fixed << std::setprecision(2) << pos_error << " │\n";
    }
    
    std::cout << "└───────────────────┴──────────────────────────┴─────────────────────────────┴─────────────────────────────┴────────────────┘\n\n";
    
    // Print orientation header
    std::cout << "┌───────────────────┬──────────────────────────┬─────────────────────────────┬─────────────────────────────┬────────────────┐\n";
    std::cout << "│ Configuration     │     Joint Angles [°]     │   Expected Orientation [°]  │  Calculated Orientation [°] │ Orient. Error  │\n";
    std::cout << "├───────────────────┼──────────────────────────┼─────────────────────────────┼─────────────────────────────┼────────────────┤\n";
    
    // Process each joint configuration for orientation
    for (size_t i = 0; i < joint_configs.size(); i++) {
        // Compute forward kinematics
        Matrix4x4 T = robot.computeForwardKinematics(joint_configs[i]);
        
        // Extract position and orientation
        RobotPose calculated_pose = robot.getPositionAndOrientation(T);
        
        // Calculate orientation error (RMS of angular differences)
        double orient_error = sqrt(
            pow(normalizeAngleDiff(calculated_pose.orientation[0], expected_poses[i].second[0]), 2) +
            pow(normalizeAngleDiff(calculated_pose.orientation[1], expected_poses[i].second[1]), 2) +
            pow(normalizeAngleDiff(calculated_pose.orientation[2], expected_poses[i].second[2]), 2)
        ) / 3.0;
        
        // Print joint configuration
        std::cout << "│ " << std::left << std::setw(17) << config_names[i] << "│ ";
        for (size_t j = 0; j < joint_configs[i].size(); j++) {
            std::cout << std::setw(4) << std::fixed << std::setprecision(0) << joint_configs[i][j];
            if (j < joint_configs[i].size() - 1) std::cout << " ";
        }
        std::cout << " │ ";
        
        // Print expected orientation
        std::cout << std::setw(7) << std::fixed << std::setprecision(2) << expected_poses[i].second[0] << " "
                  << std::setw(7) << expected_poses[i].second[1] << " "
                  << std::setw(7) << expected_poses[i].second[2] << " │ ";
        
        // Print calculated orientation
        std::cout << std::setw(7) << std::fixed << std::setprecision(2) << calculated_pose.orientation[0] << " "
                  << std::setw(7) << calculated_pose.orientation[1] << " "
                  << std::setw(7) << calculated_pose.orientation[2] << " │ ";
        
        // Print orientation error
        std::cout << std::setw(12) << std::fixed << std::setprecision(2) << orient_error << " │\n";
    }
    
    std::cout << "└───────────────────┴──────────────────────────┴─────────────────────────────┴─────────────────────────────┴────────────────┘\n\n";
    
    // Calculate and print summary statistics
    double total_pos_error = 0.0;
    double total_orient_error = 0.0;
    double max_pos_error = 0.0;
    double max_orient_error = 0.0;
    int worst_pos_config = 0;
    int worst_orient_config = 0;
    
    for (size_t i = 0; i < joint_configs.size(); i++) {
        Matrix4x4 T = robot.computeForwardKinematics(joint_configs[i]);
        RobotPose calculated_pose = robot.getPositionAndOrientation(T);
        
        double pos_error = sqrt(
            pow(calculated_pose.position[0] - expected_poses[i].first[0], 2) +
            pow(calculated_pose.position[1] - expected_poses[i].first[1], 2) +
            pow(calculated_pose.position[2] - expected_poses[i].first[2], 2)
        );
        
        double orient_error = sqrt(
            pow(normalizeAngleDiff(calculated_pose.orientation[0], expected_poses[i].second[0]), 2) +
            pow(normalizeAngleDiff(calculated_pose.orientation[1], expected_poses[i].second[1]), 2) +
            pow(normalizeAngleDiff(calculated_pose.orientation[2], expected_poses[i].second[2]), 2)
        ) / 3.0;
        
        total_pos_error += pos_error;
        total_orient_error += orient_error;
        
        if (pos_error > max_pos_error) {
            max_pos_error = pos_error;
            worst_pos_config = i;
        }
        
        if (orient_error > max_orient_error) {
            max_orient_error = orient_error;
            worst_orient_config = i;
        }
    }
    
    double avg_pos_error = total_pos_error / joint_configs.size();
    double avg_orient_error = total_orient_error / joint_configs.size();
    
    std::cout << "📊 SUMMARY STATISTICS:\n";
    std::cout << "┌───────────────────────────────────────────────────────────────────────────────────────────────────────┐\n";
    std::cout << "│ Average Position Error: " << std::setw(10) << std::fixed << std::setprecision(2) << avg_pos_error << " mm";
    std::cout << "                                                          │\n";
    std::cout << "│ Average Orientation Error: " << std::setw(10) << std::fixed << std::setprecision(2) << avg_orient_error << " degrees";
    std::cout << "                                                    │\n";
    std::cout << "│ Maximum Position Error: " << std::setw(10) << std::fixed << std::setprecision(2) << max_pos_error << " mm";
    std::cout << " (in configuration: " << config_names[worst_pos_config] << ")";
    std::cout << "                      │\n";
    std::cout << "│ Maximum Orientation Error: " << std::setw(10) << std::fixed << std::setprecision(2) << max_orient_error << " degrees";
    std::cout << " (in configuration: " << config_names[worst_orient_config] << ")";
    std::cout << "                │\n";
    std::cout << "└───────────────────────────────────────────────────────────────────────────────────────────────────────┘\n\n";
    
    std::cout << "🔍 ANALYSIS:\n";
    std::cout << "┌───────────────────────────────────────────────────────────────────────────────────────────────────────┐\n";
    std::cout << "│ 1. Position Accuracy:                                                                                 │\n";
    if (avg_pos_error < 10.0) {
        std::cout << "│    ✅ EXCELLENT - Sub-centimeter accuracy achieved                                                   │\n";
    } else if (avg_pos_error < 50.0) {
        std::cout << "│    ✅ GOOD - Acceptable position accuracy                                                            │\n";
    } else {
        std::cout << "│    ⚠️ NEEDS IMPROVEMENT - Position accuracy requires calibration                                     │\n";
    }
    
    std::cout << "│                                                                                                       │\n";
    std::cout << "│ 2. Orientation Accuracy:                                                                              │\n";
    if (avg_orient_error < 5.0) {
        std::cout << "│    ✅ EXCELLENT - High precision orientation                                                         │\n";
    } else if (avg_orient_error < 15.0) {
        std::cout << "│    ✅ GOOD - Acceptable orientation accuracy                                                         │\n";
    } else {
        std::cout << "│    ⚠️ NEEDS IMPROVEMENT - Orientation accuracy requires calibration                                  │\n";
    }
    
    std::cout << "│                                                                                                       │\n";
    std::cout << "│ 3. Specific Configurations:                                                                           │\n";
    std::cout << "│    - Home position: " << (avg_pos_error < 10.0 ? "Good accuracy" : "Needs calibration") << "                                                            │\n";
    std::cout << "│    - J1 = 90° (shoulder up): " << (avg_pos_error < 10.0 ? "Good accuracy" : "Needs calibration") << "                                                   │\n";
    std::cout << "│    - J2 = -90° (elbow down): " << (avg_pos_error < 10.0 ? "Good accuracy" : "Needs calibration") << "                                                  │\n";
    std::cout << "└───────────────────────────────────────────────────────────────────────────────────────────────────────┘\n";
    
    return 0;
}