// #include <iostream>
// #include <cmath>
// #include <Eigen/Dense>

// using namespace Eigen;
// using namespace std;

// int main() {
//     const double D2R = M_PI / 180.0;
//     const double R2D = 180.0 / M_PI;

//     // Input Cartesian values
//     // double input_x = -156.76;
//     // double input_y = -155.15;
//     // double input_z = 814.96;
//     // double input_rx = -43.47;
//     // double input_ry = 80.56;
//     // double input_rz = -60.88;

//     double input_x = -1.12;
//     double input_y = -208.31;
//     // double input_z = 1096.68;
//     double input_z = 927.27;
//     double input_rx = 0.00;
//     double input_ry = 0.00;
//     double input_rz = -0.28;

//     double x = input_x;
//     double y = input_y;
//     double z = input_z;
//     double rx = input_rx * D2R;
//     double ry = input_ry * D2R;
//     double rz = input_rz * D2R;

//     // Robot parameters
//     // double d1 = 169.2;
//     // double d2 = 148.4;
//     // double d3 = 148.4;
//     // double d4 = 110.7;
//     // double d5 = 110.7;
//     // double d6 = 96.7;
//     // double a1 = 425.0;
//     // double a2 = 392.0;

//     double d1 = 165.5;
//     double d2 = 151.4;
//     double d3 = 151.4;
//     double d4 = 110.7;
//     double d5 = 110.7;
//     double d6 = 96.7;
//     double a1 = 425.0;
//     double a2 = 392.0;

//     // Rotation matrices
//     Matrix3d Rz, Ry, Rx;
//     Rz << cos(rz), -sin(rz), 0,
//           sin(rz),  cos(rz), 0,
//           0, 0, 1;

//     Ry << cos(ry), 0, sin(ry),
//           0, 1, 0,
//           -sin(ry), 0, cos(ry);

//     Rx << 1, 0, 0,
//           0, cos(rx), -sin(rx),
//           0, sin(rx), cos(rx);

//     Matrix3d R = Rz * Ry * Rx;

//     Vector3d Y06 = R.col(1);
//     Vector3d P06(x, y, z);
//     Vector3d P05 = P06 + d6 * Y06;

//     // Calculate theta1, theta5, theta6
//     double th1 = atan2(P05(1), P05(0)) - acos(d4 / sqrt(pow(P05(1), 2) + pow(P05(0), 2))) + 0.5 * M_PI;
//     double th5 = acos((sin(th1)*P06(0) - cos(th1)*P06(1) - d4) / d6);
//     double numerator1 = -(-sin(th1)*R(0,0) + cos(th1)*R(1,0));
//     double numerator2 = -sin(th1)*R(0,2) + cos(th1)*R(1,2);
//     double denominator = sin(th5);
//     double th6 = atan2(numerator1/denominator, numerator2/denominator) + 0.5 * M_PI;

//     // Transformation matrices
//     Matrix4d A01;
//     A01 << cos(th1), 0, -sin(th1), 0,
//            sin(th1), 0,  cos(th1), 0,
//            0, -1, 0, d1,
//            0, 0, 0, 1;

//     Matrix4d A67;
//     A67 << 1, 0, 0, 0,
//            0, 0, -1, 0,
//            0, 1, 0, 0,
//            0, 0, 0, 1;

//     Matrix4d A78;
//     A78 << cos(th5), 0, -sin(th5), 0,
//            sin(th5), 0,  cos(th5), 0,
//            0, -1, 0, d5,
//            0, 0, 0, 1;

//     Matrix4d A89;
//     A89 << cos(th6), 0, sin(th6), 0,
//            sin(th6), 0, -cos(th6), 0,
//            0, 1, 0, -d6,
//            0, 0, 0, 1;

//     Matrix4d T;
//     T.block<3,3>(0,0) = R;
//     T.block<3,1>(0,3) = P06;
//     T(3,3) = 1;

//     Matrix4d A17 = A01.inverse() * T * A89.inverse() * A78.inverse() * A67.inverse();
//     Vector3d P14(A17(0,3), A17(1,3), A17(2,3));

//     // Calculate theta2, theta3
//     double th3 = acos((pow(P14(0),2) + pow(P14(1),2) - a1*a1 - a2*a2) / (2*a1*a2));
//     double th2 = atan2(P14(0), -P14(1)) - asin((a2*sin(th3)) / sqrt(pow(P14(0),2) + pow(P14(1),2)));

//     // Additional transformation matrices
//     Matrix4d A12;
//     A12 << sin(th2), cos(th2), 0, 0,
//            -cos(th2), sin(th2), 0, 0,
//            0, 0, 1, -d2,
//            0, 0, 0, 1;

//     Matrix4d A23 = Matrix4d::Identity();
//     A23(0,3) = a1;

//     Matrix4d A34;
//     A34 << cos(th3), -sin(th3), 0, 0,
//            sin(th3),  cos(th3), 0, 0,
//            0, 0, 1, d3,
//            0, 0, 0, 1;

//     Matrix4d A45 = Matrix4d::Identity();
//     A45(0,3) = a2;

//     Matrix4d A56_cal = A45.inverse() * A34.inverse() * A23.inverse() * A12.inverse() * A01.inverse() * T * A89.inverse() * A78.inverse() * A67.inverse();

//     // Calculate theta4
//     double th4 = atan2(A56_cal(1,0), A56_cal(0,0)) - 0.5 * M_PI;

//     // Convert to degrees
//     th1 *= R2D;
//     th2 *= R2D;
//     th3 *= R2D;
//     th4 *= R2D;
//     th5 *= R2D;
//     th6 *= R2D;

//     // Output results
//     cout << "---------------------------------\n";
//     cout << "Inverse Kinematics Result (deg)\n";
//     cout << "---------------------------------\n";
//     cout << "th1: " << th1 << endl;
//     cout << "th2: " << th2 << endl;
//     cout << "th3: " << th3 << endl;
//     cout << "th4: " << th4 << endl;
//     cout << "th5: " << th5 << endl;
//     cout << "th6: " << th6 << endl;

//     return 0;
// }


#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

class Matrix4x4 {
public:
    double data[4][4];
    
    Matrix4x4() {
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                data[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    }
    
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
    
    void print() const {
        std::cout << std::fixed << std::setprecision(3);
        for(int i = 0; i < 4; i++) {
            std::cout << "[ ";
            for(int j = 0; j < 4; j++) {
                std::cout << std::setw(8) << data[i][j] << " ";
            }
            std::cout << "]\n";
        }
    }
};

class RB3_730ES_FK {
private:
    // DH parameters from official specs
    const struct DHParams {
        double a;        // link length (mm)
        double alpha;    // link twist (rad)
        double d;        // link offset (mm)
        double theta;    // joint angle (rad)
        bool is_revolute;
    } dh_table[9] = {
        {0.0, -M_PI/2, 730.0, 0.0, true},         // L1: Base
        {0.0, 0.0,     -145.3, -M_PI/2, false},    // L2: Shoulder (prismatic)
        {286.0, 0.0,    0.0,    0.0,    false},    // L3: Upper Arm
        {0.0, 0.0,     344.0,  M_PI/2, false},     // L4: Elbow (prismatic)
        {0.0, M_PI/2,   0.0,    0.0,    false},    // L5: Elbow-Wrist
        {0.0, -M_PI/2, 100.0,   0.0,    true},     // L6: Wrist1
        {0.0, 0.0,    -117.15,  0.0,    false},    // L7: Wrist1-2 (prismatic)
        {0.0, M_PI/2,  110.7,   0.0,    true},     // L8: Wrist2
        {0.0, 0.0,     94.6,    0.0,    true}      // L9: Tool
    };
    
    Matrix4x4 createDH(double theta, double d, double a, double alpha) const {
        Matrix4x4 T;
        double ct = cos(theta);
        double st = sin(theta);
        double ca = cos(alpha);
        double sa = sin(alpha);
        
        T.data[0][0] = ct;
        T.data[0][1] = -st * ca;
        T.data[0][2] = st * sa;
        T.data[0][3] = a * ct;
        
        T.data[1][0] = st;
        T.data[1][1] = ct * ca;
        T.data[1][2] = -ct * sa;
        T.data[1][3] = a * st;
        
        T.data[2][1] = sa;
        T.data[2][2] = ca;
        T.data[2][3] = d;
        
        return T;
    }
    
    double deg2rad(double deg) { return deg * M_PI / 180.0; }

public:
    Matrix4x4 computeFK(const std::vector<double>& joints) {
        if(joints.size() != 6) {
            std::cerr << "Need 6 joints: [θ1, d2, θ3, d4, θ5, d6]" << std::endl;
            return Matrix4x4();
        }
        
        Matrix4x4 T;
        for(int i = 0; i < 9; i++) {
            double theta = dh_table[i].theta;
            double d = dh_table[i].d;
            
            if(dh_table[i].is_revolute) {
                // Handle revolute joints with offsets
                if(i == 1) theta += deg2rad(joints[1]);      // θ2-90°
                else if(i == 3) theta += deg2rad(joints[2]); // θ3+90°
                else if(i >= 5) theta += deg2rad(joints[i-3]);
            } else {
                // Handle prismatic joints
                if(i == 1) d += joints[0];     // d2
                else if(i == 3) d += joints[3];// d4
                else if(i == 6) d += joints[4];// d6
            }
            
            T = T * createDH(theta, d, dh_table[i].a, dh_table[i].alpha);
        }
        return T;
    }
    
    void printPose(const Matrix4x4& T) {
        double x = T.data[0][3];
        double y = T.data[1][3];
        double z = T.data[2][3];
        
        std::cout << "End Effector Position:\n";
        std::cout << "X: " << x << " mm\n";
        std::cout << "Y: " << y << " mm\n";
        std::cout << "Z: " << z << " mm\n\n";
    }
};

int main() {
    RB3_730ES_FK fk;
    
    // Test cases
    std::vector<std::vector<double>> tests = {
        {0, 0, 0, 0, 0, 0},       // Home
        {0, 90, 0, 0, 0, 0},      // Shoulder up
        {0, -90, 0, 0, 0, 0},     // Shoulder down
        {45, 30, -60, 120, 30, -90}
    };
    
    for(auto& joints : tests) {
        std::cout << "Testing joints: [";
        for(auto j : joints) std::cout << j << " ";
        std::cout << "]\n";
        
        Matrix4x4 result = fk.computeFK(joints);
        fk.printPose(result);
    }
    
    return 0;
}