/**
 * @file test_rotation_utils_basic.cpp
 * @brief Basic unit tests for rotation_utils
 *
 * This file contains simple tests to verify the rotation utility functions work correctly.
 * It can be compiled and run as: g++ -std=c++17 test_rotation_utils_basic.cpp rotation_utils.cpp numeric.cpp -o test_rotation -I. -lEigen3 -lglog
 */

#include "rotation_utils.h"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace insight;

void print_result(const std::string& test_name, bool passed) {
    std::cout << std::setw(50) << std::left << test_name 
              << (passed ? "✓ PASS" : "✗ FAIL") << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "\n=== Rotation Utils Tests ===\n" << std::endl;
    
    int passed = 0, total = 0;
    
    // Test 1: Basic OPK to Matrix conversion
    {
        total++;
        double omega = 0.0, phi = 0.0, kappa = 0.0;
        Mat3 R = OPK_to_RotationMatrix(omega, phi, kappa);
        Mat3 I = Mat3::Identity();
        bool test = (R - I).norm() < 1e-10;
        print_result("OPK(0,0,0) should give identity matrix", test);
        if (test) passed++;
    }
    
    // Test 2: Gimbal lock detection
    {
        total++;
        double phi_safe = 0.5;  // Safe
        double phi_danger = M_PI / 2.0;  // Gimbal lock
        
        bool test = !IsGimbalLockRisk(phi_safe) && IsGimbalLockRisk(phi_danger);
        print_result("Gimbal lock detection", test);
        if (test) passed++;
    }
    
    // Test 3: Rotation matrix validity
    {
        total++;
        Mat3 R = OPK_to_RotationMatrix(0.1, 0.2, 0.3);
        bool test = IsValidRotationMatrix(R);
        print_result("Generated rotation matrix is valid", test);
        if (test) passed++;
    }
    
    // Test 4: Round-trip OPK → Matrix → OPK
    {
        total++;
        double omega = 0.05, phi = 0.1, kappa = 0.15;
        double err_o, err_p, err_k;
        double max_err = TestRoundTrip_OPK_Matrix_OPK(omega, phi, kappa, err_o, err_p, err_k);
        bool test = max_err < 1e-10;
        print_result("Round-trip OPK→Matrix→OPK (max error < 1e-10)", test);
        std::cout << "  - Max error: " << std::scientific << max_err << std::defaultfloat << std::endl;
        if (test) passed++;
    }
    
    // Test 5: OPK to Quaternion conversion
    {
        total++;
        double omega = 0.05, phi = 0.1, kappa = 0.15;
        Quaternion q = OPK_to_Quaternion(omega, phi, kappa);
        bool test = IsValidQuaternion(q);
        print_result("Quaternion is normalized", test);
        if (test) passed++;
    }
    
    // Test 6: Round-trip OPK → Quaternion → OPK
    {
        total++;
        double omega = 0.05, phi = 0.1, kappa = 0.15;
        double err_o, err_p, err_k;
        double max_err = TestRoundTrip_OPK_Quaternion_OPK(omega, phi, kappa, err_o, err_p, err_k);
        bool test = max_err < 1e-10;
        print_result("Round-trip OPK→Quaternion→OPK (max error < 1e-10)", test);
        std::cout << "  - Max error: " << std::scientific << max_err << std::defaultfloat << std::endl;
        if (test) passed++;
    }
    
    // Test 7: Matrix to OPK extraction
    {
        total++;
        double omega = 0.05, phi = 0.1, kappa = 0.15;
        Mat3 R = OPK_to_RotationMatrix(omega, phi, kappa);
        double omega2, phi2, kappa2;
        RotationMatrix_to_OPK(R, omega2, phi2, kappa2);
        
        bool test = (std::abs(omega - omega2) < 1e-10 &&
                     std::abs(phi - phi2) < 1e-10 &&
                     std::abs(kappa - kappa2) < 1e-10);
        print_result("Extract OPK from rotation matrix", test);
        if (test) passed++;
    }
    
    // Test 8: Quaternion to OPK extraction
    {
        total++;
        double omega = 0.05, phi = 0.1, kappa = 0.15;
        Quaternion q = OPK_to_Quaternion(omega, phi, kappa);
        double omega2, phi2, kappa2;
        Quaternion_to_OPK(q, omega2, phi2, kappa2);
        
        bool test = (std::abs(omega - omega2) < 1e-10 &&
                     std::abs(phi - phi2) < 1e-10 &&
                     std::abs(kappa - kappa2) < 1e-10);
        print_result("Extract OPK from quaternion", test);
        if (test) passed++;
    }
    
    // Summary
    std::cout << "\n" << std::string(50, '=') << std::endl;
    std::cout << "Results: " << passed << "/" << total << " tests passed" << std::endl;
    std::cout << std::string(50, '=') << "\n" << std::endl;
    
    return (passed == total) ? 0 : 1;
}
