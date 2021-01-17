#include "ukf.h"
#include <gtest/gtest.h>
#include "Eigen/Dense"

TEST(AugmentedSigmaPointGenerationTest, HandleInput)
{
 auto state_space_dim = 5;
 auto augmented_state_space_dim = 7;
 
 Eigen::MatrixXd P = Eigen::MatrixXd(state_space_dim, state_space_dim);      
 P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
         -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
          0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
         -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
         -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
   
 Eigen::VectorXd x = Eigen::MatrixXd(state_space_dim);
 x <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;
 
 UKF ukf;

 ukf.x_ = x;
 ukf.P_ = P;
 
 Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(augmented_state_space_dim, 2*augmented_state_space_dim+1);
 Xsig_aug.fill(0.0);
 
 ukf.AugmentedSigmaPoints(Xsig_aug);
 
 Eigen::MatrixXd Xsig_aug_expected = Eigen::MatrixXd(augmented_state_space_dim, 2*augmented_state_space_dim+1);
 Xsig_aug_expected <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

 EXPECT_TRUE(Xsig_aug.isApprox(Xsig_aug_expected));

}