#pragma once


#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <Eigen/src/Core/Matrix.h>
#include <open3d/Open3D.h>



void viewCloud(const open3d::geometry::PointCloud& pcd1, 
                const open3d::geometry::PointCloud& pcd2);
                
Eigen::Vector3d computeCentroid(const std::vector<Eigen::Matrix<double, 3, 1>>& points);


std::vector<Eigen::Matrix<double, 3,1>> substractCentroid(const std::vector<Eigen::Matrix<double,3,1>> &points,
                                                const Eigen::Vector3d& centroid);


Eigen::Matrix3d covariance(const std::vector<Eigen::Matrix<double, 3, 1>>& p_prime,
                                 const std::vector<Eigen::Matrix<double, 3, 1>>& q_prime);


Eigen::Matrix3d SVD(const Eigen::Matrix3d& H);

Eigen::Vector3d translation(const Eigen::Vector3d& centroid1, 
                            const Eigen::Vector3d& centroid2, 
                            const Eigen::Matrix3d& R);


Eigen::Matrix4d Transformation(const Eigen::Matrix3d& R, 
                                          const Eigen::Vector3d& t);

Eigen::Matrix3d checkR(const Eigen::Matrix3d& R);

