#include "myPointCloud.hpp"
#include <numeric>
#include <iostream>  
#include <Eigen/Dense>
#include <vector>


void viewCloud(const open3d::geometry::PointCloud& pcd1, const open3d::geometry::PointCloud& pcd2){
    open3d::visualization::DrawGeometries({
        std::make_shared<open3d::geometry::PointCloud>(pcd1),
        std::make_shared<open3d::geometry::PointCloud>(pcd2)
        });
        }

Eigen::Vector3d computeCentroid(const std::vector<Eigen::Matrix<double, 3, 1>>& points){
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();

    if (points.empty()) {
        return centroid;
    }

    for (const auto& point : points) {
        centroid += point;
    }

    centroid /= points.size();

    return centroid;
}

std::vector<Eigen::Matrix<double, 3,1>> substractCentroid(const std::vector<Eigen::Matrix<double, 3, 1>>& points, const Eigen::Vector3d& centroid){
    std::vector<Eigen::Matrix<double, 3, 1>> centeredPoints;

    centeredPoints.reserve(points.size());

    for (const auto& point : points) {
        centeredPoints.emplace_back(point - centroid);
    }

    return centeredPoints;
}

Eigen::Matrix3d covariance(const std::vector<Eigen::Matrix<double, 3, 1>>& p_prime, const std::vector<Eigen::Matrix<double, 3, 1>>& q_prime){
Eigen::Matrix3d H =Eigen::Matrix3d::Zero();
for (size_t i =0; i<p_prime.size(); ++i){
    H += p_prime[i] * q_prime[i].transpose();
}
return H;
}

Eigen::Matrix3d SVD(const Eigen::Matrix3d& H){
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * U.transpose();
    return R;
}


Eigen::Matrix3d checkR(const Eigen::Matrix3d& R){
    double det = R.determinant();
    if (det < 0){
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        I(2, 2) = -1;
        return R * I;
    }
    return R;
}

Eigen::Vector3d translation(const Eigen::Vector3d& centroid1, const Eigen::Vector3d& centroid2, const Eigen::Matrix3d& R){
    Eigen::Vector3d t = centroid2 - R * centroid1;
    return t;
}

Eigen::Matrix4d Transformation(const Eigen::Matrix3d& R, 
                                          const Eigen::Vector3d& t){
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = R;
    transformation.block<3, 1>(0, 3) = t;
    return transformation;

    }

    