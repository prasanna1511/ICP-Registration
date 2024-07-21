#include <iostream>
#include "myPointCloud.hpp"
#include <open3d/Open3D.h>

std::vector<Eigen::Matrix<double, 3, 1>> pointCloudToVector(const open3d::geometry::PointCloud& pcd) {
    std::vector<Eigen::Matrix<double, 3, 1>> points;
    points.reserve(pcd.points_.size());

    for (const auto& point : pcd.points_) {
        points.emplace_back(point.x(), point.y(), point.z());
    }

    return points;
}

void applyTransformation(open3d::geometry::PointCloud& pcd, const Eigen::Matrix4d& transformation) {
    for (auto& point : pcd.points_) {
        Eigen::Vector4d hom_point(point.x(), point.y(), point.z(), 1.0);
        Eigen::Vector4d transformed_point = transformation * hom_point;
        point = transformed_point.head<3>();
    }
}


int main(int argc, char const *argv[]) {
  if (argc != 3) {
    std::cerr << "[ERROR] Please provide the path to a pointcloud" << std::endl;
    return 1;
  }

  const std::string &filename(argv[1]), &filename1(argv[2]);
  open3d::geometry::PointCloud pcd1, pcd2;
  open3d::io::ReadPointCloud(filename, pcd1);
  open3d::io::ReadPointCloud(filename1, pcd2);
  viewCloud(pcd1, pcd2);

std::vector<Eigen::Matrix<double, 3, 1>> pcd1_points = pointCloudToVector(pcd1);
std::vector<Eigen::Matrix<double, 3, 1>> pcd2_points = pointCloudToVector(pcd2);

Eigen::Vector3d centroid1 = computeCentroid(pcd1_points);
Eigen::Vector3d centroid2 = computeCentroid(pcd2_points);  
std::cout << "Centroid 1: " << centroid1.transpose() << std::endl;
std::cout << "Centroid 2: " << centroid2.transpose() << std::endl;

std::vector<Eigen::Matrix<double, 3, 1>> pcd1_prime = substractCentroid(pcd1_points, centroid1);
std::vector<Eigen::Matrix<double, 3, 1>> pcd2_prime = substractCentroid(pcd2_points, centroid2);

Eigen::Matrix3d H = covariance(pcd1_prime, pcd2_prime);
std::cout << "Covariance Matrix H: " << std::endl << H << std::endl;
viewCloud(pcd1_prime, pcd2_prime);


Eigen::Matrix3d R = SVD(H);
std::cout << "Rotation Matrix R: " << std::endl << R << std::endl;

std::cout<<"orthonormality check: \n"<< H*H.transpose() <<std::endl;


Eigen::Matrix3d R_det = checkR(R);
std::cout << "Check R determinant: " << R_det.determinant() << std::endl;


Eigen::Vector3d t = translation(centroid1, centroid2, R);
std::cout << "Translation Vector t: " << t.transpose() << std::endl;


Eigen::Matrix4d transformation = Transformation(R, t);
std::cout << "Transformation Matrix: " << std::endl << transformation << std::endl;

applyTransformation(pcd1, transformation);
viewCloud(pcd1, pcd2);

return 0;
}

