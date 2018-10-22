// Copyright 2018 Yasu
#include "CurvatureCalculator.h"

#include "OptiTrackClient.cpp"
// This looks 100% like I have no idea what I'm doing...
// But, due to the implementation of NatNetLinux, it is impossible to link
// multiple compiled object files that use NatNetLinux.
// So, rather than linking them while compiling, I just included the cpp files
// directly.

/*
https://stackoverflow.com/questions/25504397/eigen-combine-rotation-and-translation-into-one-matrix/25504884

*/
CurvatureCalculator::CurvatureCalculator(int numOfRigidBodies,
                                         int sensorType = USE_OPTITRACK)
    : numOfRigidBodies(numOfRigidBodies), sensorType(sensorType) {
  for (int i = 0; i <= numOfRigidBodies; i++) {
    // initialize positions and quaternions
    // positions.push_back(Eigen::Translation<double,3>(0,0,0));
    // quaternions.push_back(Eigen::Quaternion<double>);
    transforms.push_back(
        // Eigen::Transform<double, 3, Eigen::Affine>{}.Identity());
        Eigen::Matrix4d{});
  }
}

void CurvatureCalculator::setupOptiTrack(std::string localAddress,
                                         std::string serverAddress) {
  if (sensorType != USE_OPTITRACK) {
    std::cout << "error: CurvatureCalculator not set up to use OptiTrack"
              << '\n';
    return;
  }
  optiTrackClient = new OptiTrackClient(localAddress, serverAddress);
}

void CurvatureCalculator::setupIntegratedSensor() {
  // for future.
}
void CurvatureCalculator::calculateCurvature() {
  // first, update the internal data for positions and quaternions of each frame
  if (sensorType == USE_OPTITRACK) {
    std::vector<RigidBody> rigidBodies = optiTrackClient->getData();
    for (int i = 0; i < rigidBodies.size(); i++) {
      int id = rigidBodies[i].id();
      if (0 <= id && id < numOfRigidBodies + 1) {
        // positions[id]=RigidBodies[i]->location;
        // quaternions[id] = RigidBodies[i]->orientation;
        Point3f position = rigidBodies[i].location();
        Quaternion4f quaternion = rigidBodies[i].orientation();
        transforms[id] =
            (Eigen::Affine3d{Eigen::Quaterniond(quaternion.qw, quaternion.qx,
                                                quaternion.qy, quaternion.qz)} *
             Eigen::Affine3d{Eigen::Translation3d(
                 Eigen::Vector3d(position.x, position.y, position.z))})
                .matrix();
      }
    }
  } else if (sensorType == USE_INTEGRATEDSENSOR) {
    // to be written?
  }

  // next, calculate the curvature (theta and phi)
  for (int i = 0; i < numOfRigidBodies; i++) {
    // Eigen::Quaterniond
  }
}

void CurvatureCalculator::stop() {
  if (sensorType == USE_OPTITRACK) {
    optiTrackClient->stop();
    delete optiTrackClient;
  }
}
int main() { return 1; }
