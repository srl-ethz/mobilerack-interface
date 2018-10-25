#include "CurvatureCalculator.h"

/*
Eigen- getting started: https://eigen.tuxfamily.org/dox/GettingStarted.html
*/
CurvatureCalculator::CurvatureCalculator(int numOfRigidBodies,
                                         int sensorType = USE_OPTITRACK)
    : numOfRigidBodies(numOfRigidBodies), sensorType(sensorType) {
  for (int i = 0; i <= numOfRigidBodies; i++) {
    // initialize transforms
    abs_transforms.push_back(Eigen::Transform<double, 3, Eigen::Affine>().Identity());
  }
  for (int j = 0; j < numOfRigidBodies; ++j) {
    theta.push_back(0.);
    phi.push_back(0.);
    rel_transforms.push_back(Eigen::Transform<double, 3, Eigen::Affine>().Identity());
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
double sign(double val){
  if (val==0) return 0.0;
  else if (val>0) return 1.0;
  else return -1.0;
}
void CurvatureCalculator::calculateCurvature() {
  // first, update the internal data for transforms of each frame
  if (sensorType == USE_OPTITRACK) {
    std::vector<RigidBody> rigidBodies = optiTrackClient->getData();
    for (int i = 0; i < rigidBodies.size(); i++) {
      int id = rigidBodies[i].id();
      if (0 <= id && id < numOfRigidBodies + 1) {
        // positions[id]=RigidBodies[i]->location;
        // quaternions[id] = RigidBodies[i]->orientation;

        Point3f position = rigidBodies[i].location();

          Quaternion4f quaternion = rigidBodies[i].orientation();
        Eigen::Quaterniond quaternion_eigen;

        quaternion_eigen.x() = quaternion.qx;
        quaternion_eigen.y() = quaternion.qy;
        quaternion_eigen.z() = quaternion.qz;
        quaternion_eigen.w() = quaternion.qw;
        quaternion_eigen.normalize();
        abs_transforms[id] = quaternion_eigen * Eigen::Translation3d(position.x, position.y, position.z);
      }
    }
  } else if (sensorType == USE_INTEGRATEDSENSOR) {
    // to be written?
  }


  // next, calculate the curvature (theta and phi)
  for (int i = 0; i < numOfRigidBodies; i++) {
    rel_transforms[i] = abs_transforms[i+1] * abs_transforms[i].inverse();

    Eigen::Transform<double, 3, Eigen::Affine>::MatrixType matrix = rel_transforms[i].matrix();
    phi[i] = atan(matrix(1,3)/matrix(0,3));
    theta[i] = sign(matrix(0,3)) * (asin(sqrt(pow(matrix(0,2),2) + pow(matrix(1,2),2))));
  }
}

void CurvatureCalculator::stop() {
  if (sensorType == USE_OPTITRACK) {
    optiTrackClient->stop();
    delete optiTrackClient;
  }
}
int main() { return 1; }
