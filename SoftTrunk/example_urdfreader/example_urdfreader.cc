/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2016 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>

#include <rbdl/rbdl.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
	#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) {
	rbdl_check_api_version (RBDL_API_VERSION);

	Model* model = new Model();

	if (!Addons::URDFReadFromFile ("../urdf/robot.urdf", model, false)) {
		std::cerr << "Error loading model ../urdf/robot.urdf" << std::endl;
		abort();
	}

	model->gravity = Vector3d (0., 0., 	9.81);

	VectorNd Q = VectorNd::Zero (model->dof_count);
	Q(0) = 3.14/4;
	VectorNd QDot = VectorNd::Zero (model->dof_count);
	VectorNd Tau = VectorNd::Zero (model->dof_count);
	VectorNd QDDot = VectorNd::Zero (model->dof_count);

//	ForwardDynamics(*model, Q, QDot, Tau, QDDot);
 	InverseDynamics (*model, Q, QDot, QDDot, Tau);

	std::cout << Q.transpose() << std::endl;
	std::cout << Tau.transpose() << std::endl;

	delete model;

 	return 0;
}
