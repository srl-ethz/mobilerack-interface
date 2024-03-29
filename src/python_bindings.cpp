#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "mobilerack-interface/QualisysClient.h"
#include "mobilerack-interface/ValveController.h"
#include "mobilerack-interface/ndarray_converter.h"

namespace py = pybind11;

/**
 * @brief define python bindings to C++ classes
 * 
 * see mobilerack-interface/examples_python for example usage.
 */

PYBIND11_MODULE(mobilerack_pybind_module, m){
    /** @todo add CI test to python as well! */

    NDArrayConverter::init_numpy(); // this must be called first, or segfault happens during conversion from cv::Mat to numpy array
    py::class_<QualisysClient>(m, "QualisysClient")
            .def(py::init<
                    int, 
                    std::vector<int>, 
                    std::string
                >())
            .def("getData6D", [](QualisysClient& qc) {
                // as Eigen::Transform cannot be automatically converted to a Python type and integers are immutable in Python (i.e. cannot be passed by reference to be modified),
                // the function cannot be bound directly, in order for the actual data to be readable from Python.
                // To resolve this, use a lambda function that calls the C++ function and formats it to a type appropriate to return to Python.
                // cf: https://pybind11.readthedocs.io/en/stable/faq.html#limitations-involving-reference-arguments
                if (qc.frameMode != "6D") {
                    throw std::runtime_error("Incorrect frame type! Not 6D.\n");
                }
                std::vector<Eigen::Transform<double, 3, Eigen::Affine>> transform_data;
                unsigned long long int timestamp;
                qc.getData(transform_data, timestamp);
                std::vector<Eigen::Matrix<double, 4, 4>> matrix_data;
                for (auto &transform : transform_data)
                    matrix_data.push_back(transform.matrix());
                return std::make_tuple(matrix_data, timestamp);
            })
            .def("getData3D", [](QualisysClient& qc) {
                if (qc.frameMode != "3D") {
                    // Unfortunately, pybind needs to know return type at compile time, so we cannot combine 6D and 3D into one "getData" function.
                    throw std::runtime_error("Incorrect frame type! Not 3D.\n");
                }
                std::vector<Eigen::Vector3d> matrix_data;
                unsigned long long int timestamp;
                qc.getData(matrix_data, timestamp);
                return std::make_tuple(matrix_data, timestamp);
            })
            .def("getImage", [](QualisysClient& qc, int id) {
                // cv::Mat is converted to numpy array thanks to ndarray_converter
                cv::Mat image;
                qc.getImage(id, image);
                // fmt::print("image size: {}\n", image.size());
                return image;
            });


    py::class_<ValveController>(m, "ValveController")
            .def(py::init<const std::string&, const std::vector<int>&, const int>())
            .def("setSinglePressure", &ValveController::setSinglePressure)
            .def("syncTimeStamp", &ValveController::syncTimeStamp)
            .def("getSinglePressure", &ValveController::getSinglePressure)
            .def("setPressures", &ValveController::setPressures)
            .def("getPressures", &ValveController::getPressures);
}
