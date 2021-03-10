#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "mobilerack-interface/QualisysClient.h"
#include "mobilerack-interface/ValveController.h"

namespace py = pybind11;

/**
 * @brief define python bindings to C++ classes
 * 
 * see mobilerack-interface/examples_python for example usage.
 */

PYBIND11_MODULE(mobilerack_pybind_module, m){
    /** @todo add CI test to python as well! */
    py::class_<QualisysClient>(m, "QualisysClient")
            .def(py::init<const char*, int>())
            .def("getData", [](QualisysClient& qc) {
                // as Eigen::Transform cannot be automatically converted to a Python type and integers are immutable in Python (i.e. cannot be passed by reference to be modified),
                // the function cannot be bound directly, in order for the actual data to be readable from Python.
                // To resolve this, use a lambda function that calls the C++ function and formats it to a type appropriate to return to Python.
                // cf: https://pybind11.readthedocs.io/en/stable/faq.html#limitations-involving-reference-arguments
                std::vector<Eigen::Transform<double, 3, Eigen::Affine>> transform_data;
                unsigned long long int timestamp;
                qc.getData(transform_data, timestamp);
                std::vector<Eigen::Matrix<double, 4, 4>> matrix_data;
                for (auto &transform : transform_data)
                    matrix_data.push_back(transform.matrix());
                return std::make_tuple(matrix_data, timestamp);
            });

    py::class_<ValveController>(m, "ValveController")
            .def(py::init<const char*, const std::vector<int>&, const int>())
            .def("setSinglePressure", &ValveController::setSinglePressure)
            .def("syncTimeStamp", &ValveController::syncTimeStamp);
}
