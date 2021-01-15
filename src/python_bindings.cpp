#include <pybind11/pybind11.h>

#include "mobilerack-interface/QualisysClient.h"

namespace py = pybind11;

/**
 * @brief define python bindings to C++ classes
 * 
 * example Python usage:
 * ```
 * cd /path/to/mobilerack-interface/lib
 * python3
 * >> import mobilerack_pybind_module
 * >> qc = mobilerack_pybind_module.QualisysClient("192.169.0.0", 22222, 3)
 * ```
 */

// this binding code will usually be located in a separate file
PYBIND11_MODULE(mobilerack_pybind_module, m){
    py::class_<QualisysClient>(m, "QualisysClient")
    .def(py::init<const char*, const unsigned short, int>());
    // @todo bind other functions
}