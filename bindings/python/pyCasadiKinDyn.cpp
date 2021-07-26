#include <casadi_kin_dyn/casadi_kin_dyn.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

namespace py = pybind11;
using namespace casadi_kin_dyn;

PYBIND11_MODULE(pycasadi_kin_dyn, m) {
    
    py::class_<CasadiKinDyn>casadikindyn(m, "CasadiKinDyn");

    casadikindyn.def(py::init<std::string>())
    //py::class_<CasadiKinDyn>(m, "CasadiKinDyn")
            .def(py::init<std::string>())
            .def("nq", &CasadiKinDyn::nq)
            .def("nv", &CasadiKinDyn::nv)
            .def("rnea", &CasadiKinDyn::rnea)
            .def("crba", &CasadiKinDyn::crba)
            .def("ccrba", &CasadiKinDyn::ccrba)
            .def("computeCentroidalDynamics", &CasadiKinDyn::computeCentroidalDynamics)
            .def("fk", &CasadiKinDyn::fk)
            .def("centerOfMass", &CasadiKinDyn::centerOfMass)
            .def("jacobian", &CasadiKinDyn::jacobian)
            .def("frameVelocity", &CasadiKinDyn::frameVelocity)
            .def("frameAcceleration", &CasadiKinDyn::frameAcceleration)
            .def("kineticEnergy", &CasadiKinDyn::kineticEnergy)
            .def("potentialEnergy", &CasadiKinDyn::potentialEnergy)
            ;

    py::enum_<CasadiKinDyn::ReferenceFrame>(casadikindyn, "ReferenceFrame")
        .value("LOCAL", CasadiKinDyn::ReferenceFrame::LOCAL)
        .value("WORLD", CasadiKinDyn::ReferenceFrame::WORLD)
        .value("LOCAL_WORLD_ALIGNED", CasadiKinDyn::ReferenceFrame::LOCAL_WORLD_ALIGNED).export_values();

    
}



