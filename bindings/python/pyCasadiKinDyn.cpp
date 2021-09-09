#include <casadi_kin_dyn/casadi_kin_dyn.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/detail/common.h>
#include <pybind11/stl.h>


namespace py = pybind11;
using namespace casadi_kin_dyn;



auto make_deserialized(std::string (CasadiKinDyn::* mem_fn)(void))
{

    // make lambda to create deserialized function
    auto deserialized = [mem_fn](CasadiKinDyn& self)
    {
        // call member function
        auto fstr = (self.*mem_fn)();

#if PYBIND11_VERSION_MINOR > 6
        auto cs = py::module_::import("casadi");
#else
        auto cs = py::module::import("casadi");
#endif
        auto Function = cs.attr("Function");
        auto deserialize = Function.attr("deserialize");
        return deserialize(fstr);
    };

    return deserialized;
}

PYBIND11_MODULE(CASADI_KIN_DYN_MODULE, m) {
    
    py::class_<CasadiKinDyn> casadikindyn(m, "CasadiKinDyn");

    casadikindyn.def(py::init<std::string>())
            .def(py::init<std::string>())
            .def("nq", &CasadiKinDyn::nq)
            .def("nv", &CasadiKinDyn::nv)
            .def("q_min", &CasadiKinDyn::q_min)
            .def("q_max", &CasadiKinDyn::q_max)
            .def("rnea", &CasadiKinDyn::rnea)
            .def("aba", make_deserialized(&CasadiKinDyn::aba))
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



