#include <casadi_kin_dyn/casadi_kin_dyn.h>
#include <casadi_kin_dyn/casadi_collision_handler.h>

#include <casadi/casadi.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/detail/common.h>
#include <pybind11/stl.h>


namespace py = pybind11;
using namespace casadi_kin_dyn;

using Ref = CasadiKinDyn::ReferenceFrame;

template <typename T, typename... Args>
auto make_deserialized(casadi::Function (T::* mem_fn)(Args... args))
{

    // make lambda to create deserialized function
    auto deserialized = [mem_fn](T& self, Args... args)
    {
        // call member function
        auto fn = (self.*mem_fn)(args...);

#if PYBIND11_VERSION_MINOR > 6
        auto cs = py::module_::import("casadi");
#else
        auto cs = py::module::import("casadi");
#endif
        auto Function = cs.attr("Function");
        auto deserialize = Function.attr("deserialize");
        return deserialize(fn.serialize());
    };

    return deserialized;
}


PYBIND11_MODULE(CASADI_KIN_DYN_MODULE, m) {

    py::class_<CasadiKinDyn, CasadiKinDyn::Ptr> casadikindyn(m, "CasadiKinDyn");

    casadikindyn.def(py::init<std::string, bool, std::map<std::string, double>>(),
                     py::arg("urdf"),
                     py::arg("verbose") = false,
                     py::arg("fixed_joints") = py::dict())
            .def(py::init<std::string>())
            .def("nq", &CasadiKinDyn::nq)
            .def("nv", &CasadiKinDyn::nv)
            .def("mapToQ", &CasadiKinDyn::mapToQ)
            .def("mapToV", &CasadiKinDyn::mapToV)
            .def("getMinimalQ", &CasadiKinDyn::getMinimalQ)
            .def("q_min", &CasadiKinDyn::q_min)
            .def("q_max", &CasadiKinDyn::q_max)
            .def("joint_names", &CasadiKinDyn::joint_names)
            .def("rnea",
                 make_deserialized(&CasadiKinDyn::rnea))
            .def("qdot",
                 make_deserialized(&CasadiKinDyn::qdot))
            .def("integrate",
                 make_deserialized(&CasadiKinDyn::integrate))
            .def("aba",
                 make_deserialized(&CasadiKinDyn::aba))
            .def("crba",
                 make_deserialized(&CasadiKinDyn::crba))
            .def("ccrba",
                 make_deserialized(&CasadiKinDyn::ccrba))
            .def("computeCentroidalDynamics",
                 make_deserialized(&CasadiKinDyn::computeCentroidalDynamics))
            .def("fk",
                 make_deserialized<CasadiKinDyn, std::string>(&CasadiKinDyn::fk))
            .def("centerOfMass",
                 make_deserialized(&CasadiKinDyn::centerOfMass))
            .def("jacobian",
                 make_deserialized<CasadiKinDyn, std::string, Ref>(&CasadiKinDyn::jacobian))
            .def("frameVelocity",
                 make_deserialized<CasadiKinDyn, std::string, Ref>(&CasadiKinDyn::frameVelocity))
            .def("frameAcceleration",
                 make_deserialized<CasadiKinDyn, std::string, Ref>(&CasadiKinDyn::frameAcceleration))
            .def("kineticEnergy",
                 make_deserialized(&CasadiKinDyn::kineticEnergy))
            .def("potentialEnergy",
                 make_deserialized(&CasadiKinDyn::potentialEnergy))
            .def("mass", &CasadiKinDyn::mass)
            .def("urdf", &CasadiKinDyn::urdf)
            ;

    py::enum_<CasadiKinDyn::ReferenceFrame>(casadikindyn, "ReferenceFrame")
        .value("LOCAL", CasadiKinDyn::ReferenceFrame::LOCAL)
        .value("WORLD", CasadiKinDyn::ReferenceFrame::WORLD)
        .value("LOCAL_WORLD_ALIGNED", CasadiKinDyn::ReferenceFrame::LOCAL_WORLD_ALIGNED).export_values();


    py::class_<CasadiCollisionHandler> collhandler(m, "CollisionHandler");

    collhandler
            .def(py::init<CasadiKinDyn::Ptr, std::string>(),
                    py::arg("kd"), py::arg("srdf"))
            .def("getDistanceFunction",
                 make_deserialized(&CasadiCollisionHandler::getDistanceFunction))
            .def("addShape", &CasadiCollisionHandler::addShape,
                 py::arg("name"), py::arg("type"), py::arg("params"))
            ;


}



