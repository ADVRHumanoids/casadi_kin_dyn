#ifndef CASADI_PINOCCHIO_BRIDGE_H
#define CASADI_PINOCCHIO_BRIDGE_H

#include <string>
#include <memory>

namespace casadi_kin_dyn {

class CasadiKinDyn
{

public:

    CasadiKinDyn(std::string urdf_string);

    int nq() const;
    int nv() const;

    std::string rnea();
    
    std::string computeCentroidalDynamics();

    std::string ccrba();

    std::string fk(std::string link_name);

    std::string jacobian(std::string link_name);

    ~CasadiKinDyn();

private:

    class Impl;
    std::unique_ptr<Impl> _impl;
    const Impl& impl() const;
    Impl& impl();

};

}

#endif // CASADI_PINOCCHIO_BRIDGE_H
