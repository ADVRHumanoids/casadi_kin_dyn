#ifndef CASADI_PINOCCHIO_BRIDGE_H
#define CASADI_PINOCCHIO_BRIDGE_H

#include <string>
#include <memory>
#include <vector>

namespace casadi_kin_dyn {

class CasadiKinDyn
{

public:
    enum ReferenceFrame
    {
      WORLD = 0, //This is spatial in world frame
      LOCAL = 1, //This is spatial in local frame
      LOCAL_WORLD_ALIGNED = 2 //This is classical in world frame
    };

    CasadiKinDyn(std::string urdf_string);

    int nq() const;
    int nv() const;

    std::string rnea();
    
    std::string computeCentroidalDynamics();

    std::string ccrba();

    std::string crba();

    std::string aba();

    std::string fk(std::string link_name);
    
    std::string centerOfMass();

    std::string jacobian(std::string link_name, ReferenceFrame ref);
    
    std::string frameVelocity(std::string link_name, ReferenceFrame ref);

    std::string frameAcceleration(std::string link_name, ReferenceFrame ref);

    std::string kineticEnergy();

    std::string potentialEnergy();

    std::vector<double> q_min() const;

    std::vector<double> q_max() const;

    std::vector<std::string> joint_names() const;

    double mass() const;

    ~CasadiKinDyn();

private:

    class Impl;
    std::unique_ptr<Impl> _impl;
    const Impl& impl() const;
    Impl& impl();

};

}

#endif // CASADI_PINOCCHIO_BRIDGE_H
