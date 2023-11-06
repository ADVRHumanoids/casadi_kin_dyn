#ifndef CASADI_PINOCCHIO_BRIDGE_H
#define CASADI_PINOCCHIO_BRIDGE_H

#include <string>
#include <memory>
#include <vector>
#include <map>
#include <any>

#include <Eigen/Dense>

namespace casadi {
class Function;
}

namespace casadi_kin_dyn {

class CasadiKinDyn
{

public:

    typedef std::shared_ptr<CasadiKinDyn> Ptr;

    enum ReferenceFrame
    {
        WORLD = 0, //This is spatial in world frame
        LOCAL = 1, //This is spatial in local frame
        LOCAL_WORLD_ALIGNED = 2 //This is classical in world frame
    };

    CasadiKinDyn(std::string urdf_string,
                 bool verbose = false,
                 std::map<std::string, double> fixed_joints = std::map<std::string, double>{});

    CasadiKinDyn(const CasadiKinDyn& other);

    int nq() const;

    int nv() const;

    int joint_nq(const std::string& jname) const;

    int joint_iq(const std::string& jname) const;

    Eigen::VectorXd mapToQ(std::map<std::string, double> jmap);

    Eigen::VectorXd mapToV(std::map<std::string, double> jmap);

    Eigen::VectorXd getMinimalQ(Eigen::VectorXd q);

    casadi::Function integrate();

    casadi::Function qdot();

    void qdot(Eigen::Ref<const Eigen::VectorXd> q,
              Eigen::Ref<const Eigen::VectorXd> v,
              Eigen::Ref<Eigen::VectorXd> qdot);

    casadi::Function rnea();

    casadi::Function computeCentroidalDynamics();

    casadi::Function ccrba();

    casadi::Function crba();

    casadi::Function aba();

    casadi::Function fk(std::string link_name);

    casadi::Function centerOfMass();

    casadi::Function jacobian(std::string link_name, ReferenceFrame ref);

    casadi::Function frameVelocity(std::string link_name, ReferenceFrame ref);

    casadi::Function frameAcceleration(std::string link_name, ReferenceFrame ref);

    casadi::Function kineticEnergy();

    casadi::Function potentialEnergy();

    Eigen::VectorXd velocityLimits() const;
    Eigen::VectorXd effortLimits() const;

    std::vector<double> q_min() const;

    std::vector<double> q_max() const;

    std::vector<std::string> joint_names() const;

    double mass() const;

    std::string urdf() const;

    std::string parentLink(const std::string& jname) const;

    std::string childLink(const std::string& jname) const;

    std::any modelHandle() const;

    ~CasadiKinDyn();

private:

    class Impl;
    std::unique_ptr<Impl> _impl;
    const Impl& impl() const;
    Impl& impl();

};

}

#endif // CASADI_PINOCCHIO_BRIDGE_H
