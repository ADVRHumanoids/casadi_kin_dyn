#ifndef CASADI_COLLISION_HANDLER_H
#define CASADI_COLLISION_HANDLER_H



#include "casadi_kin_dyn.h"

#include <Eigen/Dense>

namespace casadi_kin_dyn {

class CasadiCollisionHandler
{

public:

    typedef std::map<std::string, std::vector<double>> ShapeParams;

    CasadiCollisionHandler(CasadiKinDyn::Ptr kd,
                           std::string srdf_string);

    void addShape(std::string name,
                  std::string type,
                  ShapeParams params);

    casadi::Function getDistanceFunction();

    size_t numPairs() const;

    bool distance(Eigen::Ref<const Eigen::VectorXd> q,
                  Eigen::Ref<Eigen::VectorXd> d);

    bool distanceJacobian(Eigen::Ref<const Eigen::VectorXd> q,
                          Eigen::Ref<Eigen::MatrixXd> J);

    bool distanceHessian(Eigen::Ref<const Eigen::VectorXd> q,
                         Eigen::Ref<Eigen::MatrixXd> H);

    ~CasadiCollisionHandler();

private:

    class Impl;
    std::unique_ptr<Impl> _impl;
    Impl& impl();
    const Impl& impl() const;
};

}

#endif // CASADI_COLLISION_HANDLER_H
