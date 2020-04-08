#ifndef PINOCCHIOWRAPPER_H
#define PINOCCHIOWRAPPER_H

#include <urdf_parser/urdf_parser.h>
#include <eigen3/Eigen/Dense>


class PinocchioWrapper
{

public:

    PinocchioWrapper(urdf::ModelInterfaceSharedPtr urdf);

    void update(const Eigen::VectorXd& q);
    Eigen::Affine3d getPose(std::string link);
    Eigen::MatrixXd getJacobian(std::string link);
    int nq();

    ~PinocchioWrapper();

private:

    class Impl;
    Impl * impl;
};

#endif // PINOCCHIOWRAPPER_H
