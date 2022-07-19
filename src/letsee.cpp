#include "casadi_kin_dyn/casadi_collision_handler.h"
#include <fstream>
#include <iostream>

int main()
{

    using namespace casadi_kin_dyn;

    std::string urdf;
    {
        std::fstream fs("/home/arturo/code/robots_ws/src/iit-teleop-ros-pkg/teleop_urdf/urdf/teleop_capsules.rviz");
        std::ostringstream os;
        os << fs.rdbuf();
        urdf = os.str();
    }

    std::string srdf;
    {
        std::fstream fs("/home/arturo/code/robots_ws/src/iit-teleop-ros-pkg/teleop_srdf/srdf/teleop_capsules.srdf");
        std::ostringstream os;
        os << fs.rdbuf();
        srdf = os.str();
    }

    auto kd = std::make_shared<CasadiKinDyn>(urdf);

    CasadiCollisionHandler ch(kd, srdf);

    ch.addShape("mysphere",
                "sphere",
                {
                    {"radius", {0.2}},
                    {"position", {0, 0, 0.8}}
                });

    ch.addShape("mycapsule",
                "capsule",
                {
                    {"radius", {0.2}},
                    {"length", {0.6}},
                    {"position", {0.2, 0, 0.8}},
                    {"add_to_all", {0}}
                });

    Eigen::VectorXd q;
    q.setZero(5);
    q << 0, 1, 1, 0, 0;

    Eigen::VectorXd d;
    d.setZero(ch.numPairs());

    ch.distance(q, d);

    Eigen::MatrixXd J;
    J.setZero(d.size(), q.size());
    ch.distanceJacobian(q, J);

    Eigen::MatrixXd Jfd(J);
    for(int i = 0; i < q.size(); i++)
    {
        Eigen::VectorXd dq = 1e-6*Eigen::VectorXd::Unit(q.size(), i)/2.0;
        Eigen::VectorXd d1(d.size()), d2(d.size());
        ch.distance(q + dq, d2);
        ch.distance(q - dq, d1);
        Jfd.col(i) = (d2 - d1)/1e-6;
    }

    std::cout << "J error is " << (J - Jfd).lpNorm<Eigen::Infinity>() << "\n";

    std::cout << "J   = \n" << J.format(3) << "\n\n";
    std::cout << "Jfd = \n" << Jfd.format(3) << "\n\n";
    std::cout << "Je  = \n" << ((J - Jfd)).format(3) << "\n\n";
}
