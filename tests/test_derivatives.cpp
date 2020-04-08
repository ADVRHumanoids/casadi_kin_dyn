#include <algorithm>
#include <gtest/gtest.h>
#include <fstream>
#include "PinocchioWrapper.h"

class TestDerivatives: public ::testing::Test {


public:

    TestDerivatives()
    {
        std::ifstream ifs("/home/arturo/Code/advr-superbuild/robots/iit-teleop-ros-pkg/teleop_urdf/urdf/teleop.urdf");
        std::string urdf_string( (std::istreambuf_iterator<char>(ifs) ),
                             (std::istreambuf_iterator<char>()    ) );

        auto urdf = urdf::parseURDF(urdf_string);
        _pin = new PinocchioWrapper(urdf);

    }

    virtual ~TestDerivatives()
    {
        delete _pin;
    }

    virtual void SetUp()
    {

    }

    virtual void TearDown()
    {

    }

protected:

    PinocchioWrapper * _pin;

};

TEST_F(TestDerivatives, checkJac)
{
    std::string link_name = "TCP";

    Eigen::VectorXd q;
    q.setRandom(_pin->nq());

    _pin->update(q);
    auto T =  _pin->getPose(link_name);
    Eigen::Vector3d p = T.translation();
    Eigen::Matrix3d R = T.linear();

    auto J = _pin->getJacobian(link_name);
    Eigen::MatrixXd Jhat = 0*J;



    /* Compute numerical jacobian matrix */

    double step_size = 1e-5;
    Eigen::VectorXd q1, q2;

    for( int i = 0; i < _pin->nq(); i++ )
    {

        q2 = q;
        q2(i) += step_size/2;

        q1 = q;
        q1(i) -= step_size/2;

        _pin->update(q1);
        auto T1 = _pin->getPose(link_name);
        Eigen::Vector3d p1 = T1.translation();
        Eigen::Matrix3d R1 = T1.linear();

        _pin->update(q2);
        auto T2 = _pin->getPose(link_name);
        Eigen::Vector3d p2 = T2.translation();
        Eigen::Matrix3d R2 = T2.linear();

        Jhat.col(i).head(3) = (p2-p1)/step_size;
        Eigen::Matrix3d S = (R2-R1)*R.transpose();
        Jhat(3,i) = S(2,1)/step_size;
        Jhat(4,i) = S(0,2)/step_size;
        Jhat(5,i) = S(1,0)/step_size;

    }

    EXPECT_LE((J-Jhat).cwiseAbs().maxCoeff(), 1e-5);

    std::cout << "J:\n" << J << std::endl;
    std::cout << "Jhat:\n" << Jhat << std::endl;



}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

