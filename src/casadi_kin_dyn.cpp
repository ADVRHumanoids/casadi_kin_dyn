#include <casadi_kin_dyn/casadi_kin_dyn.h>

#include <casadi/casadi.hpp>

#define PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>

#include <urdf_parser/urdf_parser.h>

namespace casadi_kin_dyn
{

class CasadiKinDyn::Impl
{

public:

    Impl(urdf::ModelInterfaceSharedPtr urdf_model);

    int nq() const;
    int nv() const;

    std::string rnea();
    
    std::string computeCentroidalDynamics();

    std::string ccrba();

    std::string fk(std::string link_name);
    
    std::string centerOfMass();

    std::string jacobian(std::string link_name);

    std::string frameVelocity(std::string link_name);

    std::string frameAcceleration(std::string link_name);

    std::string crba();

private:

    typedef casadi::SX Scalar;
    typedef Eigen::Matrix<Scalar, -1, 1>  VectorXs;
    typedef Eigen::Matrix<Scalar, -1, -1> MatrixXs;

    static VectorXs cas_to_eig(const casadi::SX& cas);
    static casadi::SX eig_to_cas(const VectorXs& eig);
    static casadi::SX eigmat_to_cas(const MatrixXs& eig);

    pinocchio::Model _model_dbl;
    casadi::SX _q, _qdot, _qddot, _tau;


};

CasadiKinDyn::Impl::Impl(urdf::ModelInterfaceSharedPtr urdf_model)
{
    pinocchio::urdf::buildModel(urdf_model, _model_dbl, true); // verbose
    _q = casadi::SX::sym("q", _model_dbl.nq);
    _qdot = casadi::SX::sym("v", _model_dbl.nv);
    _qddot = casadi::SX::sym("a", _model_dbl.nv);
    _tau = casadi::SX::sym("tau", _model_dbl.nv);

}

int CasadiKinDyn::Impl::nq() const
{
    return _model_dbl.nq;
}

int CasadiKinDyn::Impl::nv() const
{
    return _model_dbl.nv;
}

std::string CasadiKinDyn::Impl::rnea()
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    pinocchio::rnea(model, data,
                    cas_to_eig(_q),
                    cas_to_eig(_qdot),
                    cas_to_eig(_qddot));

    auto tau = eig_to_cas(data.tau);
    casadi::Function ID("rnea",
    {_q, _qdot, _qddot}, {tau},
    {"q", "v", "a"}, {"tau"});

    std::stringstream ss;
    ss << ID.serialize();

    return ss.str();
}

std::string CasadiKinDyn::Impl::computeCentroidalDynamics()
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    pinocchio::computeCentroidalDynamics(model, data,
                                         cas_to_eig(_q),
                                         cas_to_eig(_qdot),
                                         cas_to_eig(_qddot));

    auto h_lin = eig_to_cas(data.hg.linear());
    auto h_ang = eig_to_cas(data.hg.angular());
    auto dh_lin = eig_to_cas(data.dhg.linear());
    auto dh_ang = eig_to_cas(data.dhg.angular());
    casadi::Function CD("computeCentroidalDynamics",
    {_q, _qdot, _qddot}, {h_lin, h_ang, dh_lin, dh_ang},
    {"q", "v", "a"}, {"h_lin", "h_ang", "dh_lin", "dh_ang"});

    std::stringstream ss;
    ss << CD.serialize();

    return ss.str();
}

std::string CasadiKinDyn::Impl::ccrba()
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    auto Ah = pinocchio::ccrba(model, data, cas_to_eig(_q), cas_to_eig(casadi::SX::zeros(nv())));
    auto Ah_cas = eigmat_to_cas(Ah);

    casadi::Function FK("ccrba",
    {_q}, {Ah_cas},
    {"q"}, {"A"});

    std::stringstream ss;
    ss << FK.serialize();

    return ss.str();

}

std::string CasadiKinDyn::Impl::frameVelocity(std::string link_name)
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    pinocchio::forwardKinematics(model, data,
                                 cas_to_eig(_q),
                                 cas_to_eig(_qdot));

    pinocchio::updateFramePlacements(model, data);

    auto frame_idx = model.getFrameId(link_name);

    auto vel = pinocchio::getFrameVelocity(model, data, frame_idx);
    auto eig_vel_linear = vel.linear();
    auto eig_vel_angular = vel.angular();
    auto ee_vel_linear = eig_to_cas(eig_vel_linear);
    auto ee_vel_angular = eig_to_cas(eig_vel_angular);

    casadi::Function FRAME_VELOCITY("frame_velocity",
    {_q, _qdot}, {ee_vel_linear, ee_vel_angular},
    {"q", "qdot"}, {"ee_vel_linear", "ee_vel_angular"});

    std::stringstream ss;
    ss << FRAME_VELOCITY.serialize();

    return ss.str();
}

std::string CasadiKinDyn::Impl::frameAcceleration(std::string link_name)
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    pinocchio::forwardKinematics(model, data,
                                 cas_to_eig(_q),
                                 cas_to_eig(_qdot),
                                 cas_to_eig(_qddot));

    pinocchio::updateFramePlacements(model, data);

    auto frame_idx = model.getFrameId(link_name);

    auto acc = pinocchio::getFrameAcceleration(model, data, frame_idx);
    auto eig_acc_linear = acc.linear();
    auto eig_acc_angular = acc.angular();
    auto ee_acc_linear = eig_to_cas(eig_acc_linear);
    auto ee_acc_angular = eig_to_cas(eig_acc_angular);

    casadi::Function FRAME_ACCELERATION("frame_acceleration",
    {_q, _qdot, _qddot}, {ee_acc_linear, ee_acc_angular},
    {"q", "qdot", "qddot"}, {"ee_acc_linear", "ee_acc_angular"});


    std::stringstream ss;
    ss << FRAME_ACCELERATION.serialize();

    return ss.str();
}

std::string CasadiKinDyn::Impl::fk(std::string link_name)
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    pinocchio::framesForwardKinematics(model, data,
                                       cas_to_eig(_q));


    auto frame_idx = model.getFrameId(link_name);
    auto eig_fk_pos = data.oMf.at(frame_idx).translation();
    auto eig_fk_rot = data.oMf.at(frame_idx).rotation();
    auto ee_position = eig_to_cas(eig_fk_pos);
    auto ee_rot = eigmat_to_cas(eig_fk_rot);


    casadi::Function FK("forward_kinematics",
    {_q}, {ee_position, ee_rot},
    {"q", "qdot", "qddot"}, {"ee_pos", "ee_rot"});


    std::stringstream ss;
    ss << FK.serialize();

    return ss.str();
}


std::string CasadiKinDyn::Impl::centerOfMass()
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    pinocchio::centerOfMass(model, data,
                            cas_to_eig(_q),
                            cas_to_eig(_qdot),
                            cas_to_eig(_qddot));

    auto com = eig_to_cas(data.com[0]);
    auto vcom = eig_to_cas(data.vcom[0]);
    auto acom = eig_to_cas(data.acom[0]);
    casadi::Function CoM("centerOfMass",
    {_q, _qdot, _qddot}, {com, vcom, acom},
    {"q", "v", "a"}, {"com", "vcom", "acom"});

    std::stringstream ss;
    ss << CoM.serialize();

    return ss.str();
    
}



std::string CasadiKinDyn::Impl::jacobian(std::string link_name)
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    auto frame_idx = model.getFrameId(link_name);

    // Compute expression for forward kinematics with Pinocchio
    Eigen::Matrix<Scalar, 6, -1> J;
    J.setZero(6, nv());

    pinocchio::computeJointJacobians(model, data, cas_to_eig(_q));
    pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q));
    pinocchio::getFrameJacobian(model, data, frame_idx, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

    auto Jac = eigmat_to_cas(J);
    casadi::Function JACOBIAN("jacobian", {_q}, {Jac}, {"q"}, {"J"});

    std::stringstream ss;
    ss << JACOBIAN.serialize();

    return ss.str();
}

std::string CasadiKinDyn::Impl::crba()
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    auto M = pinocchio::crba(model, data, cas_to_eig(_q));
    M.triangularView<Eigen::Lower>() = M.transpose();

    auto Inertia = eigmat_to_cas(M);
    casadi::Function INERTIA("crba", {_q}, {Inertia}, {"q"}, {"B"});

    std::stringstream ss;
    ss << INERTIA.serialize();

    return ss.str();
}

CasadiKinDyn::Impl::VectorXs CasadiKinDyn::Impl::cas_to_eig(const casadi::SX & cas)
{
    VectorXs eig(cas.size1());
    for(int i = 0; i < eig.size(); i++)
    {
        eig(i) = cas(i);
    }
    return eig;
}

casadi::SX CasadiKinDyn::Impl::eig_to_cas(const CasadiKinDyn::Impl::VectorXs & eig)
{
    auto sx = casadi::SX(casadi::Sparsity::dense(eig.size()));
    for(int i = 0; i < eig.size(); i++)
    {
        sx(i) = eig(i);
    }
    return sx;

}

casadi::SX CasadiKinDyn::Impl::eigmat_to_cas(const CasadiKinDyn::Impl::MatrixXs & eig)
{
    auto sx = casadi::SX(casadi::Sparsity::dense(eig.rows(), eig.cols()));
    for(int i = 0; i < eig.rows(); i++)
    {
        for(int j = 0; j < eig.cols(); j++)
        {
            sx(i,j) = eig(i,j);
        }
    }
    return sx;

}

CasadiKinDyn::CasadiKinDyn(std::string urdf_string)
{
    auto urdf = urdf::parseURDF(urdf_string);
    _impl.reset(new Impl(urdf));
}

int CasadiKinDyn::nq() const
{
    return impl().nq();
}

int CasadiKinDyn::nv() const
{
    return impl().nv();
}

std::string CasadiKinDyn::rnea()
{
    return impl().rnea();
}

std::string CasadiKinDyn::computeCentroidalDynamics()
{
    return impl().computeCentroidalDynamics();
}

std::string CasadiKinDyn::ccrba()
{
    return impl().ccrba();
}

std::string CasadiKinDyn::crba()
{
    return impl().crba();
}

std::string CasadiKinDyn::fk(std::string link_name)
{
    return impl().fk(link_name);
}

std::string CasadiKinDyn::frameVelocity(std::string link_name)
{
    return impl().frameVelocity(link_name);
}

std::string CasadiKinDyn::frameAcceleration(std::string link_name)
{
    return impl().frameAcceleration(link_name);
}

std::string CasadiKinDyn::centerOfMass()
{
    return impl().centerOfMass();
}

std::string CasadiKinDyn::jacobian(std::string link_name)
{
    return impl().jacobian(link_name);
}

CasadiKinDyn::~CasadiKinDyn()
{

}

const CasadiKinDyn::Impl & CasadiKinDyn::impl() const
{
    return *_impl;
}

CasadiKinDyn::Impl & CasadiKinDyn::impl()
{
    return *_impl;
}

}
