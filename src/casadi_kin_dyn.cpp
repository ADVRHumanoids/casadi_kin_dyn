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
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/energy.hpp>
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/algorithm/aba.hpp>

#include <urdf_parser/urdf_parser.h>



namespace casadi_kin_dyn
{

class CasadiKinDyn::Impl
{

public:

    Impl(urdf::ModelInterfaceSharedPtr urdf_model,
         bool verbose,
         std::map<std::string, double> fixed_joints);

    int nq() const;
    int nv() const;
    std::vector<double> q_min() const;
    std::vector<double> q_max() const;
    std::vector<std::string> joint_names() const;
    double mass() const;

    Eigen::VectorXd mapToQ(std::map<std::string, double> jmap);

    Eigen::VectorXd mapToV(std::map<std::string, double> jmap);

    Eigen::VectorXd getMinimalQ(Eigen::VectorXd q);

    casadi::Function integrate();

    casadi::Function qdot();

    casadi::Function rnea();

    casadi::Function computeCentroidalDynamics();

    casadi::Function ccrba();

    casadi::Function fk(std::string link_name);

    casadi::Function centerOfMass();

    casadi::Function jacobian(std::string link_name, ReferenceFrame ref);

    casadi::Function frameVelocity(std::string link_name, ReferenceFrame ref);

    casadi::Function frameAcceleration(std::string link_name, ReferenceFrame ref);

    casadi::Function crba();

    casadi::Function kineticEnergy();

    casadi::Function potentialEnergy();

    casadi::Function aba();

    pinocchio::Model model() const;

    std::string urdf;


private:

    typedef casadi::SX Scalar;
    typedef Eigen::Matrix<Scalar, -1, 1>  VectorXs;
    typedef Eigen::Matrix<Scalar, -1, -1> MatrixXs;

    static VectorXs cas_to_eig(const casadi::SX& cas);
    static casadi::SX eig_to_cas(const VectorXs& eig);
    static casadi::SX eigmat_to_cas(const MatrixXs& eig);

    pinocchio::Model _model_dbl;
    casadi::SX _q, _qdot, _qddot, _tau;
    std::vector<double> _q_min, _q_max;
    urdf::ModelInterfaceSharedPtr _urdf;

};

CasadiKinDyn::Impl::Impl(urdf::ModelInterfaceSharedPtr urdf_model,
                         bool verbose,
                         std::map<std::string, double> fixed_joints):
    _urdf(urdf_model)
{
    // parse pinocchio model from urdf
    pinocchio::Model model_full;
    pinocchio::urdf::buildModel(urdf_model, model_full, verbose);

    // reduce model
    std::vector<pinocchio::JointIndex> joints_to_lock;
    auto joint_pos = pinocchio::neutral(model_full);

    for(auto [jname, jpos] : fixed_joints)
    {
        if(!model_full.existJointName(jname))
        {
            throw std::invalid_argument("joint does not exist (" + jname + ")");
        }

        size_t jidx = model_full.getJointId(jname);
        size_t qidx = model_full.idx_qs[jidx];
        size_t nq = model_full.nqs[jidx];
        if(nq != 1)
        {
            throw std::invalid_argument("only 1-dof fixed joints are supported (" + jname + ")");
        }

        joints_to_lock.push_back(jidx);
        joint_pos[qidx] = jpos;

    }

    pinocchio::buildReducedModel(model_full, joints_to_lock, joint_pos, _model_dbl);

    // create symsfixed_joints
    _q = casadi::SX::sym("q", _model_dbl.nq);
    _qdot = casadi::SX::sym("v", _model_dbl.nv);
    _qddot = casadi::SX::sym("a", _model_dbl.nv);
    _tau = casadi::SX::sym("tau", _model_dbl.nv);

    _q_min.resize(_model_dbl.lowerPositionLimit.size());
    for(unsigned int i = 0; i < _model_dbl.lowerPositionLimit.size(); ++i)
        _q_min[i] = _model_dbl.lowerPositionLimit[i];

    _q_max.resize(_model_dbl.upperPositionLimit.size());
    for(unsigned int i = 0; i < _model_dbl.upperPositionLimit.size(); ++i)
        _q_max[i] = _model_dbl.upperPositionLimit[i];

}

double CasadiKinDyn::Impl::mass() const
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    Scalar M = pinocchio::computeTotalMass(model, data);

    return double(M);

}

std::vector<double> CasadiKinDyn::Impl::q_min() const
{
    return _q_min;
}

std::vector<double> CasadiKinDyn::Impl::q_max() const
{
    return _q_max;
}

std::vector<std::string> CasadiKinDyn::Impl::joint_names() const
{
    return _model_dbl.names;
}

Eigen::VectorXd CasadiKinDyn::Impl::mapToQ(std::map<std::string, double> jmap)
{
    auto joint_pos = pinocchio::neutral(_model_dbl);

    for(auto [jname, jpos] : jmap)
    {
        if(!_model_dbl.existJointName(jname))
        {
            throw std::invalid_argument("joint does not exist (" + jname + ")");
        }

        size_t jidx = _model_dbl.getJointId(jname);
        size_t qidx = _model_dbl.idx_qs[jidx];
        size_t nq = _model_dbl.nqs[jidx];

        if(nq == 2)
        {
            // throw std::invalid_argument("only 1-dof joints are supported (" + jname + ")");
            joint_pos[qidx] = cos(jpos);
            joint_pos[qidx+1] = sin(jpos);
        }
        else
        {
            joint_pos[qidx] = jpos;
        }



    }

    return joint_pos;
}

Eigen::VectorXd CasadiKinDyn::Impl::mapToV(std::map<std::string, double> jmap)
{
    auto joint_vel = Eigen::VectorXd::Zero(nv()).eval();

    for(auto [jname, jvel] : jmap)
    {
        if(!_model_dbl.existJointName(jname))
        {
            throw std::invalid_argument("joint does not exist (" + jname + ")");
        }

        size_t jidx = _model_dbl.getJointId(jname);
        size_t vidx = _model_dbl.idx_vs[jidx];
        size_t nv = _model_dbl.nvs[jidx];

        if(nv != 1)
        {
            throw std::invalid_argument("only 1-dof joints are supported (" + jname + ")");
        }

        joint_vel[vidx] = jvel;

    }

    return joint_vel;
}

Eigen::VectorXd CasadiKinDyn::Impl::getMinimalQ(Eigen::VectorXd q)
{

    // add guards if q input by user is not of dimension nq()
    auto model = _model_dbl.cast<Scalar>();
    int reduced_size = 0;

    for(int n_joint = 0; n_joint < model.njoints; n_joint++)
    {
        int nq = model.nqs[n_joint];

        if(nq==2)
        {
            reduced_size++;
        }
        else
        {
            reduced_size+=nq;
        }
    }

    auto q_minimal = Eigen::VectorXd::Zero(reduced_size).eval();

    int i = 0;
    int j = 0;
    for(int n_joint = 0; n_joint < model.njoints; n_joint++)
    {
        int nq = model.nqs[n_joint];

        if(nq == 0)
        {
            continue;
        }

        if(nq == 7)
        {
            for (int k = 0; k < 7; k++)
            {
                q_minimal[j+k] = q[i+k];
            }
            j+=6;
        }

        if(nq == 1)
        {
            q_minimal[j] = q[i];
        }

        if(nq == 2)
        {
            q_minimal[j] = atan2(q[i], q[i+1]);
        }
        j++;
        i+=nq;
    }

    return q_minimal;

}

casadi::Function CasadiKinDyn::Impl::integrate()
{
    auto model = _model_dbl.cast<Scalar>();
    auto qnext = pinocchio::integrate(model, cas_to_eig(_q), cas_to_eig(_qdot));

    casadi::Function integrate("integrate",
                               {_q, _qdot}, {eig_to_cas(qnext)},
                               {"q", "v"}, {"qnext"});

    return integrate;
}

casadi::Function CasadiKinDyn::Impl::qdot()
{
    auto model = _model_dbl.cast<Scalar>();
    Eigen::Matrix<Scalar, -1, 1> qdot(model.nq);

    auto qeig = cas_to_eig(_q);
    auto veig = cas_to_eig(_qdot);

    for(int i = 0; i < model.njoints; i++)
    {
        int nq = model.nqs[i];

        if(nq == 0)
        {
            continue;
        }

        auto jname = model.names[i];
        auto uj = _urdf->getJoint(jname);
        int iv = model.idx_vs[i];
        int iq = model.idx_qs[i];

        if(nq == 7)
        {
            Eigen::Quaternion<Scalar> quat(
                        qeig[iq+6],
                        qeig[iq+3],
                        qeig[iq+4],
                        qeig[iq+5]);

            Eigen::Quaternion<Scalar> qomega(
                        0,
                        veig[iv+3],
                        veig[iv+4],
                        veig[iv+5]);

            Eigen::Matrix<Scalar, 3, 1> pdot = quat * veig.segment<3>(iv);

            Eigen::Matrix<Scalar, 4, 1> quat_dot = 0.5 * (quat * qomega).coeffs();

            qdot.segment<3>(iq) = pdot;

            qdot.segment<4>(iq+3) = quat_dot;

        }
        else if(nq == 2)
        {
            qdot[iq] = -veig[iv]*qeig[iq+1]; // cos
            qdot[iq+1] = veig[iv]*qeig[iq]; // sin
        }
        else if(nq == 1)
        {
            qdot[iq] = veig[iv];
        }
        else
        {
            throw std::runtime_error("invalid nq: " + std::to_string(nq));
        }
    }

    return casadi::Function("qdot", {_q, _qdot}, {eig_to_cas(qdot)},
                            {"q", "v"}, {"qdot"});
}

int CasadiKinDyn::Impl::nq() const
{
    return _model_dbl.nq;
}

int CasadiKinDyn::Impl::nv() const
{
    return _model_dbl.nv;
}

casadi::Function CasadiKinDyn::Impl::kineticEnergy()
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);


    Scalar DT = pinocchio::computeKineticEnergy(model, data, cas_to_eig(_q), cas_to_eig(_qdot));

    casadi::Function KINETICENERGY("kineticEnergy",
                                   {_q, _qdot}, {DT},
                                   {"q", "v"}, {"DT"});

    return KINETICENERGY;
}

casadi::Function CasadiKinDyn::Impl::potentialEnergy()
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);


    Scalar DU = pinocchio::computePotentialEnergy(model, data, cas_to_eig(_q));

    casadi::Function POTENTIALENERGY("potentialEnergy",
                                     {_q}, {DU},
                                     {"q"}, {"DU"});

    return POTENTIALENERGY;
}

casadi::Function CasadiKinDyn::Impl::aba()
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);


    pinocchio::aba(model, data, cas_to_eig(_q), cas_to_eig(_qdot), cas_to_eig(_tau));


    auto ddq = eig_to_cas(data.ddq);
    casadi::Function FD("rnea",
                        {_q, _qdot, _tau}, {ddq},
                        {"q", "v", "tau"}, {"a"});

    return FD;
}

pinocchio::Model CasadiKinDyn::Impl::model() const
{
    return _model_dbl;
}


casadi::Function CasadiKinDyn::Impl::rnea()
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);


    pinocchio::rnea(model, data, cas_to_eig(_q), cas_to_eig(_qdot), cas_to_eig(_qddot));


    auto tau = eig_to_cas(data.tau);
    casadi::Function ID("rnea",
                        {_q, _qdot, _qddot}, {tau},
                        {"q", "v", "a"}, {"tau"});

    return ID;
}

casadi::Function CasadiKinDyn::Impl::computeCentroidalDynamics()
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    pinocchio::computeCentroidalMomentumTimeVariation(model, data,
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

    return CD;
}

casadi::Function CasadiKinDyn::Impl::ccrba()
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    auto Ah = pinocchio::ccrba(model, data, cas_to_eig(_q), cas_to_eig(casadi::SX::zeros(nv())));
    auto Ah_cas = eigmat_to_cas(Ah);

    casadi::Function CCRBA("ccrba",
                        {_q}, {Ah_cas},
                        {"q"}, {"A"});

    return CCRBA;

}

casadi::Function CasadiKinDyn::Impl::frameVelocity(std::string link_name, ReferenceFrame ref)
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    auto frame_idx = model.getFrameId(link_name);

    // Compute expression for forward kinematics with Pinocchio
    Eigen::Matrix<Scalar, 6, -1> J;
    J.setZero(6, nv());

    pinocchio::computeJointJacobians(model, data, cas_to_eig(_q));
    //pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q));
    pinocchio::getFrameJacobian(model, data, frame_idx, pinocchio::ReferenceFrame(ref), J);



    Eigen::Matrix<Scalar, 6, 1> eig_vel = J*cas_to_eig(_qdot);


    auto ee_vel_linear = eig_to_cas(eig_vel.head(3));
    auto ee_vel_angular = eig_to_cas(eig_vel.tail(3));

    casadi::Function FRAME_VELOCITY("frame_velocity",
                                    {_q, _qdot}, {ee_vel_linear, ee_vel_angular},
                                    {"q", "qdot"}, {"ee_vel_linear", "ee_vel_angular"});

    return FRAME_VELOCITY;
}

casadi::Function CasadiKinDyn::Impl::frameAcceleration(std::string link_name, ReferenceFrame ref)
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    auto frame_idx = model.getFrameId(link_name);

    // Compute expression for forward kinematics with Pinocchio
    Eigen::Matrix<Scalar, 6, -1> J, Jdot;
    J.setZero(6, nv());
    Jdot.setZero(6, nv());

    pinocchio::computeJointJacobians(model, data, cas_to_eig(_q));
    pinocchio::computeJointJacobiansTimeVariation(model, data, cas_to_eig(_q), cas_to_eig(_qdot));
    //pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q));


    pinocchio::getFrameJacobian(model, data, frame_idx, pinocchio::ReferenceFrame(ref), J);
    pinocchio::getFrameJacobianTimeVariation(model, data, frame_idx, pinocchio::ReferenceFrame(ref), Jdot);

    Eigen::Matrix<Scalar, 6, 1> eig_acc = J*cas_to_eig(_qddot) + Jdot*cas_to_eig(_qdot);

    auto ee_acc_linear = eig_to_cas(eig_acc.head(3));
    auto ee_acc_angular = eig_to_cas(eig_acc.tail(3));

    casadi::Function FRAME_ACCEL("frame_acceleration",
                                    {_q, _qdot, _qddot}, {ee_acc_linear, ee_acc_angular},
                                    {"q", "qdot", "qddot"}, {"ee_acc_linear", "ee_acc_angular"});

    return FRAME_ACCEL;
}

casadi::Function CasadiKinDyn::Impl::fk(std::string link_name)
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q));

    auto frame_idx = model.getFrameId(link_name);
    auto eig_fk_pos = data.oMf.at(frame_idx).translation();
    auto eig_fk_rot = data.oMf.at(frame_idx).rotation();
    auto ee_pos = eig_to_cas(eig_fk_pos);
    auto ee_rot = eigmat_to_cas(eig_fk_rot);


    casadi::Function FK("forward_kinematics",
                        {_q}, {ee_pos, ee_rot},
                        {"q"}, {"ee_pos", "ee_rot"});


    return FK;
}


casadi::Function CasadiKinDyn::Impl::centerOfMass()
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

    return CoM;

}



casadi::Function CasadiKinDyn::Impl::jacobian(std::string link_name, ReferenceFrame ref)
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    auto frame_idx = model.getFrameId(link_name);

    // Compute expression for forward kinematics with Pinocchio
    Eigen::Matrix<Scalar, 6, -1> J;
    J.setZero(6, nv());

    pinocchio::computeJointJacobians(model, data, cas_to_eig(_q));
    pinocchio::framesForwardKinematics(model, data, cas_to_eig(_q));
    pinocchio::getFrameJacobian(model, data, frame_idx, pinocchio::ReferenceFrame(ref), J); //"LOCAL" DEFAULT PINOCCHIO COMPUTATION


    auto Jac = eigmat_to_cas(J);
    casadi::Function JACOBIAN("jacobian", {_q}, {Jac}, {"q"}, {"J"});

    return JACOBIAN;
}


casadi::Function CasadiKinDyn::Impl::crba()
{
    auto model = _model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    auto M = pinocchio::crba(model, data, cas_to_eig(_q));
    M.triangularView<Eigen::Lower>() = M.transpose();

    auto Inertia = eigmat_to_cas(M);
    casadi::Function INERTIA("crba", {_q}, {Inertia}, {"q"}, {"B"});

    return INERTIA;
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

CasadiKinDyn::CasadiKinDyn(std::string urdf_string,
                           bool verbose,
                           std::map<std::string, double> fixed_joints)
{
    auto urdf = urdf::parseURDF(urdf_string);
    _impl= std::make_unique<Impl>(urdf, verbose, fixed_joints);
    _impl->urdf = urdf_string;
}

CasadiKinDyn::CasadiKinDyn(const CasadiKinDyn &other)
{
    _impl = std::make_unique<Impl>(*other._impl);
}

int CasadiKinDyn::nq() const
{
    return impl().nq();
}

int CasadiKinDyn::nv() const
{
    return impl().nv();
}

Eigen::VectorXd CasadiKinDyn::mapToQ(std::map<std::string, double> jmap)
{
    return impl().mapToQ(jmap);
}

Eigen::VectorXd CasadiKinDyn::mapToV(std::map<std::string, double> jmap)
{
    return impl().mapToV(jmap);
}

Eigen::VectorXd CasadiKinDyn::getMinimalQ(Eigen::VectorXd q)
{
    return impl().getMinimalQ(q);
}

casadi::Function CasadiKinDyn::integrate()
{
    return impl().integrate();
}

casadi::Function CasadiKinDyn::qdot()
{
    return impl().qdot();
}

casadi::Function CasadiKinDyn::rnea()
{
    return impl().rnea();
}

casadi::Function CasadiKinDyn::computeCentroidalDynamics()
{
    return impl().computeCentroidalDynamics();
}

casadi::Function CasadiKinDyn::ccrba()
{
    return impl().ccrba();
}

casadi::Function CasadiKinDyn::crba()
{
    return impl().crba();
}

casadi::Function CasadiKinDyn::aba()
{
    return impl().aba();
}

casadi::Function CasadiKinDyn::fk(std::string link_name)
{
    return impl().fk(link_name);
}

casadi::Function CasadiKinDyn::frameVelocity(std::string link_name, ReferenceFrame ref)
{
    return impl().frameVelocity(link_name, ref);
}

casadi::Function CasadiKinDyn::frameAcceleration(std::string link_name, ReferenceFrame ref)
{
    return impl().frameAcceleration(link_name, ref);
}

casadi::Function CasadiKinDyn::centerOfMass()
{
    return impl().centerOfMass();
}

casadi::Function CasadiKinDyn::jacobian(std::string link_name, ReferenceFrame ref)
{
    return impl().jacobian(link_name, ref);
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

casadi::Function CasadiKinDyn::kineticEnergy()
{
    return impl().kineticEnergy();
}

casadi::Function CasadiKinDyn::potentialEnergy()
{
    return impl().potentialEnergy();
}

std::vector<double> CasadiKinDyn::q_min() const
{
    return impl().q_min();
}

std::vector<double> CasadiKinDyn::q_max() const
{
    return impl().q_max();
}

std::vector<std::string> CasadiKinDyn::joint_names() const
{
    return impl().joint_names();
}

double CasadiKinDyn::mass() const
{
    return impl().mass();
}

std::string CasadiKinDyn::urdf() const
{
    return impl().urdf;
}

std::any CasadiKinDyn::modelHandle() const
{
    return impl().model();
}

}
