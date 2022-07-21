#include "casadi_kin_dyn/casadi_collision_handler.h"

#define PINOCCHIO_WITH_HPP_FCL
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

#include <casadi/casadi.hpp>

namespace pin = pinocchio;

using namespace casadi_kin_dyn;

static CasadiCollisionHandler * main_collision_handler = nullptr;

namespace casadi_kin_dyn {

CasadiCollisionHandler * get_collision_handler()
{
    thread_local CasadiCollisionHandler * ch = nullptr;

    if(!main_collision_handler)
    {
        throw std::runtime_error("No CasadiCollisionHandler instance is alive");
    }

    if(!ch)
    {
        // note: we deliberately leak memory
        ch = new CasadiCollisionHandler(*main_collision_handler);
    }

    return ch;
}

}

class CasadiCollisionHandler::Impl
{

public:

    Impl(CasadiKinDyn::Ptr kd,
         std::string srdf_string);

    size_t numPairs() const;

    bool distance(Eigen::Ref<const Eigen::VectorXd> q,
                  Eigen::Ref<Eigen::VectorXd> d);

    bool distanceJacobian(Eigen::Ref<const Eigen::VectorXd> q,
                          Eigen::Ref<Eigen::MatrixXd> J);

    bool distanceHessian(Eigen::Ref<const Eigen::VectorXd> q,
                         Eigen::Ref<Eigen::MatrixXd> H);

    void addShape(std::string name,
                  std::string type,
                  ShapeParams params);

    casadi::Function getDistanceFunction();

    CasadiKinDyn::Ptr kd;

    ~Impl();

private:

    Eigen::VectorXd _last_q;
    Eigen::VectorXd _d_cached;

    pin::Model _mdl;
    pin::Data _data;
    pin::GeometryModel _geom_mdl;
    pin::GeometryData _geom_data;

    std::vector<Eigen::MatrixXd> _joint_J;

};

CasadiCollisionHandler::CasadiCollisionHandler(CasadiKinDyn::Ptr kd,
                                               std::string srdf_string)
{
    _impl = std::make_unique<Impl>(kd, srdf_string);

    main_collision_handler = this;
}

CasadiCollisionHandler::CasadiCollisionHandler(const CasadiCollisionHandler & other)
{
    _impl = std::make_unique<Impl>(*other._impl);
    _impl->kd = std::make_shared<CasadiKinDyn>(*_impl->kd);
}

CasadiKinDyn::Ptr CasadiCollisionHandler::kd()
{
    return impl().kd;
}

void CasadiCollisionHandler::addShape(std::string name,
                                      std::string type,
                                      CasadiCollisionHandler::ShapeParams params)
{
    return impl().addShape(name, type, params);
}

casadi::Function CasadiCollisionHandler::getDistanceFunction()
{
    return impl().getDistanceFunction();
}

size_t CasadiCollisionHandler::numPairs() const
{
    return impl().numPairs();
}

bool CasadiCollisionHandler::distance(Eigen::Ref<const Eigen::VectorXd> q,
                                      Eigen::Ref<Eigen::VectorXd> d)
{
    return impl().distance(q, d);
}

bool CasadiCollisionHandler::distanceJacobian(Eigen::Ref<const Eigen::VectorXd> q,
                                              Eigen::Ref<Eigen::MatrixXd> J)
{
    return impl().distanceJacobian(q, J);
}

CasadiCollisionHandler::~CasadiCollisionHandler()
{

}

CasadiCollisionHandler::Impl &CasadiCollisionHandler::impl()
{
    return *_impl;
}

const CasadiCollisionHandler::Impl &CasadiCollisionHandler::impl() const
{
    return *_impl;
}


CasadiCollisionHandler::Impl::Impl(CasadiKinDyn::Ptr _kd,
                                   std::string srdf_string):
    _mdl(std::any_cast<pin::Model>(_kd->modelHandle())),
    _data(_mdl),
    kd(_kd)
{
    std::istringstream urdf_stream(_kd->urdf());

    _geom_mdl = pin::urdf::buildGeom(_mdl,
                                     urdf_stream,
                                     pin::GeometryType::COLLISION,
                                     _geom_mdl);

    _geom_mdl.addAllCollisionPairs();

    pin::srdf::removeCollisionPairsFromXML(_mdl, _geom_mdl, srdf_string);

    _geom_data = pin::GeometryData(_geom_mdl);

    _joint_J.assign(_mdl.njoints,
                    Eigen::MatrixXd::Zero(6, _mdl.nv));

}


size_t CasadiCollisionHandler::Impl::numPairs() const
{
    return _geom_mdl.collisionPairs.size();
}

bool CasadiCollisionHandler::Impl::distance(Eigen::Ref<const Eigen::VectorXd> q,
                                            Eigen::Ref<Eigen::VectorXd> d)
{
    if(q.size() != _mdl.nq ||
            d.size() != _geom_mdl.collisionPairs.size())
    {
        std::cerr << __func__ << ": wrong input size \n";
        return false;
    }

    if(_last_q.size() == 0 || (_last_q.cwiseNotEqual(q)).any())
    {

        auto tic = std::chrono::high_resolution_clock::now();

        pin::computeDistances(_mdl, _data,
                              _geom_mdl, _geom_data,
                              q);

        _last_q = q;

        auto toc = std::chrono::high_resolution_clock::now();

        auto dur_sec = std::chrono::duration<double>(toc - tic);

    }

    for(size_t k = 0; k < _geom_mdl.collisionPairs.size(); ++k)
    {
        const auto& cp = _geom_mdl.collisionPairs[k];
        const auto& dr = _geom_data.distanceResults[k];
        auto name_1 = _geom_mdl.geometryObjects[cp.first].name;
        auto name_2 = _geom_mdl.geometryObjects[cp.second].name;

        d[k] = dr.min_distance;
    }

    return true;
}

bool CasadiCollisionHandler::Impl::distanceJacobian(Eigen::Ref<const Eigen::VectorXd> q,
                                                    Eigen::Ref<Eigen::MatrixXd> J)
{
    if(q.size() != _mdl.nq ||
            J.rows() != _geom_mdl.collisionPairs.size() ||
            J.cols() != _mdl.nv)
    {
        std::cerr << __func__ << ": wrong input size \n";
        return false;
    }

    _d_cached.setZero(numPairs());
    if(!distance(q, _d_cached))
    {
        std::cerr << __func__ << ": distance computation failed \n";
        return false;
    }

    auto tic = std::chrono::high_resolution_clock::now();

    pin::computeJointJacobians(_mdl, _data);

    for(size_t i = 0; i <_mdl.njoints; i++)
    {
        _joint_J[i].setZero(6, _mdl.nv);

        pin::getJointJacobian(_mdl,
                              _data,
                              i,
                              pin::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                              _joint_J[i]);
    }

    for(size_t k = 0; k < _geom_mdl.collisionPairs.size(); ++k)
    {
        const auto& cp = _geom_mdl.collisionPairs[k];
        const auto& dr = _geom_data.distanceResults[k];

        auto& go_1 = _geom_mdl.geometryObjects[cp.first];
        auto& go_2 = _geom_mdl.geometryObjects[cp.second];

        auto name_1 = go_1.name;
        auto name_2 = go_2.name;

        size_t joint_1_id = _geom_mdl.geometryObjects[cp.first].parentJoint;
        size_t joint_2_id = _geom_mdl.geometryObjects[cp.second].parentJoint;

        if(joint_1_id > 0)
        {
            // joint 1 jacobian
            const auto& J_1 = _joint_J[joint_1_id];

            // witness point w.r.t. world
            Eigen::Vector3d w1 = dr.nearest_points[0];

            // translation
            Eigen::Vector3d r = w1 - _data.oMi[joint_1_id].translation();

            J.row(k) = -dr.normal.transpose()*J_1.topRows<3>();
            J.row(k) -= (r.cross(dr.normal)).transpose()*J_1.bottomRows<3>();
        }
        else
        {
            J.row(k).setZero();
        }

        if(joint_2_id > 0)
        {
            // joint 2 jacobian
            const auto & J_2 = _joint_J[joint_2_id];

            // witness point w.r.t. world
            Eigen::Vector3d w2 = dr.nearest_points[1];

            // translation
            Eigen::Vector3d r = w2 - _data.oMi[joint_2_id].translation();

            J.row(k) += dr.normal.transpose()*J_2.topRows<3>();
            J.row(k) += (r.cross(dr.normal)).transpose()*J_2.bottomRows<3>();
        }

    }

    auto toc = std::chrono::high_resolution_clock::now();

    auto dur_sec = std::chrono::duration<double>(toc - tic);

    if(!J.allFinite() || J.hasNaN())
    {
        std::cout << "bad values in distance jacobian: \n";

        std::cout << "q = " << q.transpose().format(3) << "\n";

        for(size_t k = 0; k < _geom_mdl.collisionPairs.size(); ++k)
        {
            if(!J.row(k).allFinite() || J.row(k).hasNaN())
            {
                std::cout << "at row " << k  << ": " <<
                             J.row(k).format(3) << "\n";
            }
        }
    }

    return true;
}

void CasadiCollisionHandler::Impl::addShape(std::string name,
                                            std::string type,
                                            ShapeParams params)
{
    pin::GeometryObject::CollisionGeometryPtr geom;

    if(type == "sphere")
    {
        geom.reset(new hpp::fcl::Sphere(params.at("radius")[0]));
    }
    else if(type == "capsule")
    {
        geom.reset(new hpp::fcl::Capsule(
                       params.at("radius")[0],
                   params.at("length")[0])
                );
    }
    else
    {
        throw std::invalid_argument(
                    "invalid type " + type);
    }

    pin::SE3 placement;

    placement.setIdentity();

    if(params.count("position"))
    {
        auto pos = params.at("position");
        placement.translation() << pos[0], pos[1], pos[2];
    }

    if(params.count("orientation"))
    {
        auto rot = params.at("orientation");

        if(rot.size() != 4)
        {
            throw std::invalid_argument("orientation must be a quaternion");
        }

        Eigen::Quaterniond quat(rot.data());
        quat.normalize();

        placement.rotation() = quat.toRotationMatrix();
    }


    pin::GeometryObject go(
                name,
                0,
                0,
                geom,
                placement);

    go.meshPath = type;

    size_t id = _geom_mdl.addGeometryObject(go);

    for(size_t k = 0; k < id; k++)
    {
        if(_geom_mdl.geometryObjects[k].parentJoint != 0)
        {
            _geom_mdl.addCollisionPair(pin::CollisionPair(k, id));
        }
    }

    _geom_data = pin::GeometryData(_geom_mdl);


}

casadi::Function CasadiCollisionHandler::Impl::getDistanceFunction()
{
    return casadi::external("collision_distance",
                            "libcasadi_compute_distance.so");
}

CasadiCollisionHandler::Impl::~Impl()
{
    main_collision_handler = nullptr;
}
