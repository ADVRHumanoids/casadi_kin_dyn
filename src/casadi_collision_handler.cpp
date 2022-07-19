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

casadi_kin_dyn::CasadiCollisionHandler * __collision_handler_ptr = nullptr;


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

private:

    pin::Model _mdl;
    pin::Data _data;
    pin::GeometryModel _geom_mdl;
    pin::GeometryData _geom_data;

    Eigen::MatrixXd J_1, J_2, J_12;
    std::vector<Eigen::MatrixXd> _joint_J;

};

CasadiCollisionHandler::CasadiCollisionHandler(CasadiKinDyn::Ptr kd,
                                               std::string srdf_string)
{
    _impl = std::make_unique<Impl>(kd, srdf_string);

    __collision_handler_ptr = this;
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
        return false;
    }

    auto tic = std::chrono::high_resolution_clock::now();

    pin::computeDistances(_mdl, _data,
                          _geom_mdl, _geom_data,
                          q);

    auto toc = std::chrono::high_resolution_clock::now();

    auto dur_sec = std::chrono::duration<double>(toc - tic);

    std::cout << _geom_mdl.collisionPairs.size() << " distances computed in " <<
                 dur_sec.count()*1e6 << " us \n";

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
    // d = |c2 - c1| - r2 - r1
    // n = (c2 - c1)/|c2 - c1|
    // w1 = c1 + n*r1
    // w2 = c2 - n*r2
    // d = |c2 -n*r2 - c1 - n*r1| = |w2 - w1|

    // w1_loc = n*r1
    // dn/dc2 =

    if(q.size() != _mdl.nq ||
            J.rows() != _geom_mdl.collisionPairs.size() ||
            J.cols() != _mdl.nv)
    {
        return false;
    }

    auto tic = std::chrono::high_resolution_clock::now();

    // note: assume distance has been called with this q vector
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

        J_12.setZero(3, _mdl.nv);

        if(joint_1_id > 0)
        {
            // joint 1 jacobian
            J_1 = _joint_J[joint_1_id];

            // witness point w.r.t. world
            Eigen::Vector3d w1 = dr.nearest_points[0];

            // translation
            pin::SE3 J1_transl;
            J1_transl.setIdentity();
            J1_transl.translation() = (w1 - _data.oMi[joint_1_id].translation());

            pin::details::translateJointJacobian(J1_transl,
                                                 J_1, J_1);

            J_12 = -J_1.topRows<3>();
        }

        if(joint_2_id > 0)
        {
            // joint 2 jacobian
            J_2 = _joint_J[joint_2_id];

            // witness point w.r.t. world
            Eigen::Vector3d w2 = dr.nearest_points[1];

            // translation
            pin::SE3 J2_transl;
            J2_transl.setIdentity();
            J2_transl.translation() = (w2 - _data.oMi[joint_2_id].translation());

            pin::details::translateJointJacobian(J2_transl,
                                                 J_2, J_2);

            J_12 += J_2.topRows<3>();
        }

        J.row(k) = dr.normal.transpose() * J_12;

    }

    auto toc = std::chrono::high_resolution_clock::now();

    auto dur_sec = std::chrono::duration<double>(toc - tic);

    std::cout << _geom_mdl.collisionPairs.size() << " distance jacobian computed in " <<
                 dur_sec.count()*1e6 << " us \n";

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
