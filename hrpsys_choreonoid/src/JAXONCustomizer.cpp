#include <stdlib.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <yaml-cpp/yaml.h>

#define CNOID_BODY_CUSTOMIZER
#ifdef CNOID_BODY_CUSTOMIZER
#include <cnoid/BodyCustomizerInterface>
#else
#include <BodyCustomizerInterface.h>
#endif

#include <iostream>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT
#endif /* Windows */

#if defined(HRPMODEL_VERSION_MAJOR) && defined(HRPMODEL_VERSION_MINOR)
#if HRPMODEL_VERSION_MAJOR >= 3 && HRPMODEL_VERSION_MINOR >= 1
#include <hrpUtil/Tvmet3dTypes.h>
#define NS_HRPMODEL hrp
#endif
#endif

#ifdef CNOID_BODY_CUSTOMIZER
#define NS_HRPMODEL cnoid
cnoid::Matrix3 trans(const cnoid::Matrix3& M) { return M.transpose(); }
double dot(const cnoid::Vector3& a, const cnoid::Vector3& b) { return a.dot(b); }
typedef cnoid::Matrix3 Matrix33;
#endif


#ifndef NS_HRPMODEL
#define NS_HRPMODEL OpenHRP
typedef OpenHRP::vector3 Vector3;
typedef OpenHRP::matrix33 Matrix33;
#endif

using namespace std;
using namespace boost;
using namespace NS_HRPMODEL;

static BodyInterface* bodyInterface = 0;

static BodyCustomizerInterface bodyCustomizerInterface;

struct JointValSet
{
  double* valuePtr;
  double* velocityPtr;
  double* torqueForcePtr;
};

struct JAXONCustomizer
{
  BodyHandle bodyHandle;

  bool hasVirtualBushJoints;
  JointValSet jointValSets[2][3];
  double springT;
  double dampingT;
  double springR;
  double dampingR;

  double tilt_upper_bound;
  double tilt_lower_bound;
  double tilt_positive_speed;
  double tilt_negative_speed;

  //* *//
  bool hasMultisenseJoint;
  JointValSet multisense_joint;

  //* *//
  bool hasTiltLaserJoint;
  JointValSet tilt_laser_joint;
};


static const char** getTargetModelNames()
{
  char *rname = getenv("CHOREONOID_ROBOT");
  if (rname != NULL) {
    std::cerr << "CHOREONID_ROBOT =" << rname << std::endl;
  }
  static const char* names[] = {
    "JAXON_JVRC",
    "JAXON_BLUE",
    "CHIDORI",
    "TABLIS",
    rname,
    0 };

  return names;
}

static void getVirtualbushJoints(JAXONCustomizer* customizer, BodyHandle body)
{
  customizer->hasVirtualBushJoints = true;

  int bushIndices[2][3];

  bushIndices[0][0] = bodyInterface->getLinkIndexFromName(body, "RLEG_BUSH_Z");
  bushIndices[0][1] = bodyInterface->getLinkIndexFromName(body, "RLEG_BUSH_ROLL");
  bushIndices[0][2] = bodyInterface->getLinkIndexFromName(body, "RLEG_BUSH_PITCH");
  bushIndices[1][0] = bodyInterface->getLinkIndexFromName(body, "LLEG_BUSH_Z");
  bushIndices[1][1] = bodyInterface->getLinkIndexFromName(body, "LLEG_BUSH_ROLL");
  bushIndices[1][2] = bodyInterface->getLinkIndexFromName(body, "LLEG_BUSH_PITCH");

  for(int i=0; i < 2; ++i){
    for(int j=0; j < 3; ++j){
      int bushIndex = bushIndices[i][j];
      if(bushIndex < 0){
        std::cerr << "[Customizer] failed to find out : " << i << " " << j << std::endl;
        customizer->hasVirtualBushJoints = false;
      } else {
        JointValSet& jointValSet = customizer->jointValSets[i][j];
        jointValSet.valuePtr = bodyInterface->getJointValuePtr(body, bushIndex);
        jointValSet.velocityPtr = bodyInterface->getJointVelocityPtr(body, bushIndex);
        jointValSet.torqueForcePtr = bodyInterface->getJointForcePtr(body, bushIndex);
      }
    }
  }

  //* additional motor joints *//
  {
    int bindex = bodyInterface->getLinkIndexFromName(body, "motor_joint");
    customizer->hasMultisenseJoint = true;
    if(bindex < 0){
      customizer->hasMultisenseJoint = false;
    } else {
      JointValSet& jointValSet = customizer->multisense_joint;
      jointValSet.valuePtr = bodyInterface->getJointValuePtr(body, bindex);
      jointValSet.velocityPtr = bodyInterface->getJointVelocityPtr(body, bindex);
      jointValSet.torqueForcePtr = bodyInterface->getJointForcePtr(body, bindex);
    }
  }

  //* additional motor joints *//
  {
    int bindex = bodyInterface->getLinkIndexFromName(body, "tilt_laser_joint");
    customizer->hasTiltLaserJoint = true;
    if(bindex < 0){
      customizer->hasTiltLaserJoint = false;
    } else {
      JointValSet& jointValSet = customizer->tilt_laser_joint;
      jointValSet.valuePtr = bodyInterface->getJointValuePtr(body, bindex);
      jointValSet.velocityPtr = bodyInterface->getJointVelocityPtr(body, bindex);
      jointValSet.torqueForcePtr = bodyInterface->getJointForcePtr(body, bindex);
    }
  }
}

static BodyCustomizerHandle create(BodyHandle bodyHandle, const char* modelName)
{
  JAXONCustomizer* customizer = 0;

  std::cerr << "create customizer : " << std::string(modelName) << std::endl;
  customizer = new JAXONCustomizer;

  customizer->bodyHandle = bodyHandle;
  customizer->hasVirtualBushJoints = false;

  customizer->springT  = 1.1e6; // N/m
  customizer->dampingT = 1.1e3; // N/(m/s)
  customizer->springR  = 2.5e3; // Nm / rad
  customizer->dampingR = 2.5;   // Nm / (rad/s)

  customizer->tilt_upper_bound = 1.35; // rad
  customizer->tilt_lower_bound = -0.7; // rad
  customizer->tilt_positive_speed = 1.0; // rad/s
  customizer->tilt_negative_speed = -1.0; // rad/s

  char* config_file_path = getenv("CUSTOMIZER_CONF_FILE");
  if (config_file_path) {
    ifstream ifs(config_file_path);
    if (ifs.is_open()) {
      std::cerr << "[JAXONCustomizer] Config file is: " << config_file_path << std::endl;
      YAML::Node param = YAML::LoadFile(config_file_path);
      customizer->springT  = param["bush"]["springT"].as<double>();
      customizer->dampingT = param["bush"]["dampingT"].as<double>();
      customizer->springR  = param["bush"]["springR"].as<double>();
      customizer->dampingR = param["bush"]["dampingR"].as<double>();
      customizer->tilt_upper_bound    = param["tilt_laser"]["TILT_UPPER_BOUND"].as<double>();
      customizer->tilt_positive_speed = param["tilt_laser"]["TILT_POSITIVE_SPEED"].as<double>();
      customizer->tilt_lower_bound    = param["tilt_laser"]["TILT_LOWER_BOUND"].as<double>();
      customizer->tilt_negative_speed = param["tilt_laser"]["TILT_NEGATIVE_SPEED"].as<double>();
    } else {
      std::cerr << "[JAXONCustomizer] " << config_file_path << " is not found" << std::endl;
    }
  }

  getVirtualbushJoints(customizer, bodyHandle);

  return static_cast<BodyCustomizerHandle>(customizer);
}


static void destroy(BodyCustomizerHandle customizerHandle)
{
  JAXONCustomizer* customizer = static_cast<JAXONCustomizer*>(customizerHandle);
  if(customizer){
    delete customizer;
  }
}

static void setVirtualJointForces(BodyCustomizerHandle customizerHandle)
{
  JAXONCustomizer* customizer = static_cast<JAXONCustomizer*>(customizerHandle);

  if(customizer->hasVirtualBushJoints){
    for(int i=0; i < 2; ++i){
      JointValSet& trans = customizer->jointValSets[i][0];
      *(trans.torqueForcePtr) = - customizer->springT * (*trans.valuePtr) - customizer->dampingT * (*trans.velocityPtr);
      //std::cerr << i << " " << 0 << " " << *(trans.torqueForcePtr) << " = " << -customizer->springT << " x " << *trans.valuePtr << " + " <<  - customizer->dampingT << " x " << *trans.velocityPtr << std::endl;

      for(int j=1; j < 3; ++j){
        JointValSet& rot = customizer->jointValSets[i][j];
        *(rot.torqueForcePtr) = - customizer->springR * (*rot.valuePtr) - customizer->dampingR * (*rot.velocityPtr);
        //std::cerr << i << " " << j << " " << *(rot.torqueForcePtr) << " = " << -customizer->springR << " x " << *rot.valuePtr << " + " <<  - customizer->dampingR << " x " << *rot.velocityPtr << std::endl;
      }
    }
  }

  if(customizer->hasMultisenseJoint) {
    JointValSet& trans = customizer->multisense_joint;
    static double dq_old = 0.0;
    double dq = *(trans.velocityPtr);
    double ddq = (dq - dq_old)/0.001; // dt = 0.001
    double tq = -(dq - 1.0) * 100 - 0.2 * ddq;
    double tlimit = 200;
    *(trans.torqueForcePtr) = std::max(std::min(tq, tlimit), -tlimit);
    dq_old = dq;
  }

  if(customizer->hasTiltLaserJoint) {
    JointValSet& trans = customizer->tilt_laser_joint;
    static double dq_old = 0.0;
    static bool move_positive = true;
    double  q = *(trans.valuePtr);
    if (q > customizer->tilt_upper_bound) {
      move_positive = false;
    } else if (q < customizer->tilt_lower_bound) {
      move_positive = true;
    }

    double dq = *(trans.velocityPtr);
    double tq;
    double ddq = (dq - dq_old) / 0.001; // dt = 0.001
    if (move_positive) {
      tq = -(dq - customizer->tilt_positive_speed) * 100 - 0.2 * ddq;
    } else {
      tq = -(dq - customizer->tilt_negative_speed) * 100 - 0.2 * ddq;
    }
    double tlimit = 200;
    *(trans.torqueForcePtr) = std::max(std::min(tq, tlimit), -tlimit);
    dq_old = dq;
  }
}

extern "C" DLL_EXPORT
NS_HRPMODEL::BodyCustomizerInterface* getHrpBodyCustomizerInterface(NS_HRPMODEL::BodyInterface* bodyInterface_)
{
  bodyInterface = bodyInterface_;

  bodyCustomizerInterface.version = NS_HRPMODEL::BODY_CUSTOMIZER_INTERFACE_VERSION;
  bodyCustomizerInterface.getTargetModelNames = getTargetModelNames;
  bodyCustomizerInterface.create = create;
  bodyCustomizerInterface.destroy = destroy;
  bodyCustomizerInterface.initializeAnalyticIk = NULL;
  bodyCustomizerInterface.calcAnalyticIk = NULL;
  bodyCustomizerInterface.setVirtualJointForces = setVirtualJointForces;

  return &bodyCustomizerInterface;
}
