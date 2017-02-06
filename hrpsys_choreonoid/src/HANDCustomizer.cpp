#include <cmath>
#include <cstring>
#include <boost/function.hpp>
#include <boost/bind.hpp>

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

struct HANDCustomizer
{
  BodyHandle bodyHandle;

  bool hasVirtualBushJoints;
  JointValSet jointValSets[2][3];
  double springT0;
  double dampingT0;
  double springT1;
  double dampingT1;
  double springT2;
  double dampingT2;
};


static const char** getTargetModelNames()
{
  static const char* names[] = {
    "JAXON_JVRC",
    "RHP2",
    0 };
  return names;
}


static void getVirtualbushJoints(HANDCustomizer* customizer, BodyHandle body)
{
  customizer->hasVirtualBushJoints = true;

  int bushIndices[2][3];

  bushIndices[0][0] = bodyInterface->getLinkIndexFromName(body, "RARM_BUSH_Z");
  bushIndices[0][1] = bodyInterface->getLinkIndexFromName(body, "RARM_BUSH_Y");
  bushIndices[0][2] = bodyInterface->getLinkIndexFromName(body, "RARM_BUSH_X");
  bushIndices[1][0] = bodyInterface->getLinkIndexFromName(body, "LARM_BUSH_Z");
  bushIndices[1][1] = bodyInterface->getLinkIndexFromName(body, "LARM_BUSH_Y");
  bushIndices[1][2] = bodyInterface->getLinkIndexFromName(body, "LARM_BUSH_X");

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
}

static BodyCustomizerHandle create(BodyHandle bodyHandle, const char* modelName)
{
  HANDCustomizer* customizer = 0;

  std::cerr << "create customizer : " << std::string(modelName) << std::endl;
  customizer = new HANDCustomizer;

  customizer->bodyHandle = bodyHandle;
  customizer->hasVirtualBushJoints = false;
#if 0
  customizer->springT0  = 2.0e4; // N/m
  customizer->dampingT0 = 1.3e2; // N/(m/s)
  customizer->springT1  = 2.0e4; // N/m
  customizer->dampingT1 = 1.3e2; // N/(m/s)
  customizer->springT2  = 2.0e4; // N/m
  customizer->dampingT2 = 1.3e2; // N/(m/s)
#endif

#if 1
  customizer->springT0  = 2.0e5; // N/m
  customizer->dampingT0 = 4.0e2; // N/(m/s)
  customizer->springT1  = 2.0e5; // N/m
  customizer->dampingT1 = 4.0e2; // N/(m/s)
  customizer->springT2  = 2.0e5; // N/m
  customizer->dampingT2 = 4.0e2; // N/(m/s)
#endif

  getVirtualbushJoints(customizer, bodyHandle);

  return static_cast<BodyCustomizerHandle>(customizer);
}


static void destroy(BodyCustomizerHandle customizerHandle)
{
  HANDCustomizer* customizer = static_cast<HANDCustomizer*>(customizerHandle);
  if(customizer){
    delete customizer;
  }
}

static void setVirtualJointForces(BodyCustomizerHandle customizerHandle)
{
  HANDCustomizer* customizer = static_cast<HANDCustomizer*>(customizerHandle);

  if(customizer->hasVirtualBushJoints){
    for(int i=0; i < 2; ++i){
      JointValSet& trans0 = customizer->jointValSets[i][0];
      *(trans0.torqueForcePtr) = - customizer->springT0 * (*trans0.valuePtr) - customizer->dampingT0 * (*trans0.velocityPtr);

      JointValSet& trans1 = customizer->jointValSets[i][1];
      *(trans1.torqueForcePtr) = - customizer->springT1 * (*trans1.valuePtr) - customizer->dampingT1 * (*trans1.velocityPtr);

      JointValSet& trans2 = customizer->jointValSets[i][2];
      *(trans2.torqueForcePtr) = - customizer->springT2 * (*trans2.valuePtr) - customizer->dampingT2 * (*trans2.velocityPtr);
    }
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
