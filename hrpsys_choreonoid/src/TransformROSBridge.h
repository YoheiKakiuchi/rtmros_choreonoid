// -*- C++ -*-
/*!
 * @file  TransformROSBridge.h * @brief openhrp image - ros bridge * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef TRANSFORMROSBRIDGE_H
#define TRANSFORMROSBRIDGE_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/InterfaceDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>
//

// ros
#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace RTC;

class TransformROSBridge  : public RTC::DataFlowComponentBase
{
 public:
  TransformROSBridge(RTC::Manager* manager);
  ~TransformROSBridge();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry() 
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  // virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">

  TimedDoubleSeq m_Tform;
  InPort<TimedDoubleSeq> m_TformIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">

  // </rtc-template>

 private:
  ros::NodeHandle nh;
  ros::Publisher odom_pub;

  bool publish_odom_;
  tf::Transform init_trans_;
  tf::Transform prev_trans_;
  ros::Time prev_stamp_;

  double pub_cycle_;
  bool is_initialized_;
  bool initial_relative_;

  tf::TransformBroadcaster br;
  bool publish_tf_;
  std::string tf_parent_frame_;
  std::string tf_frame_;

  void convertTformToTfTransform(tf::Transform& result_trans);
  void calculateTwist(const tf::Transform& _current_trans, const tf::Transform& _prev_trans, tf::Vector3& _linear_twist, tf::Vector3& _angular_twist, double _dt);
  
};


extern "C"
{
  DLL_EXPORT void TransformROSBridgeInit(RTC::Manager* manager);
};

#endif // RANGESENSORROSBRIDGE_H
