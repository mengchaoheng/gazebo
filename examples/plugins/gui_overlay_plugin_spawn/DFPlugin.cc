/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>
#include <string>
#include <sdf/sdf.hh>
//#include <ignition/common/Profiler.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "DFPlugin.hh"

using namespace gazebo;


GZ_REGISTER_MODEL_PLUGIN(DFPlugin)

////////////////////////////////////////////////////////////////////////////////
DFPlugin::DFPlugin()
{
  this->cmds.fill(0.0f);

  // PID default parameters.
  this->propellerPID.Init(50.0, 0.1, 1, 0.0, 0.0, 20000.0, -20000.0);
  this->propellerPID.SetCmd(0.0);

  for (auto &pid : this->controlSurfacesPID)
  {
    pid.Init(50.0, 0.1, 1, 0.0, 0.0, 20.0, -20.0);
    pid.SetCmd(0.0);
  }
}

/////////////////////////////////////////////////
DFPlugin::~DFPlugin()
{
  this->updateConnection.reset();
}

/////////////////////////////////////////////////
bool DFPlugin::FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf,
    physics::JointPtr &_joint)
{
  // Read the required plugin parameters.
  if (!_sdf->HasElement(_sdfParam))
  {
    gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
    return false;
  }

  std::string jointName = _sdf->Get<std::string>(_sdfParam);
  _joint = this->model->GetJoint(jointName);
  if (!_joint)
  {
    gzerr << "Failed to find joint [" << jointName
          << "] aborting plugin load." << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
void DFPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "DFPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "DFPlugin _sdf pointer is NULL");
  this->model = _model;

  // Read the required parameter for the propeller max RPMs.
  if (!_sdf->HasElement("propeller_max_rpm"))
  {
    gzerr << "Unable to find the <propeller_max_rpm> parameter." << std::endl;
    return;
  }
  this->propellerMaxRpm = _sdf->Get<int32_t>("propeller_max_rpm");
  if (this->propellerMaxRpm == 0)
  {
    gzerr << "Maximum propeller RPMs cannot be 0" << std::endl;
    return;
  }

  // Read the required joint name parameters.
  std::vector<std::string> requiredParams = {"csroll", "cspitch", "r1", "r2"};

  for (size_t i = 0; i < requiredParams.size(); ++i)
  {
    if (!this->FindJoint(requiredParams[i], _sdf, this->joints[i]))
      return;
  }

  // Overload the PID parameters if they are available.
  if (_sdf->HasElement("propeller_p_gain"))
    this->propellerPID.SetPGain(_sdf->Get<double>("propeller_p_gain"));

  if (_sdf->HasElement("propeller_i_gain"))
    this->propellerPID.SetIGain(_sdf->Get<double>("propeller_i_gain"));

  if (_sdf->HasElement("propeller_d_gain"))
    this->propellerPID.SetDGain(_sdf->Get<double>("propeller_d_gain"));

  if (_sdf->HasElement("surfaces_p_gain"))
  {
    for (auto &pid : this->controlSurfacesPID)
      pid.SetPGain(_sdf->Get<double>("surfaces_p_gain"));
  }

  if (_sdf->HasElement("surfaces_i_gain"))
  {
    for (auto &pid : this->controlSurfacesPID)
      pid.SetIGain(_sdf->Get<double>("surfaces_i_gain"));
  }

  if (_sdf->HasElement("surfaces_d_gain"))
  {
    for (auto &pid : this->controlSurfacesPID)
      pid.SetDGain(_sdf->Get<double>("surfaces_d_gain"));
  }

  // Controller time control.
  this->lastControllerUpdateTime = this->model->GetWorld()->SimTime();

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&DFPlugin::Update, this, std::placeholders::_1));

  // Initialize transport.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  std::string prefix = "~/" + this->model->GetName() + "/";
  this->statePub = this->node->Advertise<ductedfan_msgs::ductedfan>(prefix + "state");
  this->controlSub = this->node->Subscribe(prefix + "control",
    &DFPlugin::OnControl, this);

  //gzlog << "gzlog: ducted_fan ready to fly. The force will be with you" << std::endl;
  //printf("printf: ducted_fan ready to fly. The force will be with you\n");
  //gzerr << "gzerr: ducted_fan ready to fly. The force will be with you" << std::endl;
  //gzwarn << "gzwarn: ducted_fan ready to fly. The force will be with you" << std::endl;
  gzdbg << "DFplugin" << std::endl;
}

/////////////////////////////////////////////////
void DFPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  //gzdbg << "gzdbg: Update" << std::endl;
  //IGN_PROFILE("DFPlugin::OnUpdate");
  std::lock_guard<std::mutex> lock(this->mutex);

  gazebo::common::Time curTime = this->model->GetWorld()->SimTime();

  if (curTime > this->lastControllerUpdateTime)
  {
    // Update the control surfaces and publish the new state.
    //IGN_PROFILE_BEGIN("Update");
    this->UpdatePIDs((curTime - this->lastControllerUpdateTime).Double());
    //IGN_PROFILE_END();
    //IGN_PROFILE_BEGIN("Publish");
    this->PublishState();
    //IGN_PROFILE_END();
    this->lastControllerUpdateTime = curTime;
  }
}
//reseive control
/////////////////////////////////////////////////

void DFPlugin::OnControl(ConstductedfanPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (_msg->has_cmd_r1_speed() &&
      std::abs(_msg->cmd_r1_speed()) <= 1)
  {
    this->cmds[kPropeller1] = _msg->cmd_r1_speed();
    gzdbg << "DFplugin cmd_r1_speed: "<< _msg->cmd_r1_speed()<<std::endl;
  }
  if (_msg->has_cmd_r2_speed() &&
      std::abs(_msg->cmd_r2_speed()) <= 1)
  {
    this->cmds[kPropeller2] = _msg->cmd_r2_speed();
  }
  if (_msg->has_cmd_csroll())
    this->cmds[kCsroll] = _msg->cmd_csroll();
  if (_msg->has_cmd_cspitch())
    this->cmds[kCspitch] = _msg->cmd_cspitch();
}

//change control sureface
/////////////////////////////////////////////////
void DFPlugin::UpdatePIDs(double _dt)
{
  // Velocity PID for the r1.
  double vel = this->joints[kPropeller1]->GetVelocity(0);
  //gzdbg << "DFplugin vel1: "<< vel<<std::endl;
  double maxVel = this->propellerMaxRpm*2.0*M_PI/60.0;
  double target = maxVel * this->cmds[kPropeller1];
  double error = vel - target;
  double force = this->propellerPID.Update(error, _dt);
  this->joints[kPropeller1]->SetForce(0, force);

   // Velocity PID for the r2.
  vel = this->joints[kPropeller2]->GetVelocity(0);
  //gzdbg << "DFplugin vel2: "<< vel<<std::endl;
  target = maxVel * this->cmds[kPropeller2];
  error = vel - target;
  force = this->propellerPID.Update(error, _dt);
  this->joints[kPropeller2]->SetForce(0, force);
  
  // Position PID for the control surfaces.
  for (size_t i = 0; i < this->controlSurfacesPID.size(); ++i)
  {
    double pos = this->joints[i]->Position(0);
    error = pos - this->cmds[i];
    force = this->controlSurfacesPID[i].Update(error, _dt);
    this->joints[i]->SetForce(0, force);
  }
}

/////////////////////////////////////////////////
void DFPlugin::PublishState()
{
  // Read the current state.
  double propellerRpms1 = this->joints[kPropeller1]->GetVelocity(0)
    /(2.0*M_PI)*60.0;//rpm
  double propellerRpms2 = this->joints[kPropeller2]->GetVelocity(0)
  /(2.0*M_PI)*60.0;//rpm
  float propellerSpeed1 = propellerRpms1 / this->propellerMaxRpm;//Normalize
  float propellerSpeed2 = propellerRpms2 / this->propellerMaxRpm;//Normalize
  float csroll = this->joints[kCsroll]->Position(0);
  float cspitch = this->joints[kCspitch]->Position(0);

  ductedfan_msgs::ductedfan msg;
  //msg.set_r1_speed(500);
  //gzdbg << "DFplugin msg: "<< msg.r1_speed()<<std::endl;
  
  // Set the observed state.
  msg.set_r1_speed(propellerSpeed1);
  msg.set_r2_speed(propellerSpeed2);
  msg.set_csroll(csroll);
  msg.set_cspitch(cspitch);
  

  // Set the target state.
  msg.set_cmd_r1_speed(this->cmds[kPropeller1]);
  msg.set_cmd_r2_speed(this->cmds[kPropeller2]);
  msg.set_cmd_csroll(this->cmds[kCsroll]);
  msg.set_cmd_cspitch(this->cmds[kCspitch]);
 
  //Publish msg
  this->statePub->Publish(msg);
 
}
