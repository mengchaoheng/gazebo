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
#include <algorithm>

#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/Actions.hh>
#include "DFGUIPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(DFGUIPlugin)

/////////////////////////////////////////////////
DFGUIPlugin::DFGUIPlugin()
  : GUIPlugin()
{
  // This is needed to avoid the creation of a black widget with default size.
  this->move(-1, -1);
  this->resize(1, 1);

  // Set the increment or decrement in angle per key pressed.
  this->angleStep.Degree(1.0);

  // Initialize transport.
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->controlPub =
  this->gzNode->Advertise<ductedfan_msgs::ductedfan>("~/ductedfan/control");
  this->stateSub = this->gzNode->Subscribe<ductedfan_msgs::ductedfan>(
    "~/ductedfan/state", &DFGUIPlugin::OnState, this);

  // Connect hotkeys.
  
  QShortcut *increaseThrust = new QShortcut(QKeySequence("w"), this);
  QObject::connect(increaseThrust, SIGNAL(activated()), this,
      SLOT(OnIncreaseThrust()));

  QShortcut *decreaseThrust = new QShortcut(QKeySequence("s"), this);
  QObject::connect(decreaseThrust, SIGNAL(activated()), this,
      SLOT(OnDecreaseThrust()));


  QShortcut *increaseRoll = new QShortcut(QKeySequence(Qt::Key_Left), this);
  QObject::connect(increaseRoll, SIGNAL(activated()), this,
      SLOT(OnIncreaseRoll()));

  QShortcut *decreaseRoll = new QShortcut(QKeySequence(Qt::Key_Right), this);
  QObject::connect(decreaseRoll, SIGNAL(activated()), this,
      SLOT(OnDecreaseRoll()));

  QShortcut *increaseElevators =
    new QShortcut(QKeySequence(Qt::Key_Down), this);
  QObject::connect(increaseElevators, SIGNAL(activated()), this,
      SLOT(OnIncreasePitch()));

  QShortcut *decreaseElevators = new QShortcut(QKeySequence(Qt::Key_Up), this);
  QObject::connect(decreaseElevators, SIGNAL(activated()), this,
      SLOT(OnDecreasePitch()));

  QShortcut *increaseRudder = new QShortcut(QKeySequence("d"), this);
  QObject::connect(increaseRudder, SIGNAL(activated()), this,
      SLOT(OnIncreaseYaw()));

  QShortcut *decreaseRudder = new QShortcut(QKeySequence("a"), this);
  QObject::connect(decreaseRudder, SIGNAL(activated()), this,
      SLOT(OnDecreaseYaw()));

  QShortcut *presetTakeOff = new QShortcut(QKeySequence('1'), this);
  QObject::connect(presetTakeOff, SIGNAL(activated()), this,
      SLOT(OnPresetTakeOff()));

  QShortcut *presetCruise = new QShortcut(QKeySequence('2'), this);
  QObject::connect(presetCruise, SIGNAL(activated()), this,
      SLOT(OnPresetCruise()));
  
  QShortcut *presetLanding = new QShortcut(QKeySequence('3'), this);
  QObject::connect(presetLanding, SIGNAL(activated()), this,
      SLOT(OnPresetLanding()));
  
  gzdbg << "Ducted Fan GUI" << std::endl;
}

/////////////////////////////////////////////////
DFGUIPlugin::~DFGUIPlugin()
{
}

/////////////////////////////////////////////////
void DFGUIPlugin::OnState(ConstductedfanPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Refresh the state.
  this->state = *_msg;
}


/////////////////////////////////////////////////
void DFGUIPlugin::OnIncreaseThrust()
{
  float thrust1;
  float thrust2;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    thrust1 = this->state.cmd_r1_speed();
    thrust2 = this->state.cmd_r2_speed();
    gzdbg << "DFGUIplugin thrust1: "<< thrust1<<std::endl;
  }

  ductedfan_msgs::ductedfan msg;
  thrust1 = std::min(thrust1 + 0.1f, 1.0f);
  thrust2 = std::min(thrust2 + 0.1f, 1.0f);
  msg.set_cmd_r1_speed(thrust1);
  msg.set_cmd_r2_speed(thrust2);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void DFGUIPlugin::OnDecreaseThrust()
{
  float thrust1;
  float thrust2;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    thrust1 = this->state.cmd_r1_speed();
    thrust2 = this->state.cmd_r2_speed();
    gzdbg << "DFGUIplugin thrust1: "<< thrust1<<std::endl;
  }

  ductedfan_msgs::ductedfan msg;
  thrust1 = std::max(thrust1 - 0.1f, 0.0f);
  thrust2 = std::max(thrust2 - 0.1f, 0.0f);
  msg.set_cmd_r1_speed(thrust1);
  msg.set_cmd_r2_speed(thrust2);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void DFGUIPlugin::OnIncreaseRoll()
{
  ignition::math::Angle roll;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    roll.Radian(this->state.cmd_csroll());
  }

  ductedfan_msgs::ductedfan msg;
  if (roll.Degree() < 20)
  {
    roll += this->angleStep;
    msg.set_cmd_csroll(roll.Radian());
    this->controlPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void DFGUIPlugin::OnDecreaseRoll()
{
  ignition::math::Angle roll;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    roll.Radian(this->state.cmd_csroll());
  }

  ductedfan_msgs::ductedfan msg;
  if (roll.Degree() > -20)
  {
    roll -= this->angleStep;
    msg.set_cmd_csroll(roll.Radian());
    this->controlPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void DFGUIPlugin::OnIncreasePitch()
{
  ignition::math::Angle pitch;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    pitch.Radian(this->state.cmd_cspitch());
  }

  ductedfan_msgs::ductedfan msg;
  if (pitch.Degree() < 20)
  {
    pitch += this->angleStep;
    msg.set_cmd_cspitch(pitch.Radian());
    this->controlPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void DFGUIPlugin::OnDecreasePitch()
{
  ignition::math::Angle pitch;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    pitch.Radian(this->state.cmd_cspitch());
  }

  ductedfan_msgs::ductedfan msg;
  if (pitch.Degree() > -20)
  {
    pitch -= this->angleStep;
    msg.set_cmd_cspitch(pitch.Radian());
    this->controlPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void DFGUIPlugin::OnIncreaseYaw()
{
  float thrust1;
  float thrust2;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    thrust1 = this->state.cmd_r1_speed();
    thrust2 = this->state.cmd_r2_speed();
  }
  ductedfan_msgs::ductedfan msg;
  thrust1 = std::max(thrust1 - 0.1f, 0.0f);
  thrust2 = std::max(thrust2 + 0.1f, 0.0f);
  msg.set_cmd_r1_speed(thrust1);
  msg.set_cmd_r2_speed(thrust2);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void DFGUIPlugin::OnDecreaseYaw()
{
  float thrust1;
  float thrust2;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    thrust1 = this->state.cmd_r1_speed();
    thrust2 = this->state.cmd_r2_speed();
  }
  ductedfan_msgs::ductedfan msg;
  thrust1 = std::max(thrust1 + 0.1f, 0.0f);
  thrust2 = std::max(thrust2 - 0.1f, 0.0f);
  msg.set_cmd_r1_speed(thrust1);
  msg.set_cmd_r2_speed(thrust2);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void DFGUIPlugin::OnPresetTakeOff()
{
  ductedfan_msgs::ductedfan msg;
  msg.set_cmd_r1_speed(0.8);
  msg.set_cmd_r2_speed(0.8);
  msg.set_cmd_csroll(0);
  msg.set_cmd_cspitch(0);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void DFGUIPlugin::OnPresetCruise()
{
  ductedfan_msgs::ductedfan msg;
  msg.set_cmd_r1_speed(0.6);
  msg.set_cmd_r2_speed(0.6);
  msg.set_cmd_csroll(0);
  msg.set_cmd_cspitch(0);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void DFGUIPlugin::OnPresetLanding()
{
  
  ductedfan_msgs::ductedfan msg;
  msg.set_cmd_r1_speed(0.3);
  msg.set_cmd_r2_speed(0.3);
  msg.set_cmd_csroll(0);
  msg.set_cmd_cspitch(0);
  this->controlPub->Publish(msg);
  
  gzdbg << "Ducted Fan GUI call" << std::endl;
}
