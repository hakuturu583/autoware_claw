// Copyright 2026 Masaya Kataoka
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_CLAW_RVIZ_PLUGINS__AUTOWARE_CLAW_PANEL_HPP_
#define AUTOWARE_CLAW_RVIZ_PLUGINS__AUTOWARE_CLAW_PANEL_HPP_

#include <QLabel>
#include <QLineEdit>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QPushButton>
#include <QTextBrowser>
#include <QWidget>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace autoware_claw_rviz_plugins
{

class AutowareClawPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AutowareClawPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

public Q_SLOTS:
  void onSendClicked();
  void onUrlChanged();
  void onNetworkReply(QNetworkReply * reply);
  void checkHealth();

private:
  void appendMessage(const QString & sender, const QString & text);
  void setConnectionStatus(bool connected);
  void sendChatRequest(const QString & message);

  // UI
  QLineEdit * url_input_;
  QLabel * status_label_;
  QTextBrowser * chat_display_;
  QLineEdit * message_input_;
  QPushButton * send_button_;

  // Network
  QNetworkAccessManager * network_manager_;
  QString gateway_url_;

  // ROS
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};

}  // namespace autoware_claw_rviz_plugins

#endif  // AUTOWARE_CLAW_RVIZ_PLUGINS__AUTOWARE_CLAW_PANEL_HPP_
