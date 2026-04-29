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

#include "autoware_claw_panel.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkRequest>
#include <QScrollBar>
#include <QTimer>
#include <QVBoxLayout>

namespace autoware_claw_rviz_plugins
{

AutowareClawPanel::AutowareClawPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // ── Connection settings ──
  auto * connection_group = new QGroupBox("NemoClaw Gateway");
  auto * connection_layout = new QHBoxLayout;

  url_input_ = new QLineEdit("http://localhost:18789");
  url_input_->setPlaceholderText("NemoClaw gateway URL");
  status_label_ = new QLabel;
  setConnectionStatus(false);

  connection_layout->addWidget(new QLabel("URL:"));
  connection_layout->addWidget(url_input_, 1);
  connection_layout->addWidget(status_label_);
  connection_group->setLayout(connection_layout);

  // ── Chat display ──
  chat_display_ = new QTextBrowser;
  chat_display_->setOpenExternalLinks(false);
  chat_display_->setReadOnly(true);
  chat_display_->setStyleSheet(
    "QTextBrowser { background-color: #1e1e1e; color: #d4d4d4; "
    "font-family: monospace; font-size: 10pt; padding: 8px; }");
  chat_display_->setHtml(
    "<p style='color: #888;'>Connected to NemoClaw — type a message to interact with "
    "the autonomous driving assistant.</p>");

  // ── Input area ──
  auto * input_layout = new QHBoxLayout;
  message_input_ = new QLineEdit;
  message_input_->setPlaceholderText("Send a message to OpenClaw...");
  send_button_ = new QPushButton("Send");
  send_button_->setDefault(true);
  send_button_->setStyleSheet(
    "QPushButton { background-color: #0078d4; color: white; padding: 6px 16px; "
    "border-radius: 4px; } QPushButton:hover { background-color: #106ebe; }");

  input_layout->addWidget(message_input_, 1);
  input_layout->addWidget(send_button_);

  // ── Main layout ──
  auto * main_layout = new QVBoxLayout;
  main_layout->addWidget(connection_group);
  main_layout->addWidget(chat_display_, 1);
  main_layout->addLayout(input_layout);
  setLayout(main_layout);

  // ── Network ──
  network_manager_ = new QNetworkAccessManager(this);
  connect(network_manager_, &QNetworkAccessManager::finished, this, &AutowareClawPanel::onNetworkReply);

  // ── Signals ──
  connect(send_button_, &QPushButton::clicked, this, &AutowareClawPanel::onSendClicked);
  connect(message_input_, &QLineEdit::returnPressed, this, &AutowareClawPanel::onSendClicked);
  connect(url_input_, &QLineEdit::editingFinished, this, &AutowareClawPanel::onUrlChanged);

  gateway_url_ = url_input_->text();

  // Health check timer
  auto * health_timer = new QTimer(this);
  connect(health_timer, &QTimer::timeout, this, &AutowareClawPanel::checkHealth);
  health_timer->start(5000);
}

void AutowareClawPanel::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
  checkHealth();
}

// ── Persistence ──

void AutowareClawPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("gateway_url", url_input_->text());
}

void AutowareClawPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString url;
  if (config.mapGetString("gateway_url", &url)) {
    url_input_->setText(url);
    gateway_url_ = url;
  }
}

// ── Slots ──

void AutowareClawPanel::onSendClicked()
{
  const auto text = message_input_->text().trimmed();
  if (text.isEmpty()) return;

  appendMessage("You", text);
  message_input_->clear();
  message_input_->setEnabled(false);
  send_button_->setEnabled(false);
  send_button_->setText("...");

  sendChatRequest(text);
}

void AutowareClawPanel::onUrlChanged()
{
  gateway_url_ = url_input_->text();
  checkHealth();
}

void AutowareClawPanel::onNetworkReply(QNetworkReply * reply)
{
  const auto url_path = reply->url().path();

  if (url_path.contains("/health") || url_path.contains("/api/tags")) {
    // Health check response
    setConnectionStatus(reply->error() == QNetworkReply::NoError);
    reply->deleteLater();
    return;
  }

  // Chat response
  message_input_->setEnabled(true);
  send_button_->setEnabled(true);
  send_button_->setText("Send");

  if (reply->error() != QNetworkReply::NoError) {
    appendMessage("System", "Error: " + reply->errorString());
    reply->deleteLater();
    return;
  }

  const auto data = reply->readAll();
  const auto doc = QJsonDocument::fromJson(data);

  if (doc.isObject()) {
    const auto obj = doc.object();
    // Try common response fields
    QString response;
    if (obj.contains("response")) {
      response = obj["response"].toString();
    } else if (obj.contains("message")) {
      const auto msg = obj["message"];
      if (msg.isString()) {
        response = msg.toString();
      } else if (msg.isObject()) {
        response = msg.toObject()["content"].toString();
      }
    } else if (obj.contains("content")) {
      response = obj["content"].toString();
    } else {
      response = QString::fromUtf8(data);
    }
    appendMessage("OpenClaw", response);
  } else {
    appendMessage("OpenClaw", QString::fromUtf8(data));
  }

  reply->deleteLater();
}

void AutowareClawPanel::checkHealth()
{
  QNetworkRequest request(QUrl(gateway_url_ + "/health"));
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
  network_manager_->get(request);
}

// ── Helpers ──

void AutowareClawPanel::appendMessage(const QString & sender, const QString & text)
{
  QString color = "#d4d4d4";
  if (sender == "You") {
    color = "#569cd6";
  } else if (sender == "OpenClaw") {
    color = "#4ec9b0";
  } else if (sender == "System") {
    color = "#ce9178";
  }

  // Escape HTML in text but preserve newlines
  QString escaped = text.toHtmlEscaped().replace("\n", "<br>");
  chat_display_->append(
    QString("<p><b style='color: %1;'>%2:</b> %3</p>").arg(color, sender, escaped));

  // Auto-scroll to bottom
  auto * scrollbar = chat_display_->verticalScrollBar();
  scrollbar->setValue(scrollbar->maximum());
}

void AutowareClawPanel::setConnectionStatus(bool connected)
{
  if (connected) {
    status_label_->setText(" Connected ");
    status_label_->setStyleSheet(
      "QLabel { background-color: #2d7d2d; color: white; padding: 2px 8px; "
      "border-radius: 3px; font-size: 9pt; }");
  } else {
    status_label_->setText(" Disconnected ");
    status_label_->setStyleSheet(
      "QLabel { background-color: #7d2d2d; color: white; padding: 2px 8px; "
      "border-radius: 3px; font-size: 9pt; }");
  }
}

void AutowareClawPanel::sendChatRequest(const QString & message)
{
  QJsonObject body;
  body["message"] = message;

  QNetworkRequest request(QUrl(gateway_url_ + "/api/v1/chat"));
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

  network_manager_->post(request, QJsonDocument(body).toJson(QJsonDocument::Compact));
}

}  // namespace autoware_claw_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware_claw_rviz_plugins::AutowareClawPanel, rviz_common::Panel)
