// Copyright 2026 — iros_llm_swarm contributors. Apache-2.0.

#include "iros_llm_rviz_panel/llm_panel.hpp"

#include <chrono>
#include <cmath>
#include <string>

#include <QColor>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTextCursor>
#include <QTextEdit>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

using namespace std::chrono_literals;

namespace iros_llm_rviz_panel
{

namespace
{

// ---- Style helpers --------------------------------------------------------

QString badgeStyle(const QString & bg, const QString & fg = "white")
{
  return QString(
    "QLabel { background-color: %1; color: %2; "
    "padding: 4px 10px; border-radius: 4px; font-weight: bold; }").arg(bg, fg);
}

QString modeColor(const QString & mode)
{
  if (mode == "mapf")      return "#1976D2";  // blue
  if (mode == "formation") return "#7B1FA2";  // purple
  return "#616161";                           // grey for idle / unknown
}

QString statusColor(const QString & status)
{
  if (status == "OK")    return "#388E3C";  // green
  if (status == "WARN")  return "#F57C00";  // orange
  if (status == "ERROR") return "#C62828";  // red
  return "#616161";
}

// Deterministic per-robot color. 20 robots → 20 hues spread around the wheel.
std_msgs::msg::ColorRGBA robotColor(uint32_t id)
{
  std_msgs::msg::ColorRGBA c;
  const float hue = std::fmod(static_cast<float>(id) * 360.0f / 20.0f, 360.0f);
  QColor q = QColor::fromHsvF(hue / 360.0f, 0.85f, 0.95f);
  c.r = static_cast<float>(q.redF());
  c.g = static_cast<float>(q.greenF());
  c.b = static_cast<float>(q.blueF());
  c.a = 0.85f;
  return c;
}

}  // namespace


// ===========================================================================
// Construction
// ===========================================================================

LlmPanel::LlmPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  buildUi();
}

LlmPanel::~LlmPanel() = default;

void LlmPanel::buildUi()
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(6, 6, 6, 6);
  root->setSpacing(6);

  // ---- Top status bar -----------------------------------------------------
  auto * status_row = new QHBoxLayout();
  status_row->setSpacing(6);

  mode_badge_     = new QLabel("idle");
  action_badge_   = new QLabel("none");
  status_badge_   = new QLabel("OK");
  thinking_label_ = new QLabel("");
  error_label_    = new QLabel("");

  mode_badge_->setStyleSheet(badgeStyle(modeColor("idle")));
  action_badge_->setStyleSheet(badgeStyle("#455A64"));
  status_badge_->setStyleSheet(badgeStyle(statusColor("OK")));

  thinking_label_->setMinimumWidth(20);
  thinking_label_->setAlignment(Qt::AlignCenter);

  error_label_->setStyleSheet("QLabel { color: #C62828; }");
  error_label_->setWordWrap(false);

  stop_button_ = new QPushButton("STOP ALL");
  stop_button_->setStyleSheet(
    "QPushButton { background-color: #C62828; color: white; "
    "font-weight: bold; padding: 6px 14px; border-radius: 4px; }"
    "QPushButton:pressed { background-color: #8E0000; }");

  status_row->addWidget(mode_badge_);
  status_row->addWidget(action_badge_);
  status_row->addWidget(status_badge_);
  status_row->addWidget(thinking_label_);
  status_row->addWidget(error_label_, /*stretch=*/1);
  status_row->addWidget(stop_button_);

  root->addLayout(status_row);

  // ---- Tab widget ---------------------------------------------------------
  tabs_ = new QTabWidget(this);

  // -- Chat tab --
  auto * chat_tab    = new QWidget();
  auto * chat_layout = new QVBoxLayout(chat_tab);
  chat_layout->setContentsMargins(0, 0, 0, 0);

  chat_view_ = new QTextEdit();
  chat_view_->setReadOnly(true);
  chat_view_->setStyleSheet("QTextEdit { font-family: monospace; }");

  auto * input_row = new QHBoxLayout();
  chat_input_ = new QLineEdit();
  chat_input_->setPlaceholderText(
    "Type a command... (e.g. \"cyan to hub\", \"stop\")");
  send_button_ = new QPushButton("Send");
  input_row->addWidget(chat_input_);
  input_row->addWidget(send_button_);

  chat_layout->addWidget(chat_view_);
  chat_layout->addLayout(input_row);

  tabs_->addTab(chat_tab, "Chat");

  // Placeholder tabs for v2 — keep the layout stable so users know more is
  // coming and so we don't have to re-do tab indices later.
  auto * mapf_tab = new QWidget();
  auto * mapf_layout = new QVBoxLayout(mapf_tab);
  mapf_layout->addWidget(new QLabel("MAPF Live — coming in v2."));
  mapf_layout->addStretch();
  tabs_->addTab(mapf_tab, "MAPF");

  auto * events_tab = new QWidget();
  auto * events_layout = new QVBoxLayout(events_tab);
  events_layout->addWidget(new QLabel("LLM Events — coming in v2."));
  events_layout->addStretch();
  tabs_->addTab(events_tab, "Events");

  root->addWidget(tabs_, /*stretch=*/1);

  // ---- Wiring -------------------------------------------------------------
  connect(send_button_, &QPushButton::clicked,
          this, &LlmPanel::onSendChat);
  connect(chat_input_,  &QLineEdit::returnPressed,
          this, &LlmPanel::onSendChat);
  connect(stop_button_, &QPushButton::clicked,
          this, &LlmPanel::onStopAll);

  // Cross-thread signals — every ROS-thread → GUI-thread bridge goes here.
  connect(this, &LlmPanel::btStateReceived,
          this, &LlmPanel::onUpdateStatus, Qt::QueuedConnection);
  connect(this, &LlmPanel::chatChunkReceived,
          this, &LlmPanel::onChatChunk,    Qt::QueuedConnection);
  connect(this, &LlmPanel::chatStageReceived,
          this, &LlmPanel::onChatStage,    Qt::QueuedConnection);
  connect(this, &LlmPanel::chatFinished,
          this, &LlmPanel::onChatFinished, Qt::QueuedConnection);
}


// ===========================================================================
// ROS setup (called after RViz finished initializing the display context)
// ===========================================================================

void LlmPanel::onInitialize()
{
  setupRos();
}

void LlmPanel::setupRos()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    // RViz couldn't give us a node — we can't do anything. The panel will
    // still render, just inert; surface the problem in the chat view.
    chat_view_->append(
      "<span style='color:#C62828'>"
      "[fatal] could not acquire RViz ROS node"
      "</span>");
    return;
  }
  node_ = abstraction->get_raw_node();

  // BTState publisher uses BEST_EFFORT (see BTStatePublisher::BTStatePublisher
  // in iros_llm_swarm_bt). Our subscription QoS must match.
  rclcpp::QoS qos(10);
  qos.best_effort();
  bt_state_sub_ = node_->create_subscription<BTState>(
    "/bt/state", qos,
    [this](BTState::ConstSharedPtr msg)
    {
      Q_EMIT btStateReceived(
        QString::fromStdString(msg->mode),
        QString::fromStdString(msg->action_status),
        QString::fromStdString(msg->active_action),
        QString::fromStdString(msg->last_error),
        msg->llm_thinking);

      publishMarkers(*msg);
    });

  chat_client_ = rclcpp_action::create_client<LlmChat>(node_, "/llm/chat");
  cmd_client_  = rclcpp_action::create_client<LlmCommand>(node_, "/llm/command");

  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/llm_panel/markers", rclcpp::QoS(10));

  RCLCPP_INFO(node_->get_logger(),
    "LlmPanel ready. Subscribing /bt/state, "
    "publishing /llm_panel/markers, "
    "talking to /llm/chat and /llm/command.");
}


// ===========================================================================
// Status bar (GUI thread)
// ===========================================================================

void LlmPanel::onUpdateStatus(
  QString mode,
  QString action_status,
  QString active_action,
  QString last_error,
  bool    llm_thinking)
{
  mode_badge_->setText(mode);
  mode_badge_->setStyleSheet(badgeStyle(modeColor(mode)));

  action_badge_->setText(active_action.isEmpty() ? "none" : active_action);

  status_badge_->setText(action_status);
  status_badge_->setStyleSheet(badgeStyle(statusColor(action_status)));

  thinking_label_->setText(llm_thinking ? QString::fromUtf8("🧠") : QString());
  error_label_->setText(last_error);
}


// ===========================================================================
// Chat (GUI thread = sending + display, ROS thread = action callbacks)
// ===========================================================================

void LlmPanel::onSendChat()
{
  if (chat_in_flight_) {
    chat_view_->append(
      "<i style='color:#999'>[busy — waiting for previous reply]</i>");
    return;
  }
  const QString text = chat_input_->text().trimmed();
  if (text.isEmpty()) {
    return;
  }
  if (!chat_client_) {
    chat_view_->append(
      "<span style='color:#C62828'>[panel not ready]</span>");
    return;
  }
  if (!chat_client_->action_server_is_ready()) {
    chat_view_->append(
      "<span style='color:#C62828'>"
      "[/llm/chat not available — is iros_llm_orchestrator running?]"
      "</span>");
    return;
  }

  chat_view_->append(QString("<b>You:</b> %1").arg(text.toHtmlEscaped()));
  chat_input_->clear();
  chat_view_->append(QString::fromUtf8("<b>🤖:</b> "));
  chat_in_flight_ = true;

  LlmChat::Goal goal;
  goal.user_message             = text.toStdString();
  goal.execute_after_planning   = true;  // MVP: panel is fire-and-forget

  rclcpp_action::Client<LlmChat>::SendGoalOptions opts;

  opts.feedback_callback =
    [this](ChatGoalH::SharedPtr,
           const std::shared_ptr<const LlmChat::Feedback> fb)
    {
      Q_EMIT chatStageReceived(QString::fromStdString(fb->stage));
      if (!fb->chunk.empty()) {
        Q_EMIT chatChunkReceived(QString::fromStdString(fb->chunk));
      }
    };

  opts.result_callback =
    [this](const ChatGoalH::WrappedResult & wrapped)
    {
      const bool transport_ok =
        (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED);
      const bool ok = transport_ok && wrapped.result && wrapped.result->success;
      const QString info = wrapped.result
        ? QString::fromStdString(wrapped.result->info)
        : QString("transport error");
      Q_EMIT chatFinished(ok, info);
    };

  chat_client_->async_send_goal(goal, opts);
}

void LlmPanel::onChatChunk(QString chunk)
{
  // Insert plain text at the end so streaming feels live. We use a cursor
  // (not append) because append would create a new paragraph for every chunk.
  auto cursor = chat_view_->textCursor();
  cursor.movePosition(QTextCursor::End);
  cursor.insertText(chunk);
  chat_view_->setTextCursor(cursor);
  chat_view_->ensureCursorVisible();
}

void LlmPanel::onChatStage(QString stage)
{
  if (stage == "executing") {
    chat_view_->append(
      "<i style='color:#1976D2'>[executing plan...]</i>");
  } else if (stage == "error") {
    // Detailed error info comes via the result; just mark the boundary.
    chat_view_->append("<i style='color:#C62828'>[error]</i>");
  }
}

void LlmPanel::onChatFinished(bool success, QString info)
{
  chat_in_flight_ = false;
  if (success) {
    chat_view_->append(QString::fromUtf8("<span style='color:#388E3C'>✓ done</span>"));
  } else {
    chat_view_->append(
      QString("<span style='color:#C62828'>[%1]</span>")
        .arg(info.isEmpty() ? "failed" : info.toHtmlEscaped()));
  }
}


// ===========================================================================
// STOP ALL — bypass LLM, send LlmCommand{idle} directly
// ===========================================================================

void LlmPanel::onStopAll()
{
  if (!cmd_client_ || !cmd_client_->action_server_is_ready()) {
    chat_view_->append(
      "<span style='color:#C62828'>"
      "[STOP failed: /llm/command not available]"
      "</span>");
    return;
  }
  LlmCommand::Goal goal;
  goal.mode   = "idle";
  goal.reason = "operator stop (RViz panel)";
  cmd_client_->async_send_goal(goal);
  chat_view_->append(
    "<span style='color:#C62828'><b>STOP ALL sent.</b></span>");
}


// ===========================================================================
// Markers — visualize the current MAPF goals in the 3D scene
// ===========================================================================

void LlmPanel::publishMarkers(const BTState & msg)
{
  // Strategy: publish only when there is something to draw, with a finite
  // lifetime so markers auto-expire when we go silent (e.g. mode != mapf).
  // BTState arrives at ~10 Hz; lifetime 500 ms gives a comfortable overlap.
  if (msg.mode != "mapf" || msg.robot_ids.empty() ||
      msg.robot_ids.size() != msg.goals.size())
  {
    return;
  }

  visualization_msgs::msg::MarkerArray ma;
  ma.markers.reserve(msg.robot_ids.size() * 2);

  const auto stamp    = node_->now();
  const auto lifetime = rclcpp::Duration::from_seconds(0.5);

  for (size_t i = 0; i < msg.robot_ids.size(); ++i) {
    const uint32_t id = msg.robot_ids[i];
    const auto &   p  = msg.goals[i];

    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "map";
    sphere.header.stamp    = stamp;
    sphere.ns              = "llm_panel/goals";
    sphere.id              = static_cast<int>(id) * 2;
    sphere.type            = visualization_msgs::msg::Marker::SPHERE;
    sphere.action          = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position   = p;
    sphere.pose.position.z = 0.2;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = 0.4;
    sphere.scale.y = 0.4;
    sphere.scale.z = 0.4;
    sphere.color    = robotColor(id);
    sphere.lifetime = lifetime;
    ma.markers.push_back(sphere);

    visualization_msgs::msg::Marker label;
    label.header           = sphere.header;
    label.ns               = "llm_panel/goals";
    label.id               = static_cast<int>(id) * 2 + 1;
    label.type             = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action           = visualization_msgs::msg::Marker::ADD;
    label.pose.position    = p;
    label.pose.position.z  = 0.7;
    label.pose.orientation.w = 1.0;
    label.scale.z          = 0.35;
    label.color.r = 1.0; label.color.g = 1.0; label.color.b = 1.0;
    label.color.a = 1.0;
    label.text     = "robot_" + std::to_string(id);
    label.lifetime = lifetime;
    ma.markers.push_back(label);
  }

  marker_pub_->publish(ma);
}


// ===========================================================================
// Persistence — nothing to save in MVP, but we honour the contract.
// ===========================================================================

void LlmPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void LlmPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace iros_llm_rviz_panel

PLUGINLIB_EXPORT_CLASS(iros_llm_rviz_panel::LlmPanel, rviz_common::Panel)
