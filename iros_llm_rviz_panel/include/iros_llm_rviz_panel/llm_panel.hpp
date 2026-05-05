// Copyright 2026 — iros_llm_swarm contributors. Apache-2.0.
//
// LlmPanel — RViz2 operator panel.
//
// MVP scope:
//   - top status bar (mode / active_action / action_status / llm_thinking /
//     last_error) driven by /bt/state, with a STOP ALL button that publishes
//     LlmCommand{mode=idle} on /llm/command
//   - Chat tab: action client to /llm/chat with streaming via feedback
//   - MarkerArray publisher on /llm_panel/markers — goal spheres + labels for
//     each robot when mode=mapf
//
// Threading: ROS callbacks fire on the executor thread, GUI updates run on
// the Qt main thread. The two are bridged exclusively through Qt signals
// with Qt::QueuedConnection — no widget is touched from a ROS callback.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QString>
#include <QWidget>

#include <rviz_common/panel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <iros_llm_swarm_interfaces/msg/bt_state.hpp>
#include <iros_llm_swarm_interfaces/action/llm_command.hpp>
#include <iros_llm_swarm_interfaces/action/llm_chat.hpp>

class QLabel;
class QLineEdit;
class QPushButton;
class QTabWidget;
class QTextEdit;

namespace iros_llm_rviz_panel
{

class LlmPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  using BTState    = iros_llm_swarm_interfaces::msg::BTState;
  using LlmCommand = iros_llm_swarm_interfaces::action::LlmCommand;
  using LlmChat    = iros_llm_swarm_interfaces::action::LlmChat;
  using ChatGoalH  = rclcpp_action::ClientGoalHandle<LlmChat>;

  explicit LlmPanel(QWidget * parent = nullptr);
  ~LlmPanel() override;

  // rviz_common::Panel interface
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

Q_SIGNALS:
  // Emitted from the ROS executor thread, consumed on the Qt main thread.
  void btStateReceived(QString mode,
                       QString action_status,
                       QString active_action,
                       QString last_error,
                       bool    llm_thinking);
  void chatChunkReceived(QString chunk);
  void chatStageReceived(QString stage);
  void chatFinished(bool success, QString info);

private Q_SLOTS:
  // User-facing actions
  void onSendChat();
  void onStopAll();

  // GUI-thread handlers for ROS-thread signals
  void onUpdateStatus(QString mode,
                      QString action_status,
                      QString active_action,
                      QString last_error,
                      bool    llm_thinking);
  void onChatChunk(QString chunk);
  void onChatStage(QString stage);
  void onChatFinished(bool success, QString info);

private:
  void buildUi();
  void setupRos();
  void publishMarkers(const BTState & msg);

  // Shared ROS node — borrowed from RViz, never owned.
  rclcpp::Node::SharedPtr node_;

  // Subscriptions / publishers / clients
  rclcpp::Subscription<BTState>::SharedPtr                          bt_state_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_action::Client<LlmChat>::SharedPtr                         chat_client_;
  rclcpp_action::Client<LlmCommand>::SharedPtr                      cmd_client_;

  // ---- Widgets — top status bar -----------------------------------------
  QLabel      * mode_badge_     {nullptr};
  QLabel      * action_badge_   {nullptr};
  QLabel      * status_badge_   {nullptr};
  QLabel      * thinking_label_ {nullptr};
  QLabel      * error_label_    {nullptr};
  QPushButton * stop_button_    {nullptr};

  // ---- Widgets — chat tab -----------------------------------------------
  QTextEdit   * chat_view_      {nullptr};
  QLineEdit   * chat_input_     {nullptr};
  QPushButton * send_button_    {nullptr};

  QTabWidget  * tabs_           {nullptr};

  // ---- State ------------------------------------------------------------
  bool chat_in_flight_{false};
};

}  // namespace iros_llm_rviz_panel
