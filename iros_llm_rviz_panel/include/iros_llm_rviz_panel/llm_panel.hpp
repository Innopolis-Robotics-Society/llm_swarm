// Copyright 2026 — iros_llm_swarm contributors. Apache-2.0.
//
// LlmPanel — RViz2 operator panel.
//
// Phases (see PANEL_ROADMAP.md):
//   MVP    — top status bar, Chat tab with streaming, STOP ALL, goal markers
//   2      — MAPF Live tab: counters, progress bar, sparklines, robot table
//   3      — LLM Events tab: /llm/events log with channel filters
//   4      — Preview & Execute via /llm/execute_plan (operator-confirmed)
//   5      — Quick actions strip (per-group home buttons)
//   6      — TF-aware arrows (robot pose → goal) and formation polygon
//   8      — Settings persistence via Panel::load/save
//
// Threading: ROS callbacks fire on the executor thread, GUI updates run on
// the Qt main thread. The two are bridged exclusively through Qt signals
// with Qt::QueuedConnection — no widget is touched from a ROS callback.

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <QString>
#include <QWidget>

#include <rviz_common/panel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <iros_llm_swarm_interfaces/msg/bt_state.hpp>
#include <iros_llm_swarm_interfaces/msg/llm_event.hpp>
#include <iros_llm_swarm_interfaces/action/llm_command.hpp>
#include <iros_llm_swarm_interfaces/action/llm_chat.hpp>
#include <iros_llm_swarm_interfaces/action/llm_execute_plan.hpp>

class QCheckBox;
class QHBoxLayout;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;
class QTabWidget;
class QTableWidget;
class QTextEdit;
class QTimer;
class QTreeWidget;

namespace iros_llm_rviz_panel
{

class Sparkline;

class LlmPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  using BTState         = iros_llm_swarm_interfaces::msg::BTState;
  using LlmEvent        = iros_llm_swarm_interfaces::msg::LlmEvent;
  using LlmCommand      = iros_llm_swarm_interfaces::action::LlmCommand;
  using LlmChat         = iros_llm_swarm_interfaces::action::LlmChat;
  using LlmExecutePlan  = iros_llm_swarm_interfaces::action::LlmExecutePlan;
  using ChatGoalH       = rclcpp_action::ClientGoalHandle<LlmChat>;
  using ExecGoalH       = rclcpp_action::ClientGoalHandle<LlmExecutePlan>;

  explicit LlmPanel(QWidget * parent = nullptr);
  ~LlmPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

Q_SIGNALS:
  // ROS-thread → GUI-thread bridges. Always wired with Qt::QueuedConnection.
  void btStateReceived(QString mode,
                       QString action_status,
                       QString active_action,
                       QString last_error,
                       QString action_summary,
                       bool    llm_thinking);
  void chatChunkReceived(QString chunk);
  void chatStageReceived(QString stage, QString detail);
  void chatFinished(bool success, QString info,
                    QString final_reply, QString plan_json);
  void execStageReceived(QString stage, QString detail);
  void execFinished(bool success, QString info);
  void eventReceived(qint64 stamp_ms, int channel,
                     QString trigger, QString output, QString reason);

private Q_SLOTS:
  // User actions
  void onSendChat();
  void onStopAll();
  void onPreviewToggled(bool on);
  void onExecutePending();
  void onCancelPending();

  // ROS-thread signal handlers
  void onUpdateStatus(QString mode,
                      QString action_status,
                      QString active_action,
                      QString last_error,
                      QString action_summary,
                      bool    llm_thinking);
  void onChatChunk(QString chunk);
  void onChatStage(QString stage, QString detail);
  void onChatFinished(bool success, QString info,
                      QString final_reply, QString plan_json);
  void onExecStage(QString stage, QString detail);
  void onExecFinished(bool success, QString info);
  void onEventReceived(qint64 stamp_ms, int channel,
                       QString trigger, QString output, QString reason);

  // Periodic refresh — sparklines, TF arrows.
  void onMarkerTick();

private:
  // ---- Construction ------------------------------------------------------
  void buildUi();
  void buildStatusRow(QHBoxLayout * row);
  void buildChatTab(QWidget * tab);
  void buildMapfTab(QWidget * tab);
  void buildEventsTab(QWidget * tab);
  void buildBtTab(QWidget * tab);
  void setupRos();

  // ---- Map YAML ----------------------------------------------------------
  struct RobotGroup {
    std::string         color;
    std::vector<int>    ids;
    std::pair<double, double> home{0.0, 0.0};
    std::unordered_map<std::string, std::pair<double, double>> spawn;
  };
  struct FormationZone {
    std::string name;
    std::pair<double, double> coords{0.0, 0.0};
    double radius{0.0};
  };
  void loadMapYaml();

  // ---- Chat / Exec dispatch ---------------------------------------------
  void sendChatGoal(const QString & text, bool execute_after_planning);

  // ---- Markers -----------------------------------------------------------
  void publishGoalMarkers(const BTState & msg);
  void publishArrowMarkers();
  void publishFormationPolygon();
  void publishPreviewMarkers();
  void clearPreviewMarkers();

  // ---- Plan tree ---------------------------------------------------------
  void renderPlanTree(const QString & plan_json);

  // Shared ROS node — borrowed from RViz, never owned.
  rclcpp::Node::SharedPtr node_;

  // Subscriptions / publishers / clients
  rclcpp::Subscription<BTState>::SharedPtr                          bt_state_sub_;
  rclcpp::Subscription<LlmEvent>::SharedPtr                         events_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_action::Client<LlmChat>::SharedPtr                         chat_client_;
  rclcpp_action::Client<LlmCommand>::SharedPtr                      cmd_client_;
  rclcpp_action::Client<LlmExecutePlan>::SharedPtr                  exec_client_;

  // TF
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- Widgets — top status bar -----------------------------------------
  QLabel      * mode_badge_     {nullptr};
  QLabel      * action_badge_   {nullptr};
  QLabel      * status_badge_   {nullptr};
  QLabel      * thinking_label_ {nullptr};
  QLabel      * error_label_    {nullptr};
  QPushButton * stop_button_    {nullptr};

  // ---- Widgets — chat tab -----------------------------------------------
  QTextEdit   * chat_view_              {nullptr};
  QLineEdit   * chat_input_             {nullptr};
  QPushButton * send_button_            {nullptr};
  QCheckBox   * preview_mode_checkbox_  {nullptr};
  QTreeWidget * plan_tree_view_         {nullptr};
  QPushButton * execute_button_         {nullptr};
  QPushButton * cancel_button_          {nullptr};

  // ---- Widgets — MAPF tab -----------------------------------------------
  QProgressBar * mapf_progress_   {nullptr};
  QLabel       * mapf_arrived_    {nullptr};
  QLabel       * mapf_active_     {nullptr};
  QLabel       * mapf_stalled_    {nullptr};
  QLabel       * mapf_replans_    {nullptr};
  QLabel       * mapf_event_tail_ {nullptr};
  Sparkline    * mapf_active_spark_  {nullptr};
  Sparkline    * mapf_arrived_spark_ {nullptr};
  QTableWidget * mapf_robot_table_   {nullptr};

  // ---- Widgets — events tab ---------------------------------------------
  QTableWidget * events_table_              {nullptr};
  QCheckBox    * event_filter_decision_     {nullptr};
  QCheckBox    * event_filter_observer_     {nullptr};
  QCheckBox    * event_filter_user_         {nullptr};

  // ---- Widgets — BT tab -------------------------------------------------
  QTableWidget * bt_props_table_  {nullptr};
  QTableWidget * bt_log_table_    {nullptr};

  QTabWidget   * tabs_   {nullptr};

  // ---- Cached BT state for marker timer + BT tab (read on GUI thread) --
  std::mutex                                cached_state_mutex_;
  std::string                               cached_mode_;
  std::vector<uint32_t>                     cached_robot_ids_;
  std::vector<geometry_msgs::msg::Point>    cached_goals_;
  std::string                               cached_leader_ns_;
  std::string                               cached_formation_id_;
  int64_t                                   cached_stamp_ms_   {0};

  // ---- Plan preview state ------------------------------------------------
  QString  pending_plan_json_;
  bool     preview_mode_  {false};

  // ---- Chat in-flight ----------------------------------------------------
  bool chat_in_flight_ {false};

  // ---- MAPF tab finalization --------------------------------------------
  // BT publishes action_summary while mapf is in flight, but stops once
  // mode flips to idle — counters would otherwise freeze on the last
  // in-flight snapshot (e.g. arrived=3/active=1 even after all 4 arrived).
  // Cache the last parsed values so we can snap to completion on the
  // mapf → idle transition.
  QString prev_mode_;
  int     last_arrived_int_ {0};
  int     last_active_int_  {0};
  bool    have_mapf_data_   {false};

  // ---- BT transition log state ------------------------------------------
  // Compared against the latest BTState to spot mode/status/action/error
  // changes; suppressed on the very first message via bt_state_seen_.
  QString prev_action_status_;
  QString prev_active_action_;
  QString prev_last_error_;
  bool    bt_state_seen_ {false};
  static constexpr int kBtLogMax = 200;

  // ---- Map context (for Phase 5/6) --------------------------------------
  std::string                  map_name_      {"cave"};
  std::vector<RobotGroup>      groups_;
  std::vector<FormationZone>   formation_zones_;
  double                       bounds_xmin_{-1e9}, bounds_xmax_{1e9};
  double                       bounds_ymin_{-1e9}, bounds_ymax_{1e9};

  // ---- Periodic refresh --------------------------------------------------
  QTimer * marker_timer_ {nullptr};

  // ---- Events buffering --------------------------------------------------
  static constexpr int kMaxEvents = 500;
};

}  // namespace iros_llm_rviz_panel
