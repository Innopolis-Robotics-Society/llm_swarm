// Copyright 2026 — iros_llm_swarm contributors. Apache-2.0.

#include "iros_llm_rviz_panel/llm_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_set>

#include <QCheckBox>
#include <QColor>
#include <QFont>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextCursor>
#include <QTextEdit>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <yaml-cpp/yaml.h>

#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <rmw/rmw.h>

#include "iros_llm_rviz_panel/action_summary.hpp"
#include "iros_llm_rviz_panel/sparkline.hpp"

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
  if (mode == "mapf")      return "#1976D2";
  if (mode == "formation") return "#7B1FA2";
  return "#616161";
}

QString statusColor(const QString & status)
{
  if (status == "OK")     return "#388E3C";
  if (status == "WARN")   return "#F57C00";
  if (status == "ERROR")  return "#C62828";
  if (status == "HALTED") return "#6A1B9A";
  return "#616161";
}

QString channelColor(int channel)
{
  switch (channel) {
    case 1: return "#1976D2";   // DECISION — blue
    case 2: return "#F57C00";   // OBSERVER — orange
    case 3: return "#388E3C";   // USER — green
    default: return "#616161";
  }
}

QString channelName(int channel)
{
  switch (channel) {
    case 1: return "DECISION";
    case 2: return "OBSERVER";
    case 3: return "USER";
    default: return "?";
  }
}

QString fmtCoord(double x, double y)
{
  return QString("(%1, %2)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2);
}

QString fmtTime(qint64 stamp_ms)
{
  const std::time_t t = static_cast<std::time_t>(stamp_ms / 1000);
  std::tm tmv;
  localtime_r(&t, &tmv);
  std::ostringstream oss;
  oss << std::put_time(&tmv, "%H:%M:%S");
  return QString::fromStdString(oss.str()) +
         QString(".%1").arg(stamp_ms % 1000, 3, 10, QChar('0'));
}

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

  auto * status_row = new QHBoxLayout();
  status_row->setSpacing(6);
  buildStatusRow(status_row);
  root->addLayout(status_row);

  tabs_ = new QTabWidget(this);

  auto * chat_tab = new QWidget();
  buildChatTab(chat_tab);
  tabs_->addTab(chat_tab, "Chat");

  auto * mapf_tab = new QWidget();
  buildMapfTab(mapf_tab);
  tabs_->addTab(mapf_tab, "MAPF");

  auto * events_tab = new QWidget();
  buildEventsTab(events_tab);
  tabs_->addTab(events_tab, "Events");

  auto * bt_tab = new QWidget();
  buildBtTab(bt_tab);
  tabs_->addTab(bt_tab, "BT");

  auto * info_tab = new QWidget();
  buildInfoTab(info_tab);
  tabs_->addTab(info_tab, "Info");

  root->addWidget(tabs_, /*stretch=*/1);

  // ---- Wiring -------------------------------------------------------------
  connect(send_button_, &QPushButton::clicked,
          this, &LlmPanel::onSendChat);
  connect(chat_input_,  &QLineEdit::returnPressed,
          this, &LlmPanel::onSendChat);
  connect(stop_button_, &QPushButton::clicked,
          this, &LlmPanel::onStopAll);
  connect(preview_mode_checkbox_, &QCheckBox::toggled,
          this, &LlmPanel::onPreviewToggled);
  connect(execute_button_, &QPushButton::clicked,
          this, &LlmPanel::onExecutePending);
  connect(cancel_button_,  &QPushButton::clicked,
          this, &LlmPanel::onCancelPending);

  // Cross-thread signals
  connect(this, &LlmPanel::btStateReceived,
          this, &LlmPanel::onUpdateStatus, Qt::QueuedConnection);
  connect(this, &LlmPanel::chatChunkReceived,
          this, &LlmPanel::onChatChunk,    Qt::QueuedConnection);
  connect(this, &LlmPanel::chatStageReceived,
          this, &LlmPanel::onChatStage,    Qt::QueuedConnection);
  connect(this, &LlmPanel::chatFinished,
          this, &LlmPanel::onChatFinished, Qt::QueuedConnection);
  connect(this, &LlmPanel::execStageReceived,
          this, &LlmPanel::onExecStage,    Qt::QueuedConnection);
  connect(this, &LlmPanel::execFinished,
          this, &LlmPanel::onExecFinished, Qt::QueuedConnection);
  connect(this, &LlmPanel::eventReceived,
          this, &LlmPanel::onEventReceived, Qt::QueuedConnection);
  connect(this, &LlmPanel::systemInfoChanged,
          this, &LlmPanel::onSystemInfoChanged, Qt::QueuedConnection);

  // 30 Hz refresh — sparkline updates if dirty, plus TF arrow republish.
  marker_timer_ = new QTimer(this);
  connect(marker_timer_, &QTimer::timeout,
          this, &LlmPanel::onMarkerTick);

  // Info tab refresh — graph + remote params, ~0.5 Hz.
  info_timer_ = new QTimer(this);
  connect(info_timer_, &QTimer::timeout,
          this, &LlmPanel::onInfoRefresh);
}

void LlmPanel::buildStatusRow(QHBoxLayout * row)
{
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

  row->addWidget(mode_badge_);
  row->addWidget(action_badge_);
  row->addWidget(status_badge_);
  row->addWidget(thinking_label_);
  row->addWidget(error_label_, /*stretch=*/1);
  row->addWidget(stop_button_);
}

void LlmPanel::buildChatTab(QWidget * tab)
{
  auto * layout = new QVBoxLayout(tab);
  layout->setContentsMargins(0, 0, 0, 0);

  chat_view_ = new QTextEdit();
  chat_view_->setReadOnly(true);
  chat_view_->setStyleSheet("QTextEdit { font-family: monospace; }");

  auto * input_row = new QHBoxLayout();
  chat_input_ = new QLineEdit();
  chat_input_->setPlaceholderText(
    "Type a command... (e.g. \"cyan to hub\", \"stop\")");
  send_button_ = new QPushButton("Send");
  preview_mode_checkbox_ = new QCheckBox("Preview");
  preview_mode_checkbox_->setToolTip(
    "When on: panel previews the parsed plan and waits for Execute.");
  input_row->addWidget(chat_input_, /*stretch=*/1);
  input_row->addWidget(preview_mode_checkbox_);
  input_row->addWidget(send_button_);

  plan_tree_view_ = new QTreeWidget();
  plan_tree_view_->setHeaderLabels(QStringList() << "node" << "info");
  plan_tree_view_->setRootIsDecorated(true);
  plan_tree_view_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  plan_tree_view_->header()->setSectionResizeMode(1, QHeaderView::Stretch);
  plan_tree_view_->setMaximumHeight(160);
  plan_tree_view_->setVisible(false);

  auto * confirm_row = new QHBoxLayout();
  execute_button_ = new QPushButton("Execute");
  execute_button_->setStyleSheet(
    "QPushButton { background-color: #388E3C; color: white; font-weight: bold; "
    "padding: 4px 12px; border-radius: 4px; }"
    "QPushButton:disabled { background-color: #757575; }");
  cancel_button_  = new QPushButton("Cancel");
  execute_button_->setEnabled(false);
  cancel_button_->setEnabled(false);
  confirm_row->addStretch(1);
  confirm_row->addWidget(cancel_button_);
  confirm_row->addWidget(execute_button_);

  layout->addWidget(chat_view_, /*stretch=*/1);
  layout->addWidget(plan_tree_view_);
  layout->addLayout(confirm_row);
  layout->addLayout(input_row);
}

void LlmPanel::buildMapfTab(QWidget * tab)
{
  auto * layout = new QVBoxLayout(tab);
  layout->setContentsMargins(2, 2, 2, 2);
  layout->setSpacing(6);

  mapf_progress_ = new QProgressBar();
  mapf_progress_->setRange(0, 20);
  mapf_progress_->setFormat("arrived %v / %m");
  layout->addWidget(mapf_progress_);

  auto * counters = new QHBoxLayout();
  auto makeCounter = [](const QString & title, QLabel ** out) -> QWidget *
  {
    auto * w = new QWidget();
    auto * l = new QVBoxLayout(w);
    l->setContentsMargins(2, 2, 2, 2);
    l->setSpacing(0);
    auto * t = new QLabel(title);
    t->setStyleSheet("QLabel { color: #888; font-size: 10px; }");
    *out = new QLabel("—");
    (*out)->setStyleSheet("QLabel { font-size: 18px; font-weight: bold; }");
    l->addWidget(t);
    l->addWidget(*out);
    return w;
  };
  counters->addWidget(makeCounter("arrived", &mapf_arrived_));
  counters->addWidget(makeCounter("active",  &mapf_active_));
  counters->addWidget(makeCounter("stalled", &mapf_stalled_));
  counters->addWidget(makeCounter("replans", &mapf_replans_));
  counters->addStretch(1);
  layout->addLayout(counters);

  auto * sparks = new QHBoxLayout();
  mapf_active_spark_  = new Sparkline();
  mapf_arrived_spark_ = new Sparkline();
  mapf_arrived_spark_->setColor(QColor(0x38, 0x8E, 0x3C));
  auto * sa = new QLabel("active:");
  auto * sb = new QLabel("arrived:");
  sparks->addWidget(sa);
  sparks->addWidget(mapf_active_spark_, /*stretch=*/1);
  sparks->addWidget(sb);
  sparks->addWidget(mapf_arrived_spark_, /*stretch=*/1);
  layout->addLayout(sparks);

  mapf_event_tail_ = new QLabel("");
  mapf_event_tail_->setStyleSheet("QLabel { color: #888; font-style: italic; }");
  mapf_event_tail_->setWordWrap(true);
  layout->addWidget(mapf_event_tail_);

  mapf_robot_table_ = new QTableWidget(0, 3);
  mapf_robot_table_->setHorizontalHeaderLabels(
    QStringList() << "id" << "goal" << "dist (m)");
  mapf_robot_table_->verticalHeader()->setVisible(false);
  mapf_robot_table_->horizontalHeader()->setSectionResizeMode(
    0, QHeaderView::ResizeToContents);
  mapf_robot_table_->horizontalHeader()->setSectionResizeMode(
    1, QHeaderView::Stretch);
  mapf_robot_table_->horizontalHeader()->setSectionResizeMode(
    2, QHeaderView::ResizeToContents);
  mapf_robot_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  layout->addWidget(mapf_robot_table_, /*stretch=*/1);
}

void LlmPanel::buildEventsTab(QWidget * tab)
{
  auto * layout = new QVBoxLayout(tab);
  layout->setContentsMargins(2, 2, 2, 2);
  layout->setSpacing(4);

  auto * filter_row = new QHBoxLayout();
  filter_row->addWidget(new QLabel("Filter:"));
  event_filter_decision_ = new QCheckBox("DECISION");
  event_filter_observer_ = new QCheckBox("OBSERVER");
  event_filter_user_     = new QCheckBox("USER");
  event_filter_decision_->setChecked(true);
  event_filter_observer_->setChecked(true);
  event_filter_user_->setChecked(true);
  filter_row->addWidget(event_filter_decision_);
  filter_row->addWidget(event_filter_observer_);
  filter_row->addWidget(event_filter_user_);
  filter_row->addStretch(1);
  layout->addLayout(filter_row);

  events_table_ = new QTableWidget(0, 5);
  events_table_->setHorizontalHeaderLabels(
    QStringList() << "time" << "channel" << "trigger" << "output" << "reason");
  events_table_->verticalHeader()->setVisible(false);
  events_table_->horizontalHeader()->setSectionResizeMode(
    0, QHeaderView::ResizeToContents);
  events_table_->horizontalHeader()->setSectionResizeMode(
    1, QHeaderView::ResizeToContents);
  events_table_->horizontalHeader()->setSectionResizeMode(
    2, QHeaderView::Stretch);
  events_table_->horizontalHeader()->setSectionResizeMode(
    3, QHeaderView::Stretch);
  events_table_->horizontalHeader()->setSectionResizeMode(
    4, QHeaderView::Stretch);
  events_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  events_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  events_table_->setWordWrap(false);
  layout->addWidget(events_table_, /*stretch=*/1);

  // Re-apply filters on toggle.
  auto reapply = [this]()
  {
    for (int row = 0; row < events_table_->rowCount(); ++row) {
      auto * item = events_table_->item(row, 1);
      if (!item) continue;
      const QString name = item->text();
      const bool visible =
        (name == "DECISION" && event_filter_decision_->isChecked()) ||
        (name == "OBSERVER" && event_filter_observer_->isChecked()) ||
        (name == "USER"     && event_filter_user_->isChecked());
      events_table_->setRowHidden(row, !visible);
    }
  };
  connect(event_filter_decision_, &QCheckBox::toggled, this, reapply);
  connect(event_filter_observer_, &QCheckBox::toggled, this, reapply);
  connect(event_filter_user_,     &QCheckBox::toggled, this, reapply);
}

void LlmPanel::buildBtTab(QWidget * tab)
{
  auto * layout = new QVBoxLayout(tab);
  layout->setContentsMargins(2, 2, 2, 2);
  layout->setSpacing(4);

  layout->addWidget(new QLabel("Live /bt/state — refreshed each tick:"));

  static constexpr int kRows = 11;
  static const char * kFields[kRows] = {
    "mode", "action_status", "active_action", "last_error",
    "formation_id", "leader_ns", "robots", "goals",
    "llm_thinking", "stamp", "action_summary",
  };

  bt_props_table_ = new QTableWidget(kRows, 2);
  bt_props_table_->setHorizontalHeaderLabels(QStringList() << "field" << "value");
  bt_props_table_->verticalHeader()->setVisible(false);
  bt_props_table_->horizontalHeader()->setSectionResizeMode(
    0, QHeaderView::ResizeToContents);
  bt_props_table_->horizontalHeader()->setSectionResizeMode(
    1, QHeaderView::Stretch);
  bt_props_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  bt_props_table_->setSelectionMode(QAbstractItemView::NoSelection);
  bt_props_table_->setAlternatingRowColors(true);
  bt_props_table_->setMaximumHeight(280);
  for (int i = 0; i < kRows; ++i) {
    auto * fld = new QTableWidgetItem(QString::fromLatin1(kFields[i]));
    QFont f = fld->font();
    f.setBold(true);
    fld->setFont(f);
    bt_props_table_->setItem(i, 0, fld);
    bt_props_table_->setItem(i, 1, new QTableWidgetItem(QString::fromUtf8("—")));
  }
  layout->addWidget(bt_props_table_);

  layout->addWidget(new QLabel(
    "Recent transitions (mode / action_status / active_action / last_error):"));

  bt_log_table_ = new QTableWidget(0, 4);
  bt_log_table_->setHorizontalHeaderLabels(
    QStringList() << "time" << "field" << "from" << "to");
  bt_log_table_->verticalHeader()->setVisible(false);
  bt_log_table_->horizontalHeader()->setSectionResizeMode(
    0, QHeaderView::ResizeToContents);
  bt_log_table_->horizontalHeader()->setSectionResizeMode(
    1, QHeaderView::ResizeToContents);
  bt_log_table_->horizontalHeader()->setSectionResizeMode(
    2, QHeaderView::Stretch);
  bt_log_table_->horizontalHeader()->setSectionResizeMode(
    3, QHeaderView::Stretch);
  bt_log_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  bt_log_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  layout->addWidget(bt_log_table_, /*stretch=*/1);
}

void LlmPanel::buildInfoTab(QWidget * tab)
{
  auto * layout = new QVBoxLayout(tab);
  layout->setContentsMargins(2, 2, 2, 2);
  layout->setSpacing(4);

  layout->addWidget(new QLabel(
    "System info — discovered from ROS graph and remote parameters:"));

  info_table_ = new QTableWidget(0, 2);
  info_table_->setHorizontalHeaderLabels(QStringList() << "key" << "value");
  info_table_->verticalHeader()->setVisible(false);
  info_table_->horizontalHeader()->setSectionResizeMode(
    0, QHeaderView::ResizeToContents);
  info_table_->horizontalHeader()->setSectionResizeMode(
    1, QHeaderView::Stretch);
  info_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  info_table_->setAlternatingRowColors(true);
  layout->addWidget(info_table_, /*stretch=*/1);

  info_updated_lbl_ = new QLabel("waiting for first refresh...");
  info_updated_lbl_->setStyleSheet("QLabel { color: #888; font-style: italic; }");
  layout->addWidget(info_updated_lbl_);
}


// ===========================================================================
// ROS setup
// ===========================================================================

void LlmPanel::onInitialize()
{
  setupRos();
  loadMapYaml();
  if (marker_timer_) {
    marker_timer_->start(33);   // ~30 Hz
  }
  if (info_timer_) {
    info_timer_->start(2000);   // ~0.5 Hz — cheap graph poll + async params
    onInfoRefresh();            // first paint without waiting 2 s
  }
}

void LlmPanel::setupRos()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    chat_view_->append(
      "<span style='color:#C62828'>"
      "[fatal] could not acquire RViz ROS node"
      "</span>");
    return;
  }
  node_ = abstraction->get_raw_node();

  // /bt/state is published RELIABLE with depth 20 by the BT runner so
  // short-lived terminal states (HALTED / ERROR) survive the ~100ms
  // window between FAILURE and the next idle snapshot. Match here.
  rclcpp::QoS bt_qos(20);
  bt_qos.reliable();
  bt_state_sub_ = node_->create_subscription<BTState>(
    "/bt/state", bt_qos,
    [this](BTState::ConstSharedPtr msg)
    {
      // Cache for the marker timer (TF arrows + formation polygon) and
      // the BT-state tab properties grid.
      {
        std::lock_guard<std::mutex> lk(cached_state_mutex_);
        cached_mode_         = msg->mode;
        cached_robot_ids_    = msg->robot_ids;
        cached_goals_        = msg->goals;
        cached_leader_ns_    = msg->leader_ns;
        cached_formation_id_ = msg->formation_id;
        cached_stamp_ms_     = msg->stamp_ms;
      }

      Q_EMIT btStateReceived(
        QString::fromStdString(msg->mode),
        QString::fromStdString(msg->action_status),
        QString::fromStdString(msg->active_action),
        QString::fromStdString(msg->last_error),
        QString::fromStdString(msg->action_summary),
        msg->llm_thinking);

      publishGoalMarkers(*msg);
    });

  events_sub_ = node_->create_subscription<LlmEvent>(
    "/llm/events", rclcpp::QoS(50).reliable(),
    [this](LlmEvent::ConstSharedPtr ev)
    {
      Q_EMIT eventReceived(
        static_cast<qint64>(ev->stamp_ms),
        static_cast<int>(ev->channel),
        QString::fromStdString(ev->trigger),
        QString::fromStdString(ev->output),
        QString::fromStdString(ev->reason));
    });

  chat_client_ = rclcpp_action::create_client<LlmChat>(node_, "/llm/chat");
  cmd_client_  = rclcpp_action::create_client<LlmCommand>(node_, "/llm/command");
  exec_client_ = rclcpp_action::create_client<LlmExecutePlan>(
    node_, "/llm/execute_plan");

  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/llm_panel/markers", rclcpp::QoS(10));

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(node_->get_logger(),
    "LlmPanel ready. /bt/state, /llm/events, /llm_panel/markers, "
    "/llm/chat, /llm/command, /llm/execute_plan.");
}


// ===========================================================================
// Map YAML — for Quick Actions and bounds checks
// ===========================================================================

void LlmPanel::loadMapYaml()
{
  groups_.clear();
  formation_zones_.clear();

  std::string yaml_path;
  try {
    yaml_path = ament_index_cpp::get_package_share_directory(
      "iros_llm_orchestrator") + "/prompts/maps/" + map_name_ + ".yaml";
  } catch (const std::exception &) {
    return;
  }

  try {
    YAML::Node root = YAML::LoadFile(yaml_path);

    if (root["bounds"]) {
      const auto b = root["bounds"];
      if (b["x_min"]) bounds_xmin_ = b["x_min"].as<double>();
      if (b["x_max"]) bounds_xmax_ = b["x_max"].as<double>();
      if (b["y_min"]) bounds_ymin_ = b["y_min"].as<double>();
      if (b["y_max"]) bounds_ymax_ = b["y_max"].as<double>();
    }

    if (root["robot_groups"]) {
      for (const auto & kv : root["robot_groups"]) {
        RobotGroup g;
        g.color = kv.first.as<std::string>();
        if (kv.second["color"]) {
          g.color = kv.second["color"].as<std::string>();
        }
        if (kv.second["ids"]) {
          for (const auto & id : kv.second["ids"]) {
            g.ids.push_back(id.as<int>());
          }
        }
        if (kv.second["home"] && kv.second["home"].size() >= 2) {
          g.home = {kv.second["home"][0].as<double>(),
                    kv.second["home"][1].as<double>()};
        }
        if (kv.second["spawn"]) {
          for (const auto & sp : kv.second["spawn"]) {
            const auto rname = sp.first.as<std::string>();
            if (sp.second.size() >= 2) {
              g.spawn[rname] = {sp.second[0].as<double>(),
                                sp.second[1].as<double>()};
            }
          }
        }
        if (!g.ids.empty()) {
          groups_.push_back(std::move(g));
        }
      }
    }

    if (root["formation_zones"]) {
      for (const auto & z : root["formation_zones"]) {
        FormationZone fz;
        fz.name = z["name"].as<std::string>("");
        if (z["coords"] && z["coords"].size() >= 2) {
          fz.coords = {z["coords"][0].as<double>(),
                       z["coords"][1].as<double>()};
        }
        fz.radius = z["radius"].as<double>(0.0);
        formation_zones_.push_back(std::move(fz));
      }
    }
  } catch (const std::exception & exc) {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(),
        "Failed to parse %s: %s", yaml_path.c_str(), exc.what());
    }
  }
}

// ===========================================================================
// Status bar (GUI thread)
// ===========================================================================

void LlmPanel::onUpdateStatus(
  QString mode,
  QString action_status,
  QString active_action,
  QString last_error,
  QString action_summary,
  bool    llm_thinking)
{
  mode_badge_->setText(mode);
  mode_badge_->setStyleSheet(badgeStyle(modeColor(mode)));

  action_badge_->setText(active_action.isEmpty() ? "none" : active_action);

  // Sticky terminal state — keep ERROR/HALTED + last_error visible for
  // kStickyMs after the BT cleared, so brief failures (rejected goal,
  // halt) don't flash past in 100 ms.
  const bool is_failure_status =
    !action_status.isEmpty() && action_status != "OK";
  const auto now_tp = std::chrono::steady_clock::now();
  if (is_failure_status || !last_error.isEmpty()) {
    sticky_action_status_ = is_failure_status ? action_status : QString();
    sticky_last_error_    = last_error;
    sticky_until_ = now_tp + std::chrono::milliseconds(kStickyMs);
  }

  QString display_status = action_status;
  QString display_error  = last_error;
  if (now_tp < sticky_until_) {
    if (!sticky_action_status_.isEmpty()) display_status = sticky_action_status_;
    if (!sticky_last_error_.isEmpty())    display_error  = sticky_last_error_;
  }

  status_badge_->setText(display_status);
  status_badge_->setStyleSheet(badgeStyle(statusColor(display_status)));

  thinking_label_->setText(llm_thinking ? QString::fromUtf8("🧠") : QString());
  error_label_->setText(display_error);

  // Phase 2 — parse action_summary, drive MAPF tab widgets.
  const auto s = parseActionSummary(action_summary.toStdString());
  if (s.arrived) {
    last_arrived_int_ = *s.arrived;
    have_mapf_data_   = true;
    mapf_arrived_->setText(QString::number(*s.arrived));
    if (s.arrived && s.active) {
      const int total = *s.arrived + *s.active;
      mapf_progress_->setMaximum(std::max(1, total));
      mapf_progress_->setValue(*s.arrived);
    }
    mapf_arrived_spark_->push(static_cast<double>(*s.arrived));
  }
  if (s.active) {
    last_active_int_ = *s.active;
    have_mapf_data_  = true;
    mapf_active_->setText(QString::number(*s.active));
    mapf_active_spark_->push(static_cast<double>(*s.active));
  }
  if (s.stall)   mapf_stalled_->setText(QString::number(*s.stall));
  if (s.replans) mapf_replans_->setText(QString::number(*s.replans));
  mapf_event_tail_->setText(QString::fromStdString(s.event_tail));

  // Snap MAPF counters to completion when the BT returns to idle. The BT
  // stops publishing fresh action_summary lines after mode flips, so the
  // live counters would otherwise stay frozen at the last in-flight
  // snapshot (typically arrived=N-1/active=1).
  if (mode == "idle" && !prev_mode_.isEmpty() && prev_mode_ != "idle"
      && have_mapf_data_)
  {
    const int total = last_arrived_int_ + last_active_int_;
    if (total > 0) {
      mapf_arrived_->setText(QString::number(total));
      mapf_active_->setText("0");
      mapf_stalled_->setText("0");
      mapf_progress_->setMaximum(std::max(1, total));
      mapf_progress_->setValue(total);
      mapf_arrived_spark_->push(static_cast<double>(total));
      mapf_active_spark_->push(0.0);
      last_arrived_int_ = total;
      last_active_int_  = 0;
    }
    have_mapf_data_ = false;
  }

  // Snapshot the rest of BTState (formation_id, stamp_ms, leader_ns) under
  // the cache lock; the marker timer also reads these fields.
  std::vector<uint32_t>                  ids;
  std::vector<geometry_msgs::msg::Point> goals;
  std::string                            formation_id;
  std::string                            leader_ns;
  int64_t                                stamp_ms = 0;
  {
    std::lock_guard<std::mutex> lk(cached_state_mutex_);
    ids          = cached_robot_ids_;
    goals        = cached_goals_;
    formation_id = cached_formation_id_;
    leader_ns    = cached_leader_ns_;
    stamp_ms     = cached_stamp_ms_;
  }

  // Robot table — repopulate from cache. Cheap, ≤20 rows.
  mapf_robot_table_->setRowCount(static_cast<int>(ids.size()));
  for (size_t i = 0; i < ids.size(); ++i) {
    const QString g_str = (i < goals.size())
      ? fmtCoord(goals[i].x, goals[i].y) : QString("—");
    mapf_robot_table_->setItem(static_cast<int>(i), 0,
      new QTableWidgetItem(QString::number(ids[i])));
    mapf_robot_table_->setItem(static_cast<int>(i), 1,
      new QTableWidgetItem(g_str));
    mapf_robot_table_->setItem(static_cast<int>(i), 2,
      new QTableWidgetItem(QString("—")));   // Phase 6 fills via TF
  }

  // BT tab — properties grid.
  if (bt_props_table_) {
    auto setCell = [this](int row, const QString & v) {
      auto * item = bt_props_table_->item(row, 1);
      if (!item) {
        item = new QTableWidgetItem();
        bt_props_table_->setItem(row, 1, item);
      }
      item->setText(v);
    };
    setCell(0,  mode);
    setCell(1,  action_status);
    setCell(2,  active_action.isEmpty() ? "none" : active_action);
    setCell(3,  last_error.isEmpty()    ? QString::fromUtf8("—") : last_error);
    setCell(4,  formation_id.empty()
                  ? QString::fromUtf8("—")
                  : QString::fromStdString(formation_id));
    setCell(5,  leader_ns.empty()
                  ? QString::fromUtf8("—")
                  : QString::fromStdString(leader_ns));
    setCell(6,  QString::number(ids.size()));
    setCell(7,  QString::number(goals.size()));
    setCell(8,  llm_thinking ? "yes" : "no");
    setCell(9,  stamp_ms > 0 ? fmtTime(static_cast<qint64>(stamp_ms))
                             : QString::fromUtf8("—"));
    setCell(10, action_summary);
  }

  // BT tab — append a row for each transition that fires on this tick.
  // Skipped on the very first message so we don't log "" → mode/status.
  if (bt_log_table_ && bt_state_seen_) {
    const QString stamp_str = stamp_ms > 0
      ? fmtTime(static_cast<qint64>(stamp_ms)) : QString::fromUtf8("—");
    auto logTransition = [&](const QString & field,
                             const QString & from,
                             const QString & to)
    {
      if (from == to) {
        return;
      }
      while (bt_log_table_->rowCount() >= kBtLogMax) {
        bt_log_table_->removeRow(0);
      }
      const int row = bt_log_table_->rowCount();
      bt_log_table_->insertRow(row);
      bt_log_table_->setItem(row, 0, new QTableWidgetItem(stamp_str));
      bt_log_table_->setItem(row, 1, new QTableWidgetItem(field));
      bt_log_table_->setItem(row, 2,
        new QTableWidgetItem(from.isEmpty() ? QString::fromUtf8("—") : from));
      bt_log_table_->setItem(row, 3,
        new QTableWidgetItem(to.isEmpty()   ? QString::fromUtf8("—") : to));
      bt_log_table_->scrollToBottom();
    };
    logTransition("mode",          prev_mode_,           mode);
    logTransition("action_status", prev_action_status_,  action_status);
    logTransition("active_action", prev_active_action_,  active_action);
    logTransition("last_error",    prev_last_error_,     last_error);
  }

  // Update transition baselines (read above) at the very end so MAPF snap
  // and transition log both see the previous tick's values.
  prev_mode_          = mode;
  prev_action_status_ = action_status;
  prev_active_action_ = active_action;
  prev_last_error_    = last_error;
  bt_state_seen_      = true;
}


// ===========================================================================
// Chat (GUI thread sends, ROS thread for action callbacks)
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

  pending_plan_json_.clear();
  plan_tree_view_->clear();
  plan_tree_view_->setVisible(false);
  execute_button_->setEnabled(false);
  cancel_button_->setEnabled(false);
  clearPreviewMarkers();

  sendChatGoal(text, /*execute_after_planning=*/!preview_mode_);
}

void LlmPanel::sendChatGoal(const QString & text, bool execute_after_planning)
{
  chat_view_->append(QString("<b>You:</b> %1").arg(text.toHtmlEscaped()));
  chat_input_->clear();
  chat_view_->append(QString::fromUtf8("<b>🤖:</b> "));
  chat_in_flight_ = true;

  LlmChat::Goal goal;
  goal.user_message           = text.toStdString();
  goal.execute_after_planning = execute_after_planning;

  rclcpp_action::Client<LlmChat>::SendGoalOptions opts;

  opts.feedback_callback =
    [this](ChatGoalH::SharedPtr,
           const std::shared_ptr<const LlmChat::Feedback> fb)
    {
      Q_EMIT chatStageReceived(
        QString::fromStdString(fb->stage),
        QString::fromStdString(fb->detail));
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
      const QString final_reply = wrapped.result
        ? QString::fromStdString(wrapped.result->final_reply)
        : QString();
      const QString plan_json = wrapped.result
        ? QString::fromStdString(wrapped.result->plan_json)
        : QString();
      Q_EMIT chatFinished(ok, info, final_reply, plan_json);
    };

  chat_client_->async_send_goal(goal, opts);
}

void LlmPanel::onChatChunk(QString chunk)
{
  auto cursor = chat_view_->textCursor();
  cursor.movePosition(QTextCursor::End);
  cursor.insertText(chunk);
  chat_view_->setTextCursor(cursor);
  chat_view_->ensureCursorVisible();
}

void LlmPanel::onChatStage(QString stage, QString detail)
{
  if (stage == "executing") {
    chat_view_->append(
      "<i style='color:#1976D2'>[executing plan...]</i>");
  } else if (stage == "parsed") {
    pending_plan_json_ = detail;
    if (preview_mode_) {
      renderPlanTree(pending_plan_json_);
      plan_tree_view_->setVisible(true);
      execute_button_->setEnabled(true);
      cancel_button_->setEnabled(true);
      publishPreviewMarkers();
    }
  } else if (stage == "error") {
    chat_view_->append("<i style='color:#C62828'>[error]</i>");
  }
}

void LlmPanel::onChatFinished(bool success, QString info,
                              QString /*final_reply*/, QString plan_json)
{
  chat_in_flight_ = false;
  if (success) {
    chat_view_->append(
      QString::fromUtf8("<span style='color:#388E3C'>✓ done</span>"));
  } else {
    chat_view_->append(
      QString("<span style='color:#C62828'>[%1]</span>")
        .arg(info.isEmpty() ? "failed" : info.toHtmlEscaped()));
  }
  if (!plan_json.isEmpty() && pending_plan_json_.isEmpty()) {
    pending_plan_json_ = plan_json;
  }
}


// ===========================================================================
// Preview & Execute (Phase 4)
// ===========================================================================

void LlmPanel::onPreviewToggled(bool on)
{
  preview_mode_ = on;
  if (!on) {
    plan_tree_view_->setVisible(false);
    execute_button_->setEnabled(false);
    cancel_button_->setEnabled(false);
    clearPreviewMarkers();
    pending_plan_json_.clear();
  }
}

void LlmPanel::onExecutePending()
{
  if (pending_plan_json_.isEmpty()) {
    return;
  }
  if (!exec_client_ || !exec_client_->action_server_is_ready()) {
    chat_view_->append(
      "<span style='color:#C62828'>"
      "[/llm/execute_plan not available]"
      "</span>");
    return;
  }

  LlmExecutePlan::Goal goal;
  goal.plan_json = pending_plan_json_.toStdString();

  rclcpp_action::Client<LlmExecutePlan>::SendGoalOptions opts;
  opts.feedback_callback =
    [this](ExecGoalH::SharedPtr,
           const std::shared_ptr<const LlmExecutePlan::Feedback> fb)
    {
      Q_EMIT execStageReceived(
        QString::fromStdString(fb->stage),
        QString::fromStdString(fb->detail));
    };
  opts.result_callback =
    [this](const ExecGoalH::WrappedResult & wrapped)
    {
      const bool transport_ok =
        (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED);
      const bool ok = transport_ok && wrapped.result && wrapped.result->success;
      const QString info = wrapped.result
        ? QString::fromStdString(wrapped.result->info)
        : QString("transport error");
      Q_EMIT execFinished(ok, info);
    };

  chat_view_->append(
    "<i style='color:#1976D2'>[executing previewed plan...]</i>");
  execute_button_->setEnabled(false);
  cancel_button_->setEnabled(false);
  exec_client_->async_send_goal(goal, opts);
}

void LlmPanel::onCancelPending()
{
  pending_plan_json_.clear();
  plan_tree_view_->clear();
  plan_tree_view_->setVisible(false);
  execute_button_->setEnabled(false);
  cancel_button_->setEnabled(false);
  clearPreviewMarkers();
  chat_view_->append("<i style='color:#888'>[preview cancelled]</i>");
}

void LlmPanel::onExecStage(QString stage, QString detail)
{
  if (stage == "error") {
    chat_view_->append(QString("<span style='color:#C62828'>[exec %1]</span>")
      .arg(detail.toHtmlEscaped()));
  }
}

void LlmPanel::onExecFinished(bool success, QString info)
{
  if (success) {
    chat_view_->append(
      QString::fromUtf8("<span style='color:#388E3C'>✓ plan executed</span>"));
  } else {
    chat_view_->append(
      QString("<span style='color:#C62828'>[exec %1]</span>")
        .arg(info.isEmpty() ? "failed" : info.toHtmlEscaped()));
  }
  pending_plan_json_.clear();
  plan_tree_view_->clear();
  plan_tree_view_->setVisible(false);
  clearPreviewMarkers();
}


// ===========================================================================
// Plan tree rendering — mirrors _describe_plan in user_chat_node
// ===========================================================================

namespace
{

void addPlanNode(QTreeWidgetItem * parent, const YAML::Node & n)
{
  if (!n.IsMap() || !n["type"]) {
    return;
  }
  const auto t = n["type"].as<std::string>("");
  QString info;
  QTreeWidgetItem * item = new QTreeWidgetItem(parent);

  if (t == "sequence" || t == "parallel") {
    const auto steps = n["steps"];
    item->setText(0, QString::fromStdString(t));
    item->setText(1, QString("%1 steps").arg(steps ? steps.size() : 0));
    if (steps) {
      for (const auto & child : steps) {
        addPlanNode(item, child);
      }
    }
  } else if (t == "mapf") {
    const auto ids = n["robot_ids"];
    const auto goals = n["goals"];
    const auto reason = n["reason"].as<std::string>("");
    item->setText(0, "mapf");
    item->setText(1, QString("%1 robots — %2")
      .arg(ids ? ids.size() : 0)
      .arg(QString::fromStdString(reason)));
    if (ids && goals) {
      for (size_t i = 0; i < ids.size() && i < goals.size(); ++i) {
        auto * leaf = new QTreeWidgetItem(item);
        leaf->setText(0, QString("robot_%1").arg(ids[i].as<int>()));
        if (goals[i].size() >= 2) {
          leaf->setText(1, fmtCoord(
            goals[i][0].as<double>(),
            goals[i][1].as<double>()));
        }
      }
    }
  } else if (t == "formation") {
    item->setText(0, "formation");
    item->setText(1, QString("id=%1 leader=%2")
      .arg(QString::fromStdString(n["formation_id"].as<std::string>("")))
      .arg(QString::fromStdString(n["leader_ns"].as<std::string>(""))));
  } else if (t == "idle") {
    item->setText(0, "idle");
    item->setText(1, "stop all");
  } else {
    item->setText(0, QString::fromStdString(t));
  }
  item->setExpanded(true);
}

}  // namespace

void LlmPanel::renderPlanTree(const QString & plan_json)
{
  plan_tree_view_->clear();
  if (plan_json.isEmpty()) {
    return;
  }
  try {
    YAML::Node root = YAML::Load(plan_json.toStdString());
    if (root["plan"]) {
      root = root["plan"];
    }
    QTreeWidgetItem * top = new QTreeWidgetItem(plan_tree_view_);
    top->setText(0, "plan");
    top->setText(1, "");
    addPlanNode(top, root);
    top->setExpanded(true);
  } catch (const std::exception & exc) {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(),
        "Plan JSON parse failed: %s", exc.what());
    }
  }
}


// ===========================================================================
// Events tab (Phase 3)
// ===========================================================================

void LlmPanel::onEventReceived(qint64 stamp_ms, int channel,
                               QString trigger, QString output, QString reason)
{
  // Cap the buffer.
  while (events_table_->rowCount() >= kMaxEvents) {
    events_table_->removeRow(0);
  }

  const int row = events_table_->rowCount();
  events_table_->insertRow(row);

  events_table_->setItem(row, 0, new QTableWidgetItem(fmtTime(stamp_ms)));
  auto * ch_item = new QTableWidgetItem(channelName(channel));
  ch_item->setForeground(QColor(channelColor(channel)));
  ch_item->setData(Qt::UserRole, channel);
  events_table_->setItem(row, 1, ch_item);
  events_table_->setItem(row, 2, new QTableWidgetItem(trigger));
  events_table_->setItem(row, 3, new QTableWidgetItem(output));
  events_table_->setItem(row, 4, new QTableWidgetItem(reason));

  // Apply current filter to the new row.
  const bool visible =
    (channel == 1 && event_filter_decision_->isChecked()) ||
    (channel == 2 && event_filter_observer_->isChecked()) ||
    (channel == 3 && event_filter_user_->isChecked());
  events_table_->setRowHidden(row, !visible);
  events_table_->scrollToBottom();
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
// Markers — goals (MVP), preview, TF arrows, formation polygon
// ===========================================================================

void LlmPanel::publishGoalMarkers(const BTState & msg)
{
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

void LlmPanel::publishArrowMarkers()
{
  if (!tf_buffer_ || !marker_pub_) {
    return;
  }

  std::string                            mode;
  std::vector<uint32_t>                  ids;
  std::vector<geometry_msgs::msg::Point> goals;
  {
    std::lock_guard<std::mutex> lk(cached_state_mutex_);
    mode  = cached_mode_;
    ids   = cached_robot_ids_;
    goals = cached_goals_;
  }
  if (mode != "mapf" || ids.empty() || ids.size() != goals.size()) {
    return;
  }

  visualization_msgs::msg::MarkerArray ma;
  ma.markers.reserve(ids.size());

  const auto stamp    = node_->now();
  const auto lifetime = rclcpp::Duration::from_seconds(0.6);

  // Update the per-robot dist column too while we're here.
  for (size_t i = 0; i < ids.size(); ++i) {
    const auto frame = "robot_" + std::to_string(ids[i]) + "/base_link";
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        "map", frame, tf2::TimePointZero, tf2::durationFromSec(0.0));
    } catch (const tf2::TransformException &) {
      continue;
    }

    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp    = stamp;
    arrow.ns              = "llm_panel/arrows";
    arrow.id              = static_cast<int>(ids[i]);
    arrow.type            = visualization_msgs::msg::Marker::ARROW;
    arrow.action          = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05;   // shaft diameter
    arrow.scale.y = 0.10;   // head diameter
    arrow.scale.z = 0.15;   // head length
    auto col = robotColor(ids[i]);
    col.a    = 1.0f;
    arrow.color = col;
    arrow.lifetime = lifetime;

    geometry_msgs::msg::Point start;
    start.x = tf.transform.translation.x;
    start.y = tf.transform.translation.y;
    start.z = 0.1;
    arrow.points.push_back(start);
    geometry_msgs::msg::Point end = goals[i];
    end.z = 0.1;
    arrow.points.push_back(end);
    ma.markers.push_back(arrow);

    // Distance — write into the table on the GUI thread (we already are).
    const double dx = goals[i].x - start.x;
    const double dy = goals[i].y - start.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (mapf_robot_table_->rowCount() > static_cast<int>(i)) {
      auto * dist_item = mapf_robot_table_->item(static_cast<int>(i), 2);
      if (!dist_item) {
        dist_item = new QTableWidgetItem();
        mapf_robot_table_->setItem(static_cast<int>(i), 2, dist_item);
      }
      dist_item->setText(QString::number(dist, 'f', 2));
    }
  }

  if (!ma.markers.empty()) {
    marker_pub_->publish(ma);
  }
}

void LlmPanel::publishFormationPolygon()
{
  if (!tf_buffer_ || !marker_pub_) {
    return;
  }
  std::string mode;
  std::string leader_ns;
  {
    std::lock_guard<std::mutex> lk(cached_state_mutex_);
    mode      = cached_mode_;
    leader_ns = cached_leader_ns_;
  }
  if (mode != "formation" || leader_ns.empty()) {
    return;
  }

  // BTState doesn't carry follower_ns directly here; for MVP visualisation we
  // walk the cached robot_ids — the BT publishes them for active formations.
  std::vector<uint32_t> ids;
  {
    std::lock_guard<std::mutex> lk(cached_state_mutex_);
    ids = cached_robot_ids_;
  }
  if (ids.size() < 2) {
    return;
  }

  visualization_msgs::msg::Marker line;
  line.header.frame_id = "map";
  line.header.stamp    = node_->now();
  line.ns              = "llm_panel/formation";
  line.id              = 0;
  line.type            = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action          = visualization_msgs::msg::Marker::ADD;
  line.scale.x         = 0.05;
  line.color.r = 0.48f; line.color.g = 0.12f; line.color.b = 0.64f;
  line.color.a = 1.0f;
  line.lifetime        = rclcpp::Duration::from_seconds(0.6);

  for (uint32_t id : ids) {
    const auto frame = "robot_" + std::to_string(id) + "/base_link";
    try {
      const auto tf = tf_buffer_->lookupTransform(
        "map", frame, tf2::TimePointZero, tf2::durationFromSec(0.0));
      geometry_msgs::msg::Point p;
      p.x = tf.transform.translation.x;
      p.y = tf.transform.translation.y;
      p.z = 0.1;
      line.points.push_back(p);
    } catch (const tf2::TransformException &) {
      // Skip missing TFs; partial polygon is better than nothing.
    }
  }

  if (line.points.size() < 2) {
    return;
  }
  // Close the loop.
  line.points.push_back(line.points.front());

  visualization_msgs::msg::MarkerArray ma;
  ma.markers.push_back(line);
  marker_pub_->publish(ma);
}

void LlmPanel::publishPreviewMarkers()
{
  if (pending_plan_json_.isEmpty() || !marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray ma;
  const auto stamp    = node_->now();
  const auto lifetime = rclcpp::Duration::from_seconds(0.0);   // 0 = forever

  int marker_id = 0;
  std::function<void(const YAML::Node &)> walk = [&](const YAML::Node & n)
  {
    if (!n.IsMap() || !n["type"]) return;
    const auto t = n["type"].as<std::string>("");
    if (t == "sequence" || t == "parallel") {
      if (n["steps"]) {
        for (const auto & c : n["steps"]) walk(c);
      }
    } else if (t == "mapf" && n["goals"]) {
      const auto ids = n["robot_ids"];
      for (size_t i = 0; i < n["goals"].size(); ++i) {
        const auto & g = n["goals"][i];
        if (g.size() < 2) continue;

        visualization_msgs::msg::Marker s;
        s.header.frame_id = "map";
        s.header.stamp    = stamp;
        s.ns              = "llm_panel/preview";
        s.id              = marker_id++;
        s.type            = visualization_msgs::msg::Marker::SPHERE;
        s.action          = visualization_msgs::msg::Marker::ADD;
        s.pose.position.x = g[0].as<double>();
        s.pose.position.y = g[1].as<double>();
        s.pose.position.z = 0.25;
        s.pose.orientation.w = 1.0;
        s.scale.x = 0.45; s.scale.y = 0.45; s.scale.z = 0.45;
        const uint32_t rid =
          (ids && i < ids.size()) ? ids[i].as<uint32_t>(0) : 0;
        s.color    = robotColor(rid);
        s.color.a  = 0.4f;     // semi-transparent — clearly a preview
        s.lifetime = lifetime;
        ma.markers.push_back(s);
      }
    }
  };

  try {
    YAML::Node root = YAML::Load(pending_plan_json_.toStdString());
    if (root["plan"]) root = root["plan"];
    walk(root);
  } catch (...) {
    return;
  }

  if (!ma.markers.empty()) {
    marker_pub_->publish(ma);
  }
}

void LlmPanel::clearPreviewMarkers()
{
  if (!marker_pub_) return;
  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker del;
  del.header.frame_id = "map";
  del.header.stamp    = node_ ? node_->now() : rclcpp::Time(0);
  del.ns              = "llm_panel/preview";
  del.action          = visualization_msgs::msg::Marker::DELETEALL;
  ma.markers.push_back(del);
  marker_pub_->publish(ma);
}

void LlmPanel::onMarkerTick()
{
  if (mapf_active_spark_  && mapf_active_spark_->dirty()) {
    mapf_active_spark_->update();
    mapf_active_spark_->clearDirty();
  }
  if (mapf_arrived_spark_ && mapf_arrived_spark_->dirty()) {
    mapf_arrived_spark_->update();
    mapf_arrived_spark_->clearDirty();
  }
  publishArrowMarkers();
  publishFormationPolygon();
}


// ===========================================================================
// Info tab — graph + remote parameter discovery
// ===========================================================================

namespace
{

// `get_node_names()` returns fully qualified names like "/llm_chat_server".
std::string stripSlash(const std::string & n)
{
  return (!n.empty() && n.front() == '/') ? n.substr(1) : n;
}

// Best-effort detection of LLM backend kind from an endpoint URL.
QString endpointKind(const std::string & ep)
{
  if (ep.empty())                                return "mock";
  if (ep.find("11434")     != std::string::npos) return "ollama";
  if (ep.find("anthropic") != std::string::npos) return "anthropic";
  if (ep.find("openai")    != std::string::npos) return "openai";
  if (ep.find("v1/chat")   != std::string::npos) return "openai-compatible";
  return QString::fromStdString(ep);
}

}  // namespace

void LlmPanel::onInfoRefresh()
{
  // Cheap, GUI-thread refresh. Synchronous calls below are
  // graph-introspection only; remote params are dispatched async.
  if (!node_) return;
  refreshSystemInfo();
}

void LlmPanel::refreshSystemInfo()
{
  // ---- Pass 1 — synchronous graph introspection -------------------------
  std::vector<std::string> all_names;
  try {
    all_names = node_->get_node_names();
  } catch (const std::exception & exc) {
    RCLCPP_DEBUG(node_->get_logger(),
      "get_node_names failed: %s", exc.what());
    return;
  }

  std::unordered_set<std::string> nodes;
  nodes.reserve(all_names.size());
  for (const auto & n : all_names) nodes.insert(stripSlash(n));

  auto has = [&](const std::string & needle) -> bool {
    return nodes.count(needle) > 0;
  };

  const bool pbs_up      = has("mapf_planner");
  const bool lns_up      = has("mapf_lns2");
  const bool form_mgr    = has("formation_manager");
  const bool form_mon    = has("formation_monitor");
  const bool obs_up      = has("llm_passive_observer");
  const bool dec_up      = has("llm_decision_server");
  const bool chat_up     = has("llm_chat_server");
  const bool exec_up     = has("llm_execute_server");
  const bool bt_up       = has("test_bt_runner");
  const bool stage_up    = has("stage_ros2");
  const bool map_srv_up  = has("map_server") || has("zone_map_server");

  std::string planner_label;
  if (pbs_up && lns_up)  planner_label = "PBS + LNS2 (BOTH UP — misconfig)";
  else if (pbs_up)       planner_label = "PBS (mapf_planner)";
  else if (lns_up)       planner_label = "LNS2 (mapf_lns2)";
  else                   planner_label = "(none)";

  // Robot count: prefer last-seen BT state over yaml.
  std::string robots_str;
  {
    std::lock_guard<std::mutex> lk(cached_state_mutex_);
    if (!cached_robot_ids_.empty()) {
      robots_str = std::to_string(cached_robot_ids_.size()) + " in BT";
    }
  }
  if (robots_str.empty()) {
    int yaml_robots = 0;
    for (const auto & g : groups_) yaml_robots += static_cast<int>(g.ids.size());
    robots_str = (yaml_robots > 0)
        ? (std::to_string(yaml_robots) + " from map yaml")
        : std::string("(unknown)");
  }

  // RMW + domain id from the running context / env.
  const char * rmw_id = rmw_get_implementation_identifier();
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  const char * cyc    = std::getenv("CYCLONEDDS_URI");

  // Compose the static portion of info_kv_. Dynamic parts (LLM model,
  // endpoint, llm_mode) are filled in by remote-parameter callbacks below
  // and merged into this same map; we preserve any previous values.
  {
    std::lock_guard<std::mutex> lk(info_mutex_);

    auto upsert = [&](const std::string & k, const std::string & v) {
      for (auto & kv : info_kv_) {
        if (kv.first == k) { kv.second = v; return; }
      }
      info_kv_.emplace_back(k, v);
    };

    upsert("Scenario / Map",      map_name_);
    upsert("Map bounds",
           "x [" + std::to_string(static_cast<int>(bounds_xmin_)) + ", "
                 + std::to_string(static_cast<int>(bounds_xmax_)) + "]  "
           "y [" + std::to_string(static_cast<int>(bounds_ymin_)) + ", "
                 + std::to_string(static_cast<int>(bounds_ymax_)) + "]");
    upsert("Planner",             planner_label);
    upsert("Robots",              robots_str);
    upsert("Stage simulator",     stage_up   ? "UP"  : "DOWN");
    upsert("Map server",          map_srv_up ? "UP"  : "DOWN");
    upsert("BT runner",           bt_up      ? "UP"  : "DOWN");
    upsert("Formation manager",   form_mgr   ? "UP"  : "DOWN");
    upsert("Formation monitor",   form_mon   ? "UP"  : "DOWN");
    upsert("LLM chat server",     chat_up    ? "UP"  : "DOWN");
    upsert("LLM decision server", dec_up     ? "UP"  : "DOWN");
    upsert("LLM execute server",  exec_up    ? "UP"  : "DOWN");
    upsert("Passive observer",    obs_up     ? "ON (channel 2)" : "OFF");
    upsert("RMW",                 rmw_id ? rmw_id : "(unknown)");
    upsert("ROS_DOMAIN_ID",       domain ? domain : "0 (default)");
    upsert("CYCLONEDDS_URI",      cyc    ? cyc    : "(unset)");

    info_last_refresh_ms_ =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
  }
  Q_EMIT systemInfoChanged();

  // ---- Pass 2 — async remote parameter fetches --------------------------
  if (chat_up) {
    requestRemoteParam("llm_chat_server",
      {"llm_model", "llm_endpoint", "map_name", "llm_temperature"},
      "Chat");
  }
  if (dec_up) {
    requestRemoteParam("llm_decision_server",
      {"llm_mode", "llm_model", "llm_endpoint"},
      "Decision");
  }
  if (obs_up) {
    requestRemoteParam("llm_passive_observer",
      {"llm_mode", "llm_model", "llm_endpoint", "enabled", "cooldown_sec"},
      "Observer");
  }
  if (pbs_up) {
    requestRemoteParam("mapf_planner",
      {"num_robots", "pbs_resolution", "max_speed", "inflation_radius"},
      "PBS");
  }
  if (lns_up) {
    requestRemoteParam("mapf_lns2",
      {"num_robots", "max_speed"},
      "LNS2");
  }
}

void LlmPanel::requestRemoteParam(
  const std::string & node_name,
  const std::vector<std::string> & param_names,
  const std::string & key_prefix)
{
  if (!node_) return;

  auto it = param_clients_.find(node_name);
  if (it == param_clients_.end()) {
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, node_name);
    it = param_clients_.emplace(node_name, client).first;
  }
  auto & client = it->second;

  // Skip this round if the parameter service isn't there yet — avoids
  // queueing futures that will never resolve on the executor.
  if (!client->service_is_ready()) return;

  client->get_parameters(
    param_names,
    [this, key_prefix](
      std::shared_future<std::vector<rclcpp::Parameter>> fut)
    {
      std::vector<rclcpp::Parameter> params;
      try {
        params = fut.get();
      } catch (const std::exception &) {
        return;
      }

      bool changed = false;
      {
        std::lock_guard<std::mutex> lk(info_mutex_);
        for (const auto & p : params) {
          if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) continue;

          std::string val;
          if (p.get_name() == "llm_endpoint") {
            val = endpointKind(p.as_string()).toStdString();
          } else {
            val = p.value_to_string();
          }
          const std::string key = key_prefix + " · " + p.get_name();

          bool found = false;
          for (auto & kv : info_kv_) {
            if (kv.first == key) {
              if (kv.second != val) { kv.second = val; changed = true; }
              found = true;
              break;
            }
          }
          if (!found) {
            info_kv_.emplace_back(key, val);
            changed = true;
          }
        }
      }
      if (changed) Q_EMIT systemInfoChanged();
    });
}

void LlmPanel::onSystemInfoChanged()
{
  if (!info_table_) return;

  std::vector<std::pair<std::string, std::string>> snapshot;
  std::int64_t stamp_ms = 0;
  {
    std::lock_guard<std::mutex> lk(info_mutex_);
    snapshot = info_kv_;
    stamp_ms = info_last_refresh_ms_;
  }

  info_table_->setRowCount(static_cast<int>(snapshot.size()));
  for (int i = 0; i < static_cast<int>(snapshot.size()); ++i) {
    auto * k = new QTableWidgetItem(QString::fromStdString(snapshot[i].first));
    QFont f = k->font();
    f.setBold(true);
    k->setFont(f);
    info_table_->setItem(i, 0, k);
    info_table_->setItem(i, 1,
      new QTableWidgetItem(QString::fromStdString(snapshot[i].second)));
  }
  if (info_updated_lbl_ && stamp_ms > 0) {
    info_updated_lbl_->setText("last refresh: " + fmtTime(stamp_ms));
  }
}


// ===========================================================================
// Persistence (Phase 8)
// ===========================================================================

void LlmPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString map_name;
  if (config.mapGetString("map_name", &map_name) && !map_name.isEmpty()) {
    map_name_ = map_name.toStdString();
    loadMapYaml();
  }

  bool preview = false;
  if (config.mapGetBool("preview_mode", &preview)) {
    preview_mode_checkbox_->setChecked(preview);
    preview_mode_ = preview;
  }

  int tab_idx = 0;
  if (config.mapGetInt("tab_index", &tab_idx)) {
    if (tab_idx >= 0 && tab_idx < tabs_->count()) {
      tabs_->setCurrentIndex(tab_idx);
    }
  }
}

void LlmPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("map_name",     QString::fromStdString(map_name_));
  config.mapSetValue("preview_mode", preview_mode_);
  config.mapSetValue("tab_index",    tabs_ ? tabs_->currentIndex() : 0);
}

}  // namespace iros_llm_rviz_panel

PLUGINLIB_EXPORT_CLASS(iros_llm_rviz_panel::LlmPanel, rviz_common::Panel)
