# TaskStates.msg
#
# Full snapshot of every declared task, published by task_manager_node on
# /tasks/state at its poll rate. TaskState alone is per-task; the aggregate is
# what makes mission progress recordable — a bag of this topic is the ground
# truth for end-to-end mission scoring, which previously existed only as the
# reply to the /tasks/list service and so could not be captured offline.
#
# Mirrors the FormationsStatus pattern: header + full array, republished every
# tick rather than on change, so a recording started mid-run still sees the
# complete state.

std_msgs/Header header

TaskState[] states
