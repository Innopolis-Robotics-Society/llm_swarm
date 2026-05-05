// Copyright 2026 — iros_llm_swarm contributors. Apache-2.0.

#include "iros_llm_rviz_panel/sparkline.hpp"

#include <algorithm>
#include <limits>

#include <QPaintEvent>
#include <QPainter>
#include <QPen>
#include <QPolygonF>

namespace iros_llm_rviz_panel
{

Sparkline::Sparkline(QWidget * parent)
: QWidget(parent)
{
  setAttribute(Qt::WA_OpaquePaintEvent, false);
}

void Sparkline::push(double value)
{
  values_[head_] = value;
  head_ = (head_ + 1) % kCapacity;
  if (count_ < kCapacity) {
    ++count_;
  }
  dirty_ = true;
}

void Sparkline::clear()
{
  head_  = 0;
  count_ = 0;
  dirty_ = true;
}

void Sparkline::setColor(const QColor & c)
{
  color_ = c;
  dirty_ = true;
}

void Sparkline::paintEvent(QPaintEvent * /*ev*/)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  const QRectF r = rect().adjusted(2, 2, -2, -2);
  // Subtle background so the widget reads as a chart even when empty.
  p.fillRect(rect(), QColor(0x2B, 0x2B, 0x2B, 40));

  if (count_ < 2) {
    return;
  }

  // Snapshot the buffer in chronological order.
  std::array<double, kCapacity> ordered{};
  for (std::size_t i = 0; i < count_; ++i) {
    const std::size_t idx =
      (head_ + kCapacity - count_ + i) % kCapacity;
    ordered[i] = values_[idx];
  }

  double lo = std::numeric_limits<double>::infinity();
  double hi = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < count_; ++i) {
    lo = std::min(lo, ordered[i]);
    hi = std::max(hi, ordered[i]);
  }
  if (hi - lo < 1e-9) {
    // Flat line — draw centred so the widget doesn't look broken.
    hi = lo + 1.0;
  }

  QPolygonF poly;
  poly.reserve(static_cast<int>(count_));
  const double dx = (count_ > 1) ? r.width() / static_cast<double>(count_ - 1) : 0.0;
  for (std::size_t i = 0; i < count_; ++i) {
    const double x = r.left() + dx * static_cast<double>(i);
    const double t = (ordered[i] - lo) / (hi - lo);
    const double y = r.bottom() - t * r.height();
    poly << QPointF(x, y);
  }

  QPen pen(color_);
  pen.setWidthF(1.6);
  pen.setCosmetic(true);
  p.setPen(pen);
  p.drawPolyline(poly);

  // Last-sample dot, slightly more visible.
  const QPointF last = poly.back();
  p.setBrush(color_);
  p.setPen(Qt::NoPen);
  p.drawEllipse(last, 2.2, 2.2);
}

}  // namespace iros_llm_rviz_panel
