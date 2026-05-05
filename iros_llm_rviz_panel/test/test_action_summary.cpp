// Copyright 2026 — iros_llm_swarm contributors. Apache-2.0.

#include <gtest/gtest.h>

#include "iros_llm_rviz_panel/action_summary.hpp"

using iros_llm_rviz_panel::ActionSummary;
using iros_llm_rviz_panel::parseActionSummary;

TEST(ActionSummary, WellFormed)
{
  const auto s = parseActionSummary(
    "[t=1200ms status=executing arrived=5 active=15 stall=0 replans=0]");
  ASSERT_TRUE(s.elapsed_ms.has_value());
  EXPECT_EQ(*s.elapsed_ms, 1200);
  ASSERT_TRUE(s.status.has_value());
  EXPECT_EQ(*s.status, "executing");
  EXPECT_EQ(s.arrived.value_or(-1), 5);
  EXPECT_EQ(s.active.value_or(-1), 15);
  EXPECT_EQ(s.stall.value_or(-1), 0);
  EXPECT_EQ(s.replans.value_or(-1), 0);
  EXPECT_TRUE(s.event_tail.empty());
}

TEST(ActionSummary, InfoTail)
{
  const auto s = parseActionSummary(
    "[t=2400ms status=executing arrived=8 active=12 stall=0 replans=0] "
    "INFO: progressing normally");
  EXPECT_EQ(s.elapsed_ms.value_or(-1), 2400);
  EXPECT_EQ(s.event_tail, "INFO: progressing normally");
}

TEST(ActionSummary, WarnTail)
{
  const auto s = parseActionSummary(
    "[t=3600ms status=executing arrived=8 active=12 stall=1 replans=0] "
    "WARN: robot_3 stalled for 5s at (12.4, 8.1)");
  EXPECT_EQ(s.stall.value_or(-1), 1);
  EXPECT_EQ(s.event_tail, "WARN: robot_3 stalled for 5s at (12.4, 8.1)");
}

TEST(ActionSummary, MissingFields)
{
  const auto s = parseActionSummary("[status=validating]");
  EXPECT_FALSE(s.elapsed_ms.has_value());
  ASSERT_TRUE(s.status.has_value());
  EXPECT_EQ(*s.status, "validating");
  EXPECT_FALSE(s.arrived.has_value());
  EXPECT_FALSE(s.active.has_value());
  EXPECT_FALSE(s.stall.has_value());
  EXPECT_FALSE(s.replans.has_value());
}

TEST(ActionSummary, ReorderedFields)
{
  const auto s = parseActionSummary(
    "[active=11 status=executing replans=2 arrived=9 t=5000ms stall=0]");
  EXPECT_EQ(s.elapsed_ms.value_or(-1), 5000);
  EXPECT_EQ(s.active.value_or(-1), 11);
  EXPECT_EQ(s.replans.value_or(-1), 2);
  EXPECT_EQ(s.arrived.value_or(-1), 9);
}

TEST(ActionSummary, EmptyString)
{
  const auto s = parseActionSummary("");
  EXPECT_FALSE(s.elapsed_ms.has_value());
  EXPECT_FALSE(s.status.has_value());
  EXPECT_FALSE(s.arrived.has_value());
  EXPECT_TRUE(s.event_tail.empty());
}

TEST(ActionSummary, NoBrackets)
{
  const auto s = parseActionSummary("status=executing arrived=5");
  EXPECT_FALSE(s.elapsed_ms.has_value());
  EXPECT_FALSE(s.status.has_value());
}

TEST(ActionSummary, MalformedNumbers)
{
  const auto s = parseActionSummary(
    "[t=abc status=executing arrived=NaN active=5]");
  EXPECT_FALSE(s.elapsed_ms.has_value());
  EXPECT_EQ(s.status.value_or(""), "executing");
  EXPECT_FALSE(s.arrived.has_value());
  EXPECT_EQ(s.active.value_or(-1), 5);
}

TEST(ActionSummary, ExtraWhitespace)
{
  const auto s = parseActionSummary(
    "[  t=100ms   status=executing    arrived=1  ]");
  EXPECT_EQ(s.elapsed_ms.value_or(-1), 100);
  EXPECT_EQ(s.status.value_or(""), "executing");
  EXPECT_EQ(s.arrived.value_or(-1), 1);
}

TEST(ActionSummary, UnclosedBracket)
{
  const auto s = parseActionSummary(
    "[t=100ms status=executing arrived=1");
  EXPECT_FALSE(s.elapsed_ms.has_value());
  EXPECT_FALSE(s.status.has_value());
}

TEST(ActionSummary, UnknownKeyIgnored)
{
  const auto s = parseActionSummary(
    "[t=100ms future_field=42 status=executing]");
  EXPECT_EQ(s.elapsed_ms.value_or(-1), 100);
  EXPECT_EQ(s.status.value_or(""), "executing");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
