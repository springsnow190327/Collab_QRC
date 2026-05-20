/*
Copyright 2025 NVIDIA CORPORATION

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include <gtest/gtest.h>
#include "glog/logging.h"

TEST(LoggingTest, CheckEQ) { EXPECT_DEATH(CHECK_EQ(1, 2) << "test", ".*"); }

TEST(LoggingTest, CheckNE) { EXPECT_DEATH(CHECK_NE(1, 1) << "test", ".*"); }

TEST(LoggingTest, CheckGT) { EXPECT_DEATH(CHECK_GT(1, 2) << "test", ".*"); }

TEST(LoggingTest, CheckGE) { EXPECT_DEATH(CHECK_GE(1, 2) << "test", ".*"); }

TEST(LoggingTest, CheckLT) { EXPECT_DEATH(CHECK_LT(2, 1) << "test", ".*"); }

TEST(LoggingTest, CheckLE) { EXPECT_DEATH(CHECK_LE(2, 1) << "test", ".*"); }

TEST(LoggingTest, CheckNotNULL) { EXPECT_DEATH(CHECK_NOTNULL(nullptr), ".*"); }

TEST(LoggingTest, CheckNear) { EXPECT_DEATH(CHECK_NEAR(1, 2, 0.1), ".*"); }

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
