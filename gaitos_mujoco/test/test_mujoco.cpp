//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "gaitos_common/Math.hpp"
#include "gaitos_system/Gaitos.hpp"
#include "gaitos_system/testing.hpp"

using namespace gaitos;

static auto * const ros_env = ::testing::AddGlobalTestEnvironment(new gaitos::testutil::RosEnvironment);

namespace
{

/// The MuJoCo assets installed by gaitos_description (FetchContent, GAITOS_FETCH_ASSETS)
bool installed(const std::string & folder)
{
  return std::filesystem::exists(
    ament_index_cpp::get_package_share_path("gaitos_description") / "robots" / folder);
}

bool assets() {return installed("go2_description/mjcf/assets");}
bool r1_assets() {return installed("r1_description/mjcf/meshes");}
bool nao_assets() {return installed("nao_description/mjcf/meshes");}

void request(Gaitos & g, const std::string & mode)
{
  auto f = g.request_mode(mode);
  g.step();
  const auto r = f.get();
  ASSERT_TRUE(r.success) << r.message;
}

void run(Gaitos & g, double seconds, const std::function<void()> & each = nullptr)
{
  const int n = static_cast<int>(seconds * g.frequency());
  for (int i = 0; i < n; ++i) {
    if (each) {
      each();
    }
    ASSERT_TRUE(g.step());
  }
}

Odometry truth(const Gaitos & g)
{
  return g.state().external_odometry.at("ground_truth").odometry;
}
}  // namespace

TEST(Mujoco, Go2StandsUpWalksAndLiesDown)
{
  if (!assets()) {
    GTEST_SKIP() << "gaitos_description was built with GAITOS_FETCH_ASSETS=OFF";
  }
  Gaitos g;
  g.configure(testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "go2", {"gaitos_mujoco/params/go2_mujoco.yaml"})));
  ASSERT_TRUE(g.start());
  run(g, 0.5);
  EXPECT_LT(truth(g).position.z(), 0.15);   // lying

  request(g, "stand_up");
  run(g, 2.5);
  EXPECT_GT(truth(g).position.z(), 0.28);
  EXPECT_LT(tilt_angle(truth(g).orientation), 0.1);

  // Estimator vs ground truth while standing
  EXPECT_NEAR(g.state().base.position.z(), truth(g).position.z(), 0.03);

  request(g, "balance_stand");
  run(g, 1.0);
  const double z0 = truth(g).position.z();
  g.set_pose_goal({0.0, 0.0, -0.06}, rpy_to_quaternion({0.0, 0.15, 0.0}));
  run(g, 1.5);
  EXPECT_NEAR(truth(g).position.z(), z0 - 0.06, 0.025);
  EXPECT_NEAR(quaternion_to_rpy(truth(g).orientation).y(), 0.15, 0.06);
  g.set_pose_goal(Eigen::Vector3d(0.0, 0.0, 0.06), rpy_to_quaternion({0.0, -0.15, 0.0}));
  run(g, 1.5);

  // Walk forward
  const Eigen::Vector3d start = truth(g).position;
  run(g, 4.0, [&g]() {g.set_twist({0.4, 0.0, 0.0}, Eigen::Vector3d::Zero());});
  EXPECT_EQ(g.state().mode, "locomotion");
  const Eigen::Vector3d moved = truth(g).position - start;
  EXPECT_GT(moved.x(), 0.6) << "moved " << moved.transpose();
  EXPECT_LT(std::abs(moved.y()), 0.4);
  EXPECT_LT(tilt_angle(truth(g).orientation), 0.3);
  // Odometry drift of the Kalman filter (leg kinematics + IMU)
  const Eigen::Vector3d est = g.state().base.position;
  EXPECT_LT((est - truth(g).position).head<2>().norm(), 0.25 * moved.norm() + 0.05);
  run(g, 2.0);   // stops

  // Turn in place
  const double yaw0 = yaw_of(truth(g).orientation);
  run(g, 3.0, [&g]() {g.set_twist(Eigen::Vector3d::Zero(), {0.0, 0.0, 0.6});});
  EXPECT_GT(wrap_angle(yaw_of(truth(g).orientation) - yaw0), 0.6);
  run(g, 2.0);

  // Walk to a goal 0.5 m ahead and 0.3 m to the left (relative to the body)
  const Eigen::Vector3d p0 = truth(g).position;
  const double yaw1 = yaw_of(truth(g).orientation);
  g.set_pose_goal({0.5, 0.3, 0.0}, Eigen::Quaterniond::Identity());
  run(g, 6.0);
  const Eigen::Vector3d rel = Eigen::AngleAxisd(-yaw1, Eigen::Vector3d::UnitZ()) * (truth(g).position - p0);
  EXPECT_NEAR(rel.x(), 0.5, 0.15) << rel.transpose();
  EXPECT_NEAR(rel.y(), 0.3, 0.15) << rel.transpose();

  request(g, "balance_stand");
  run(g, 1.0);
  request(g, "stand_down");
  run(g, 2.5);
  EXPECT_LT(truth(g).position.z(), 0.15);
  request(g, "damp");
  run(g, 0.5);
}

TEST(Mujoco, Go2SitAndFallDetection)
{
  if (!assets()) {
    GTEST_SKIP() << "gaitos_description was built with GAITOS_FETCH_ASSETS=OFF";
  }
  Gaitos g;
  g.configure(testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "go2", {"gaitos_mujoco/params/go2_mujoco.yaml"})));
  ASSERT_TRUE(g.start());
  request(g, "stand_up");
  run(g, 2.5);
  request(g, "sit");
  run(g, 2.5);
  const Eigen::Vector3d rpy = quaternion_to_rpy(truth(g).orientation);
  EXPECT_LT(rpy.y(), -0.2) << "sitting: the body pitches up";
  EXPECT_LT(tilt_angle(truth(g).orientation), 1.0);
  EXPECT_EQ(g.state().mode, "sit");
}

TEST(Mujoco, R1StandsAndSquats)
{
  if (!r1_assets())
  {
    GTEST_SKIP() << "gaitos_description was built with GAITOS_FETCH_ASSETS=OFF";
  }
  Gaitos g;
  g.configure(testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "r1", {"gaitos_mujoco/params/r1_mujoco.yaml"})));
  ASSERT_TRUE(g.start());
  run(g, 2.5);
  EXPECT_EQ(g.state().mode, "stand_up");
  const double z_stand = truth(g).position.z();
  EXPECT_GT(z_stand, 0.6);
  EXPECT_LT(tilt_angle(truth(g).orientation), 0.15);

  request(g, "balance_stand");
  run(g, 1.0);
  g.set_pose_goal({0.0, 0.0, -0.08}, Eigen::Quaterniond::Identity());
  run(g, 2.0);
  EXPECT_NEAR(truth(g).position.z(), z_stand - 0.08, 0.03);
  EXPECT_LT(tilt_angle(truth(g).orientation), 0.15);

  request(g, "squat");
  run(g, 3.0);
  EXPECT_LT(truth(g).position.z(), z_stand - 0.12);
  EXPECT_LT(tilt_angle(truth(g).orientation), 0.3);
}

TEST(Mujoco, NaoStandsFromCrouch)
{
  if (!nao_assets()) {
    GTEST_SKIP() << "gaitos_description was built with GAITOS_FETCH_ASSETS=OFF";
  }
  Gaitos g;
  g.configure(testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "nao", {"gaitos_mujoco/params/nao_mujoco.yaml"})));
  ASSERT_TRUE(g.start());
  run(g, 1.0);
  EXPECT_EQ(g.state().mode, "crouch");
  const double z_crouch = truth(g).position.z();
  EXPECT_NEAR(z_crouch, 0.23, 0.03);

  request(g, "stand_init");
  run(g, 3.0);
  const double z_stand = truth(g).position.z();
  EXPECT_GT(z_stand, z_crouch + 0.07) << "standing raises the torso";
  EXPECT_LT(tilt_angle(truth(g).orientation), 0.25);   // stand_init leans 0.1 rad forward
  EXPECT_LT(std::abs(truth(g).position.x()), 0.03);     // and stays in place

  request(g, "balance_stand");
  run(g, 1.0);
  g.set_pose_goal({0.0, 0.0, -0.02}, Eigen::Quaterniond::Identity());
  run(g, 2.0);
  EXPECT_NEAR(truth(g).position.z(), z_stand - 0.02, 0.01);

  request(g, "crouch");
  run(g, 3.0);
  EXPECT_NEAR(truth(g).position.z(), z_crouch, 0.01);
  EXPECT_LT(tilt_angle(truth(g).orientation), 0.15);
}

TEST(Mujoco, NaoWalks)
{
  if (!nao_assets()) {
    GTEST_SKIP() << "gaitos_description was built with GAITOS_FETCH_ASSETS=OFF";
  }
  Gaitos g;
  g.configure(testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "nao", {"gaitos_mujoco/params/nao_mujoco.yaml"})));
  ASSERT_TRUE(g.start());
  run(g, 0.5);
  request(g, "stand_init");
  run(g, 3.0);
  request(g, "balance_stand");
  run(g, 1.0);
  const std::string walk = std::getenv("GAITOS_TEST_WALK") ? std::getenv("GAITOS_TEST_WALK") : "locomotion";
  request(g, walk);
  const auto print = [&g](const char * what) {
      const auto t = truth(g);
      const Eigen::Vector3d rpy = quaternion_to_rpy(t.orientation);
      if (std::getenv("GAITOS_TEST_VERBOSE")) {
        const auto & c = g.state().contacts;
        std::printf("%-8s t=%5.2f %-15s x=%6.3f y=%6.3f z=%5.3f roll=%5.2f pitch=%5.2f yaw=%5.2f c=%d%d F=%3.0f/%3.0f\n", what,
          g.state().time, g.state().mode.c_str(), t.position.x(), t.position.y(), t.position.z(),
          rpy.x(), rpy.y(), rpy.z(), c.size() > 0 && c.contact[0] ? 1 : 0, c.size() > 1 && c.contact[1] ? 1 : 0,
          c.size() > 0 ? c.force[0] : 0.0, c.size() > 1 ? c.force[1] : 0.0);
      }
    };
  const double x0 = truth(g).position.x();
  // Forward
  const double dt_print = std::getenv("GAITOS_TEST_DT") ? std::atof(std::getenv("GAITOS_TEST_DT")) : 0.5;
  for (int k = 0; k < static_cast<int>(8.0 / dt_print); ++k) {
    const double vx = std::getenv("GAITOS_TEST_VX") ? std::atof(std::getenv("GAITOS_TEST_VX")) : 0.06;
    run(g, dt_print, [&g, vx]() {g.set_twist({vx, 0.0, 0.0}, Eigen::Vector3d::Zero());});
    print("forward");
  }
  EXPECT_EQ(g.state().mode, walk);
  EXPECT_GT(truth(g).position.x() - x0, 0.25) << "walked forward";
  EXPECT_LT(tilt_angle(truth(g).orientation), 0.3);
  // Turn in place
  const double yaw0 = gaitos::yaw_of(truth(g).orientation);
  for (int k = 0; k < 8; ++k) {
    run(g, 0.5, [&g]() {g.set_twist(Eigen::Vector3d::Zero(), {0.0, 0.0, 0.3});});
    print("turn");
  }
  EXPECT_GT(gaitos::wrap_angle(gaitos::yaw_of(truth(g).orientation) - yaw0), 0.5) << "turned left";
  // Stop
  run(g, 3.0);
  print("stop");
  EXPECT_LT(tilt_angle(truth(g).orientation), 0.3);
  EXPECT_GT(truth(g).position.z(), 0.28) << "still standing";
}

TEST(Mujoco, Go2ConvexMpcWalksAndTurns)
{
  if (!assets()) {
    GTEST_SKIP() << "gaitos_description was built with GAITOS_FETCH_ASSETS=OFF";
  }
  Gaitos g;
  g.configure(testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "go2", {"gaitos_mujoco/params/go2_mujoco.yaml"})));
  if (!g.controller("mpc")) {
    GTEST_SKIP() << "Built without OSQP";
  }
  ASSERT_TRUE(g.start());
  request(g, "stand_up");
  run(g, 2.5);
  request(g, "balance_stand");
  run(g, 0.5);
  request(g, "locomotion_mpc");
  run(g, 1.0);
  EXPECT_NEAR(truth(g).position.z(), 0.36, 0.03);   // the MPC holds the body standing
  const Eigen::Vector3d start = truth(g).position;
  run(g, 4.0, [&g]() {g.set_twist({0.4, 0.0, 0.0}, Eigen::Vector3d::Zero());});
  EXPECT_GT((truth(g).position - start).x(), 1.0);
  EXPECT_LT(tilt_angle(truth(g).orientation), 0.2);
  const double yaw0 = yaw_of(truth(g).orientation);
  run(g, 3.0, [&g]() {g.set_twist(Eigen::Vector3d::Zero(), {0.0, 0.0, 0.8});});
  EXPECT_GT(wrap_angle(yaw_of(truth(g).orientation) - yaw0), 1.5);
  run(g, 2.0);   // stops standing
  EXPECT_EQ(g.state().mode, "locomotion_mpc");
  EXPECT_GT(truth(g).position.z(), 0.3);
}

TEST(Mujoco, R1LipmWalksAndTurns)
{
  if (!r1_assets())
  {
    GTEST_SKIP() << "gaitos_description was built with GAITOS_FETCH_ASSETS=OFF";
  }
  Gaitos g;
  g.configure(testutil::make_params(testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "r1", {"gaitos_mujoco/params/r1_mujoco.yaml"})));
  ASSERT_TRUE(g.start());
  run(g, 2.5);
  request(g, "balance_stand");
  run(g, 0.5);
  // A twist switches balance_stand to the ZMP / LIPM walking mode
  const Eigen::Vector3d start = truth(g).position;
  run(g, 8.0, [&g]() {g.set_twist({0.15, 0.0, 0.0}, Eigen::Vector3d::Zero());});
  EXPECT_EQ(g.state().mode, "locomotion_lipm");
  EXPECT_GT((truth(g).position - start).x(), 0.4);
  EXPECT_LT(tilt_angle(truth(g).orientation), 0.2);
  run(g, 3.0);   // stops with the feet together
  EXPECT_EQ(g.state().mode, "locomotion_lipm");
  const double yaw0 = yaw_of(truth(g).orientation);
  run(g, 6.0, [&g]() {g.set_twist(Eigen::Vector3d::Zero(), {0.0, 0.0, 0.3});});
  EXPECT_GT(wrap_angle(yaw_of(truth(g).orientation) - yaw0), 1.0);
  run(g, 3.0);
  EXPECT_EQ(g.state().mode, "locomotion_lipm");
  EXPECT_GT(truth(g).position.z(), 0.6);
}

TEST(Mujoco, R1Dex3ModelStandsAndWalks)
{
  if (!r1_assets()) {
    GTEST_SKIP() << "gaitos_description was built with GAITOS_FETCH_ASSETS=OFF";
  }
  const std::string share = "package://gaitos_description/robots/r1_description/";
  auto files = testutil::robot_files(std::string(GAITOS_SOURCE_DIR), "r1",
      {"gaitos_mujoco/params/r1_mujoco.yaml"});
  files.push_back(testutil::overrides(
      "robot:\n  urdf: " + share + "urdf/r1_dex3.urdf\n"
      "hardware:\n  model: " + share + "mjcf/scene_dex3.xml"));
  Gaitos g;
  g.configure(testutil::make_params(files));
  ASSERT_TRUE(g.start());
  EXPECT_NEAR(g.model()->tree().mass(), 30.53, 0.05);   // with the hands
  run(g, 2.5);
  EXPECT_EQ(g.state().mode, "stand_up");
  EXPECT_GT(truth(g).position.z(), 0.6);

  // Walking with the weight of the hands (the LIPM controller balances it)
  request(g, "locomotion_lipm");
  const Eigen::Vector3d start = truth(g).position;
  run(g, 6.0, [&g]() {g.set_twist({0.15, 0.0, 0.0}, Eigen::Vector3d::Zero());});
  EXPECT_GT((truth(g).position - start).x(), 0.3);
  EXPECT_LT(tilt_angle(truth(g).orientation), 0.2);
}
