#include "elastoplastic_lugre_controller/elastoplastic_model.hpp"

#include <eigen3/Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

elastoplastic::ElastoplasticModelData
get_model_data()
{
  elastoplastic::ElastoplasticModelData data;
  data.inertia_inv = Eigen::Vector3d({10.0, 10.0, 10.0}).cwiseInverse();
  data.lugre.sigma_0 = 1000.0;
  data.lugre.sigma_1 = 10.0;
  data.lugre.sigma_2 = 10.0;
  data.lugre.z_ss = 0.5;
  data.lugre.z_ba = 0.1;
  data.lugre.tau_w = 0.02;
  data.reset_condition.reset_window_size = 1000;
  data.reset_condition.reset_threshold = 1;
  return data;
}

TEST(ElastoplasticModelTest, init)
{
  elastoplastic::ElastoplasticModel model(get_model_data());
  EXPECT_TRUE(model.z().isZero());
  EXPECT_TRUE(model.w().isZero());
  EXPECT_EQ(model.r(), 0.0);
}

TEST(ElastoplasticModelTest, with_no_force)
{
  elastoplastic::ElastoplasticModel model(get_model_data());
  ASSERT_TRUE(model.z().isZero());
  ASSERT_TRUE(model.w().isZero());
  ASSERT_EQ(model.r(), 0.0);

  const long steps = 1000;
  const double period = 0.02;

  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_out;

  for(size_t idx = 0; idx < steps; ++idx)
  {
    acc_out = model.update(v, Eigen::Vector3d::Zero(), period);
    v = acc_out * period;
  }

  EXPECT_TRUE(v.isZero());
  EXPECT_TRUE(model.z().isZero());
  EXPECT_TRUE(model.w().isZero());
  EXPECT_EQ(model.r(), 0.0);
}

TEST(ElastoplasticModelTest, with_small_force)
{
  elastoplastic::ElastoplasticModel model(get_model_data());
  ASSERT_TRUE(model.z().isZero());
  ASSERT_TRUE(model.w().isZero());
  ASSERT_EQ(model.r(), 0.0);

  const long steps = 10000;
  const double period = 0.02;

  double limit_force = model.params().lugre.sigma_0 * model.params().lugre.z_ba;

  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d w({0.9 * limit_force, 0.0, 0.0});
  Eigen::Vector3d acc_out;

  for(size_t idx = 0; idx < steps; ++idx)
  {
    acc_out = model.update(v, w, period);
    v = acc_out * period;
  }

  EXPECT_TRUE(v.isZero());
  EXPECT_FALSE(model.z().isZero());
  EXPECT_LE(model.z().maxCoeff(), model.params().lugre.z_ba);
  EXPECT_TRUE(model.w().isZero());
  EXPECT_EQ(model.r(), 0.0);
}

TEST(ElastoplasticModelTest, with_high_force)
{
  elastoplastic::ElastoplasticModel model(get_model_data());
  ASSERT_TRUE(model.z().isZero());
  ASSERT_TRUE(model.w().isZero());
  ASSERT_EQ(model.r(), 0.0);

  const long steps = 1000;
  const double period = 0.02;

  double limit_force = model.params().lugre.sigma_0 * model.params().lugre.z_ba;

  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d w({10 * limit_force, 0.0, 0.0});
  Eigen::Vector3d acc_out;

  for(size_t idx = 0; idx < steps; ++idx)
  {
    acc_out = model.update(v, w, period);
    v = acc_out * period;
  }

  EXPECT_FALSE(v.isZero());
  EXPECT_GT(model.z().maxCoeff(), model.params().lugre.z_ba);
  EXPECT_LE(model.z().maxCoeff(), model.params().lugre.z_ss);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
