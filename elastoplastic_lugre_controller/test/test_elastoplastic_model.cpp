#include "elastoplastic_lugre_controller/elastoplastic_variable_model.hpp"

#include "control_toolbox/filters.hpp"
#include "elastoplastic_lugre_controller/utils.hpp"
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

namespace elastoplastic {

elastoplastic::ElastoplasticModelData get_model_data() {
  elastoplastic::ElastoplasticModelData data;
  double wc = 5;
  Eigen::Vector6d m = Eigen::Vector6d({10, 10, 10, 10, 10, 10});
  data.inertia_inv.setZero();
  data.inertia_inv.diagonal() = m.cwiseInverse();
  data.k.diagonal() = std::pow(wc, 2) * m;
  data.d.diagonal() = data.k.diagonal().cwiseSqrt() * 1.5;
  data.z_max = 0.5;
  data.z_kmax = 0.2;
  data.z_start = 0.05;
  data.leak_coefficient = 0.05;
  data.enable_axis = std::vector<bool>({true, true, true, false, false, true});
  return data;
}

TEST(ElastoplasticModelTest, Init) {
  elastoplastic::ElastoplasticModel model(get_model_data());
  EXPECT_DOUBLE_EQ(model.z(), 0.0);
}

TEST(ElastoplasticModelTest, privateComputeK) {
  elastoplastic::ElastoplasticModelData data = get_model_data();
  elastoplastic::ElastoplasticModel model(data);
  EXPECT_TRUE(model.compute_k(0.0).isApprox(data.k));
  EXPECT_TRUE(model.compute_k(data.z_start).isApprox(data.k));
  EXPECT_TRUE(model.compute_k(data.z_kmax).isApprox(Eigen::Matrix6d::Zero()));
  EXPECT_TRUE(model.compute_k(data.z_max).isApprox(Eigen::Matrix6d::Zero()));
}

TEST(ElastoplasticModelTest, privateComputeZp) {
  elastoplastic::ElastoplasticModelData data = get_model_data();
  elastoplastic::ElastoplasticModel model(data);

  double z = 0;
  double z_new = 0;
  double u = 1000.0;
  double us = 0;
  const int steps{1000};
  const double period{0.01};
  for (size_t idx = 0; idx < steps; ++idx) {
    us = filters::exponentialSmoothing(u, us, 0.1);
    z_new =
      utils::rk4([model, &period](const double& xin, const double& uin) -> double { return model.compute_zp(xin, uin, period); },
                 z, us, period);
    ASSERT_FALSE(std::isnan(z_new)) << "z: " << z << ", idx: " << idx;
    z = std::max(z_new, 0.0);
  }
  EXPECT_GT(z, 0.0);
  EXPECT_LT(z, data.z_max);
}

TEST(ElastoplasticModelTest, computeImpedanceNoForce) {
  elastoplastic::ElastoplasticModelData data = get_model_data();
  elastoplastic::ElastoplasticModel model(data);
  Eigen::Vector6d v = Eigen::Vector6d::Zero();
  Eigen::Vector6d x = Eigen::Vector6d::Zero();
  Eigen::Vector6d f = Eigen::Vector6d::Zero();
  EXPECT_TRUE(model.compute_impedance(x, v, f, Eigen::Affine3d::Identity()).isZero());
}

TEST(ElastoplasticModelTest, computeImpedanceWithForce) {
  elastoplastic::ElastoplasticModelData data = get_model_data();
  elastoplastic::ElastoplasticModel model(data);
  Eigen::Vector6d v = Eigen::Vector6d::Zero();
  Eigen::Vector6d x = Eigen::Vector6d::Zero();
  Eigen::Vector6d f = Eigen::Vector6d::Constant(10.0);
  Eigen::Vector6d r1 = model.compute_impedance(x, v, f, Eigen::Affine3d::Identity());
  EXPECT_FALSE(r1.isZero());
  ASSERT_DOUBLE_EQ(model.z(), 0.0);
  r1 = model.compute_impedance(x, v, -f, Eigen::Affine3d::Identity());
  EXPECT_FALSE(r1.isZero());
  ASSERT_DOUBLE_EQ(model.z(), 0.0);
}

TEST(ElastoplasticModelTest, updateNoForceNoRef) {
  elastoplastic::ElastoplasticModelData data = get_model_data();
  elastoplastic::ElastoplasticModel model(data);
  ASSERT_FLOAT_EQ(model.z(), 0);

  constexpr long steps{1000};
  constexpr double period{0.02};

  Eigen::Vector6d v = Eigen::Vector6d::Zero();
  Eigen::Vector6d x = Eigen::Vector6d::Zero();
  Eigen::Vector6d f = Eigen::Vector6d::Zero();

  Eigen::Vector6d a_out, v_out, x_out;

  for (size_t idx = 0; idx < steps; ++idx) {
    std::tie(x_out, v_out, a_out) = model.update(x, v, f, Eigen::Affine3d::Identity(), period);
    ASSERT_TRUE(a_out.isZero());
    ASSERT_TRUE(v_out.isZero());
    ASSERT_TRUE(x_out.isZero());
    ASSERT_DOUBLE_EQ(model.z(), 0.0);
    x = x_out;
    v = v_out;
  }
  EXPECT_TRUE(v_out.isZero());
  EXPECT_TRUE(x_out.isZero());
}

TEST(ElastoplasticModelTest, withSmallForceNoRef) {
  elastoplastic::ElastoplasticModelData data = get_model_data();
  elastoplastic::ElastoplasticModel model(data);
  ASSERT_FLOAT_EQ(model.z(), 0);

  constexpr long steps{1000};
  constexpr double period{0.01};

  Eigen::Vector6d v = Eigen::Vector6d::Zero();
  Eigen::Vector6d x = Eigen::Vector6d::Zero();
  Eigen::Vector6d f = Eigen::Vector6d::Constant(0.001);
  Eigen::Vector6d fs = Eigen::Vector6d::Zero();

  Eigen::Vector6d a_out, v_out, x_out;

  for (size_t idx = 0; idx < steps; ++idx) {
    std::transform(f.begin(), f.end(), fs.begin(), fs.begin(),
                   [](const double fin, const double fsin) -> double { return filters::exponentialSmoothing(fin, fsin, 0.1); });
    std::tie(x_out, v_out, a_out) = model.update(x, v, fs, Eigen::Affine3d::Identity(), period);
    ASSERT_FALSE(a_out.isZero());
    ASSERT_FALSE(v_out.isZero());
    ASSERT_FALSE(x_out.isZero());
    ASSERT_LT(model.z(), data.z_start) << "z: " << model.z() << ", idx: " << idx
                                       << ", zp: " << model.compute_zp(model.z(), fs.transpose() * v, period)
                                       << ", fs: " << fs.transpose() << ", v: " << v.transpose();
    ASSERT_TRUE(model.compute_k(model.z()).isApprox(data.k))
      << "z: " << model.z() << ", idx: " << idx << "k: " << model.compute_k(model.z());
    x = x_out;
    v = v_out;
  }
  EXPECT_FALSE(v_out.isZero());
  EXPECT_FALSE(x_out.isZero());
  EXPECT_LT(model.z(), data.z_kmax);
  EXPECT_TRUE(model.compute_k(model.z()).isApprox(data.k));
}

TEST(ElastoplasticModelTest, withHighForceNoRef) {
  elastoplastic::ElastoplasticModelData data = get_model_data();
  elastoplastic::ElastoplasticModel model(data);
  ASSERT_FLOAT_EQ(model.z(), 0);

  constexpr long steps{1000};
  constexpr double period{0.01};

  Eigen::Vector6d v = Eigen::Vector6d::Zero();
  Eigen::Vector6d x = Eigen::Vector6d::Zero();
  Eigen::Vector6d f = Eigen::Vector6d::Constant(100.0);

  Eigen::Vector6d a_out, v_out, x_out;

  for (size_t idx = 0; idx < steps; ++idx) {
    std::tie(x_out, v_out, a_out) = model.update(x, v, f, Eigen::Affine3d::Identity(), period);
    ASSERT_FALSE(a_out.isZero());
    ASSERT_LT(model.z(), data.z_max) << "z: " << model.z();
    x = x_out;
    v = v_out;
  }
  EXPECT_GT(model.z(), data.z_kmax);
  EXPECT_GE(model.z(), data.z_max * 0.99);
  EXPECT_LE(model.z(), data.z_max);
  EXPECT_TRUE((model.compute_k(model.z()).diagonal().array() <= data.k.diagonal().array()).all());
}

TEST(ElastoplasticModelTest, noForcewithRef) {
  elastoplastic::ElastoplasticModelData data = get_model_data();
  elastoplastic::ElastoplasticModel model(data);
  ASSERT_FLOAT_EQ(model.z(), 0);

  constexpr long steps{1000};
  constexpr double period{0.02};

  Eigen::Vector6d v = Eigen::Vector6d::Zero();
  Eigen::Vector6d x = Eigen::Vector6d::Zero();
  Eigen::Vector6d f = Eigen::Vector6d::Zero();

  Eigen::Vector6d xgoal = Eigen::Vector6d::Constant(2.0);
  Eigen::Vector6d vref = (xgoal - x) / (steps * period);

  Eigen::Vector6d a_out, v_out, x_out;

  Eigen::Vector6d xstart = x;
  Eigen::Vector6d xref = xstart;
  for (size_t idx = 0; idx < steps; ++idx) {
    xref = xstart + vref * period * idx;
    std::tie(x_out, v_out, a_out) = model.update(x - xref, v - vref, f, Eigen::Affine3d::Identity(), period);
    ASSERT_DOUBLE_EQ(model.z(), 0.0);
    x = x_out + xref;
    v = v_out + vref;
  }
  EXPECT_DOUBLE_EQ(model.z(), 0.0);
  EXPECT_LT((x - xgoal).norm(), 5E-2) << "x: " << x.transpose() << "\nxgoal: " << xgoal;
}

} // namespace elastoplastic

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
