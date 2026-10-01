#include <gtest/gtest.h>

#include "Integrators/eulerIntegrator.h"

TEST(EulerIntegratorTest, IntegratesFirstOrderState)
{
    EulerIntegrator integrator;

    const Eigen::Vector3d value{1.0, 2.0, 3.0};
    const Eigen::Vector3d derivative{2.0, 3.0, 4.0};
    const double dt = 0.5;

    const Eigen::Vector3d result =
        integrator.integrateFirstOrder(value, derivative, dt);

    EXPECT_DOUBLE_EQ(result.x(), 2.0);
    EXPECT_DOUBLE_EQ(result.y(), 3.5);
    EXPECT_DOUBLE_EQ(result.z(), 5.0);
}

TEST(EulerIntegratorTest, IntegratesSecondOrderState)
{
    EulerIntegrator integrator;

    const Eigen::Vector3d position{1.0, 2.0, 3.0};
    const Eigen::Vector3d velocity{2.0, 0.0, -1.0};
    const Eigen::Vector3d acceleration{4.0, 2.0, 6.0};
    const double dt = 0.5;

    const Eigen::Vector3d result =
        integrator.integrateSecondOrder(
            position,
            velocity,
            acceleration,
            dt
            );

    EXPECT_DOUBLE_EQ(result.x(), 2.5);
    EXPECT_DOUBLE_EQ(result.y(), 2.25);
    EXPECT_DOUBLE_EQ(result.z(), 3.25);
}