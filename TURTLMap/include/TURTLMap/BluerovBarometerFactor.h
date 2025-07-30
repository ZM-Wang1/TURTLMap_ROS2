// #include <rosbag/bag.h>
// #include <rosbag/view.h>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/navigation/BarometricFactor.h>
#include <gtsam/inference/Symbol.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <gtsam/navigation/NavState.h>
#include <Eigen/Dense>
#include <std_msgs/msg/string.hpp>

#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/navigation/ImuFactor.h>

using namespace gtsam;

class BluerovBarometerFactor : public NoiseModelFactor1<Pose3>
{
private:
    /* data */
    double measured_;

public:
    /// Constructor
    BluerovBarometerFactor()
    {
    }
    virtual ~BluerovBarometerFactor() {}
    BluerovBarometerFactor(Key key, double measured, const SharedNoiseModel &model) : NoiseModelFactor1<Pose3>(model, key), measured_(measured) {}
    // BluerovBarometerFactor(/* args */);
    // ~BluerovBarometerFactor();
    // virtual ~BluerovBarometerFactor() override {}
    Vector evaluateError(const Pose3 &pose, boost::optional<gtsam::Matrix &> H) const;
};

