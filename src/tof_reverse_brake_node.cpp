#include <hm_ld1_sdk/hm_ld1_sdk.hpp>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace {

enum class Axis {
    X,
    Y,
    Z,
};

struct Parameters {
    std::string uvcDevice = "/dev/video0";
    std::string uvcProfile = "pointcloud";
    std::string cmdVelTopic = "/cmd_vel";
    double reverseSpeedMps = 0.10;
    double publishRateHz = 20.0;
    int frameTimeoutMs = 2000;
    bool stopOnNoData = true;

    double stopDistanceM = 0.10;
    double minDistanceM = 0.03;
    int minObstaclePoints = 8;
    int clearRequiredFrames = 3;

    Axis distanceAxis = Axis::Z;
    double distanceSign = 1.0;
    Axis heightAxis = Axis::Y;
    double heightSign = -1.0;
    double heightMinM = -0.10;
    double heightMaxM = 0.10;
};

struct ObstacleStats {
    int pointCount = 0;
    double nearestDistanceM = std::numeric_limits<double>::infinity();
};

bool ParseAxis(const std::string& raw, Axis* axis) {
    if (axis == nullptr) {
        return false;
    }
    std::string value = raw;
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });

    if (value == "x") {
        *axis = Axis::X;
        return true;
    }
    if (value == "y") {
        *axis = Axis::Y;
        return true;
    }
    if (value == "z") {
        *axis = Axis::Z;
        return true;
    }
    return false;
}

std::string AxisName(Axis axis) {
    switch (axis) {
        case Axis::X:
            return "x";
        case Axis::Y:
            return "y";
        case Axis::Z:
            return "z";
    }
    return "unknown";
}

Axis LoadAxisParam(ros::NodeHandle& privateNh, const std::string& name, Axis defaultAxis) {
    std::string raw = AxisName(defaultAxis);
    privateNh.param(name, raw, raw);

    Axis axis = defaultAxis;
    if (!ParseAxis(raw, &axis)) {
        ROS_WARN("Invalid axis parameter ~%s='%s'; using '%s'", name.c_str(), raw.c_str(), AxisName(defaultAxis).c_str());
    }
    return axis;
}

Parameters LoadParameters(ros::NodeHandle& privateNh) {
    Parameters params;
    privateNh.param("uvc_device", params.uvcDevice, params.uvcDevice);
    privateNh.param("uvc_profile", params.uvcProfile, params.uvcProfile);
    privateNh.param("cmd_vel_topic", params.cmdVelTopic, params.cmdVelTopic);
    privateNh.param("reverse_speed_mps", params.reverseSpeedMps, params.reverseSpeedMps);
    privateNh.param("publish_rate_hz", params.publishRateHz, params.publishRateHz);
    privateNh.param("frame_timeout_ms", params.frameTimeoutMs, params.frameTimeoutMs);
    privateNh.param("stop_on_no_data", params.stopOnNoData, params.stopOnNoData);

    privateNh.param("stop_distance_m", params.stopDistanceM, params.stopDistanceM);
    privateNh.param("min_distance_m", params.minDistanceM, params.minDistanceM);
    privateNh.param("min_obstacle_points", params.minObstaclePoints, params.minObstaclePoints);
    privateNh.param("clear_required_frames", params.clearRequiredFrames, params.clearRequiredFrames);

    params.distanceAxis = LoadAxisParam(privateNh, "distance_axis", params.distanceAxis);
    privateNh.param("distance_sign", params.distanceSign, params.distanceSign);
    params.heightAxis = LoadAxisParam(privateNh, "height_axis", params.heightAxis);
    privateNh.param("height_sign", params.heightSign, params.heightSign);
    privateNh.param("height_min_m", params.heightMinM, params.heightMinM);
    privateNh.param("height_max_m", params.heightMaxM, params.heightMaxM);

    params.reverseSpeedMps = std::abs(params.reverseSpeedMps);
    params.publishRateHz = std::max(1.0, params.publishRateHz);
    params.frameTimeoutMs = std::max(1, params.frameTimeoutMs);
    params.stopDistanceM = std::max(0.0, params.stopDistanceM);
    params.minDistanceM = std::max(0.0, params.minDistanceM);
    params.minObstaclePoints = std::max(1, params.minObstaclePoints);
    params.clearRequiredFrames = std::max(1, params.clearRequiredFrames);
    if (params.heightMaxM < params.heightMinM) {
        std::swap(params.heightMinM, params.heightMaxM);
    }

    return params;
}

hm_ld1::UvcStreamProfile ParseUvcProfile(const std::string& raw) {
    std::string value = raw;
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });

    if (value == "auto") {
        return hm_ld1::UvcStreamProfile::Auto;
    }
    if (value == "depth" || value == "depth40x30") {
        return hm_ld1::UvcStreamProfile::Depth40x30;
    }
    if (value == "mixed" || value == "mixed120x90") {
        return hm_ld1::UvcStreamProfile::Mixed120x90;
    }
    if (value == "pointcloud" || value == "pointcloud160x120") {
        return hm_ld1::UvcStreamProfile::PointCloud160x120;
    }
    if (value == "raw" || value == "raw480x360") {
        return hm_ld1::UvcStreamProfile::Raw480x360;
    }

    ROS_WARN("Invalid uvc_profile '%s'; using pointcloud", raw.c_str());
    return hm_ld1::UvcStreamProfile::PointCloud160x120;
}

double AxisValueM(const hm_ld1::Point3f& point, Axis axis, double sign) {
    double valueMm = 0.0;
    switch (axis) {
        case Axis::X:
            valueMm = point.x;
            break;
        case Axis::Y:
            valueMm = point.y;
            break;
        case Axis::Z:
            valueMm = point.z;
            break;
    }
    return sign * valueMm / 1000.0;
}

bool IsFinitePoint(const hm_ld1::Point3f& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

ObstacleStats AnalyzePointCloud(const hm_ld1::PointCloudFrame& pointCloud, const Parameters& params) {
    ObstacleStats stats;
    for (const hm_ld1::Point3f& point : pointCloud.points) {
        if (!IsFinitePoint(point) || point.z <= 0.0f) {
            continue;
        }

        const double heightM = AxisValueM(point, params.heightAxis, params.heightSign);
        if (heightM < params.heightMinM || heightM > params.heightMaxM) {
            continue;
        }

        const double distanceM = AxisValueM(point, params.distanceAxis, params.distanceSign);
        if (!std::isfinite(distanceM) || distanceM < params.minDistanceM || distanceM > params.stopDistanceM) {
            continue;
        }

        ++stats.pointCount;
        stats.nearestDistanceM = std::min(stats.nearestDistanceM, distanceM);
    }
    return stats;
}

geometry_msgs::Twist MakeCommand(bool stopActive, double reverseSpeedMps) {
    geometry_msgs::Twist twist;
    if (!stopActive) {
        twist.linear.x = -std::abs(reverseSpeedMps);
    }
    return twist;
}

class ToFReverseBrakeNode {
public:
    ToFReverseBrakeNode(ros::NodeHandle nh, ros::NodeHandle privateNh)
        : nh_(nh),
          privateNh_(privateNh),
          params_(LoadParameters(privateNh_)),
          cmdVelPublisher_(nh_.advertise<geometry_msgs::Twist>(params_.cmdVelTopic, 1)),
          stopActivePublisher_(privateNh_.advertise<std_msgs::Bool>("stop_active", 1, true)),
          obstacleCountPublisher_(privateNh_.advertise<std_msgs::Int32>("obstacle_point_count", 1, true)),
          nearestDistancePublisher_(privateNh_.advertise<std_msgs::Float32>("nearest_obstacle_distance", 1, true)) {}

    bool OpenCamera() {
        hm_ld1::CameraConfig config;
        config.transportType = hm_ld1::TransportType::Uvc;
        config.uvc.device = params_.uvcDevice;
        const hm_ld1::UvcStreamProfile profile = ParseUvcProfile(params_.uvcProfile);
        config.uvc.workingProfile = profile;
        config.uvc.bootstrapCalibration = profile == hm_ld1::UvcStreamProfile::Depth40x30;
        config.uvc.bootstrapProfile = hm_ld1::UvcStreamProfile::Mixed120x90;
        config.uvc.bootstrapTimeoutMs = 1500;

        std::string error;
        if (!camera_.Open(config, &error)) {
            ROS_ERROR("Failed to open HM-LD1 UVC camera '%s': %s", params_.uvcDevice.c_str(), error.c_str());
            return false;
        }

        ROS_INFO("Opened HM-LD1 camera: %s", camera_.Describe().c_str());
        ROS_INFO("Reverse speed %.3f m/s, stop distance %.3f m, min obstacle points %d",
                 params_.reverseSpeedMps,
                 params_.stopDistanceM,
                 params_.minObstaclePoints);
        return true;
    }

    void Spin() {
        ros::Rate rate(params_.publishRateHz);
        while (ros::ok()) {
            const bool pointCloudUpdated = PollCameraOnce();
            UpdateBrakeState(pointCloudUpdated);

            PublishDiagnostics(stopActive_);
            cmdVelPublisher_.publish(MakeCommand(stopActive_, params_.reverseSpeedMps));

            ros::spinOnce();
            rate.sleep();
        }

        cmdVelPublisher_.publish(MakeCommand(true, params_.reverseSpeedMps));
        camera_.Close();
    }

private:
    bool PollCameraOnce() {
        hm_ld1::FrameSet frame;
        std::string error;
        if (!camera_.Poll(&frame, &error)) {
            ROS_ERROR_THROTTLE(1.0, "HM-LD1 poll failed: %s", error.c_str());
            return false;
        }

        if (frame.pointCloud.empty()) {
            return false;
        }

        latestObstacleStats_ = AnalyzePointCloud(frame.pointCloud, params_);
        hasPointCloud_ = true;
        lastPointCloudTime_ = ros::Time::now();

        ROS_DEBUG_THROTTLE(1.0,
                           "pointcloud width=%u height=%u points=%zu obstacle_points=%d nearest=%.3f",
                           frame.pointCloud.width,
                           frame.pointCloud.height,
                           frame.pointCloud.points.size(),
                           latestObstacleStats_.pointCount,
                           latestObstacleStats_.nearestDistanceM);
        return true;
    }

    bool IsPointCloudStale() const {
        if (!params_.stopOnNoData) {
            return false;
        }
        if (!hasPointCloud_) {
            return true;
        }
        const double age = (ros::Time::now() - lastPointCloudTime_).toSec();
        return age > static_cast<double>(params_.frameTimeoutMs) / 1000.0;
    }

    void UpdateBrakeState(bool pointCloudUpdated) {
        const bool stale = IsPointCloudStale();
        bool nextStopActive = stopActive_;

        if (stale) {
            clearFrameCount_ = 0;
            nextStopActive = true;
        } else if (pointCloudUpdated) {
            if (latestObstacleStats_.pointCount >= params_.minObstaclePoints) {
                clearFrameCount_ = 0;
                nextStopActive = true;
            } else {
                clearFrameCount_ = std::min(clearFrameCount_ + 1, params_.clearRequiredFrames);
                if (clearFrameCount_ >= params_.clearRequiredFrames) {
                    nextStopActive = false;
                }
            }
        }

        if (nextStopActive != stopActive_) {
            stopActive_ = nextStopActive;
            if (stopActive_) {
                ROS_WARN("Brake active: obstacle_points=%d nearest=%.3f stale=%s",
                         latestObstacleStats_.pointCount,
                         latestObstacleStats_.nearestDistanceM,
                         stale ? "true" : "false");
            } else {
                ROS_INFO("Brake released after %d clear frame(s): obstacle_points=%d",
                         clearFrameCount_,
                         latestObstacleStats_.pointCount);
            }
        }
    }

    void PublishDiagnostics(bool stopActive) {
        std_msgs::Bool stopMsg;
        stopMsg.data = stopActive;
        stopActivePublisher_.publish(stopMsg);

        std_msgs::Int32 countMsg;
        countMsg.data = latestObstacleStats_.pointCount;
        obstacleCountPublisher_.publish(countMsg);

        std_msgs::Float32 distanceMsg;
        distanceMsg.data = static_cast<float>(latestObstacleStats_.nearestDistanceM);
        nearestDistancePublisher_.publish(distanceMsg);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle privateNh_;
    Parameters params_;
    ros::Publisher cmdVelPublisher_;
    ros::Publisher stopActivePublisher_;
    ros::Publisher obstacleCountPublisher_;
    ros::Publisher nearestDistancePublisher_;
    hm_ld1::Camera camera_;

    bool hasPointCloud_ = false;
    bool stopActive_ = true;
    int clearFrameCount_ = 0;
    ros::Time lastPointCloudTime_;
    ObstacleStats latestObstacleStats_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "tof_reverse_brake_node");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    ToFReverseBrakeNode node(nh, privateNh);
    if (!node.OpenCamera()) {
        return 1;
    }

    node.Spin();
    return 0;
}
