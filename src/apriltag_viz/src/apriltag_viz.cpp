// ros
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>


class AprilVizNode : public rclcpp::Node {
public:
    AprilVizNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions())
    : Node("apriltag_viz", rclcpp::NodeOptions(options).use_intra_process_comms(true))
    {
        get_parameter_or<std::string>("overlay_mode", overlay_mode, "axes");

        std::string image_transport;
        get_parameter_or<std::string>("image_transport", image_transport, "raw");

        pub_tags = image_transport::create_publisher(this, "tag_detections_image");

        sub_img = image_transport::create_subscription(this, "image",
            std::bind(&AprilVizNode::onImage, this, std::placeholders::_1),
            image_transport);

        sub_tag = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "detections", rclcpp::QoS(1),
            std::bind(&AprilVizNode::onTags, this, std::placeholders::_1));
    }

private:
    image_transport::Subscriber sub_img;
    image_transport::Publisher pub_tags;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr sub_tag;

    cv::Mat img;
    cv::Mat merged;
    cv::Mat overlay;
    std::string overlay_mode;

    static const std::array<cv::Scalar, 4> colours;

    static std::array<double, 2> project(const std::array<double,9> H, // homography matrix
                                         const std::array<double,2> pc) // point in camera
    {
        std::array<double,2> pi;    // point in image
        const auto z = H[3*2+0] * pc[0] + H[3*2+1] * pc[1] + H[3*2+2];
        for(uint i(0); i<2; i++) {
            pi[i] = (H[3*i+0] * pc[0] + H[3*i+1] * pc[1] + H[3*i+2]) / z;
        }
        return pi;
    }

    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg_img) {
        img = cv_bridge::toCvCopy(msg_img)->image;

        if(overlay.empty()) {
            merged = img;
        }
        else {
            // blend overlay and image
            double alpha;
            get_parameter_or("alpha", alpha, 0.5);
            cv::addWeighted(img, 1, overlay, alpha, 0, merged, -1);
        }

        pub_tags.publish(cv_bridge::CvImage(msg_img->header, msg_img->encoding, merged).toImageMsg());
    }

    void onTags(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg_tag) {
        if(img.empty())
            return;

        // overlay with transparent background
        overlay = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0,0,0,0));

        for(const auto& d : msg_tag->detections) {
            if(overlay_mode=="axes") {
                // axes
                const auto c = project(d.homography, {{0,0}});
                const auto x = project(d.homography, {{1,0}});
                const auto y = project(d.homography, {{0,1}});
                cv::line(overlay, cv::Point2d(c[0], c[1]), cv::Point2d(x[0],x[1]), cv::Scalar(0,0,255,255), 3);
                cv::line(overlay, cv::Point2d(c[0], c[1]), cv::Point2d(y[0],y[1]), cv::Scalar(0,255,0,255), 3);
            }
            else if(overlay_mode=="tri") {
                // triangle patches
                std::array<cv::Point,3> points;
                points[0].x = d.centre.x;
                points[0].y = d.centre.y;

                for(uint i(0); i<4; i++) {
                    points[1].x = d.corners[i%4].x;
                    points[1].y = d.corners[i%4].y;
                    points[2].x = d.corners[(i+1)%4].x;
                    points[2].y = d.corners[(i+1)%4].y;

                    cv::fillConvexPoly(overlay, points.data(), 3, colours[i]);
                }
            }
            else {
                throw std::runtime_error("unknown overlay mode");
            }

            for(uint i(0); i<4; i++) {
                cv::circle(overlay, cv::Point(d.corners[i].x, d.corners[i].y), 5, colours[i], 2);
            }
        }
    }
};

const std::array<cv::Scalar, 4> AprilVizNode::colours = {{
    cv::Scalar(0,0,255,255),    // red
    cv::Scalar(0,255,0,255),    // green
    cv::Scalar(255,0,0,255),    // blue
    cv::Scalar(0,255,255,255)   // yellow
}};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(AprilVizNode)
