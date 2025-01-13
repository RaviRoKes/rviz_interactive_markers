#include "rviz_interactive_markers/mt_rviz_ui.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QLabel>
#include <QString>
#include <QPushButton>

namespace rviz_interactive_markers
{

    MTRVizUI::MTRVizUI(QWidget *parent)
        : rviz_common::Panel(parent), is_box_(true)
    {
        // Initialize ROS 2 node
        node_ = std::make_shared<rclcpp::Node>("mt_rviz_ui");


        // Initialize interactive marker server
        marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("interactive_marker_server", node_);

        // Set up UI components
        auto *layout = new QVBoxLayout();

        // Add frame input fields
        auto *frame_layout = new QHBoxLayout();
        frame_layout->addWidget(new QLabel("Child Frame:"));
        child_frame_input_ = new QLineEdit();
        frame_layout->addWidget(child_frame_input_);
        frame_layout->addWidget(new QLabel("Parent Frame:"));
        parent_frame_input_ = new QLineEdit();
        frame_layout->addWidget(parent_frame_input_);
        layout->addLayout(frame_layout);

        // Add position inputs
        auto *pos_layout = new QHBoxLayout();
        pos_layout->addWidget(new QLabel("X:"));
        x_input_ = new QLineEdit();
        pos_layout->addWidget(x_input_);
        pos_layout->addWidget(new QLabel("Y:"));
        y_input_ = new QLineEdit();
        pos_layout->addWidget(y_input_);
        pos_layout->addWidget(new QLabel("Z:"));
        z_input_ = new QLineEdit();
        pos_layout->addWidget(z_input_);
        layout->addLayout(pos_layout);

        // Add broadcast button
        broadcast_button_ = new QPushButton("Broadcast Transform");
        layout->addWidget(broadcast_button_);
        connect(broadcast_button_, &QPushButton::clicked, this, &MTRVizUI::broadcastTransform);

        // Add "Toggle Marker" button
        toggle_marker_button_ = new QPushButton("Toggle Marker");
        layout->addWidget(toggle_marker_button_);
        connect(toggle_marker_button_, &QPushButton::clicked, this, &MTRVizUI::toggleMarker);

        setLayout(layout);
        setFixedSize(400, 300);
    }

    void MTRVizUI::onInitialize()
    {
        // Create a node for the plugin if not already created
        if (!node_)
        {
            node_ = rclcpp::Node::make_shared("mt_rviz_ui_node");
        }

        // spin node in extra thread to allow communication with ROS
        spin_thread_ = std::thread
        ([this]() {
            rclcpp::spin(node_);
        });

        RCLCPP_INFO(node_->get_logger(), "MTRVizUI initialized.");
        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        RCLCPP_INFO(node_->get_logger(), "Transform broadcaster initialized.");
    }

    MTRVizUI::~MTRVizUI()
    {
        marker_server_.reset();
    }

    void MTRVizUI::toggleMarker()
    {
        // Toggle marker shape
        is_box_ = !is_box_;

        // Create a new interactive marker with the updated shape
        auto int_marker = createInteractiveMarker();
        // marker_server_->clear(); // Clear existing markers
        marker_server_->insert(int_marker, std::bind(&MTRVizUI::processFeedback, this, std::placeholders::_1));
        marker_server_->applyChanges();

        RCLCPP_INFO(node_->get_logger(), "Toggled marker to %s.", is_box_ ? "Box" : "Sphere");
    }

    visualization_msgs::msg::InteractiveMarker MTRVizUI::createInteractiveMarker()
    {
        visualization_msgs::msg::InteractiveMarker int_marker;

        // Set marker properties
        int_marker.header.frame_id = "world";
        int_marker.name = "toggle_marker";
        int_marker.description = "Click to Toggle Box/Sphere";
        int_marker.scale = 1.0;

        // Set initial position
        int_marker.pose.position.x = 0.0;
        int_marker.pose.position.y = 0.0;
        int_marker.pose.position.z = 0.0;

        // Create a marker (box or sphere)
        visualization_msgs::msg::Marker marker;
        marker.type = is_box_ ? visualization_msgs::msg::Marker::CUBE : visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // Add marker control
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.always_visible = true;
        control.markers.push_back(marker);
        int_marker.controls.push_back(control);

        // Add a button control for interaction
        visualization_msgs::msg::InteractiveMarkerControl button_control;
        button_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;

        visualization_msgs::msg::Marker button_control_marker = marker;
        button_control_marker.pose.position.x = 1.0;
        // button_control_marker.color.a = 0.0;
        button_control.markers.push_back(button_control_marker);
        int_marker.controls.push_back(button_control);

        return int_marker;
    }

    void MTRVizUI::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        RCLCPP_INFO(node_->get_logger(), "Feedback from marker '%s'.", feedback->marker_name.c_str());
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK)
        {
            RCLCPP_INFO(node_->get_logger(), "Marker '%s' clicked.", feedback->marker_name.c_str());
            toggleMarker(); // Toggle marker shape on click
        }
    }

    void MTRVizUI::broadcastTransform()
    {
        std::string parent_frame = parent_frame_input_->text().toStdString();
        std::string child_frame = child_frame_input_->text().toStdString();

        // Check if either frame name is empty or contains spaces
        if (parent_frame.empty() || child_frame.empty() || parent_frame.find(" ") != std::string::npos || child_frame.find(" ") != std::string::npos)
        {
            RCLCPP_ERROR(node_->get_logger(), "Parent or Child frame is empty or contains spaces.");
            return;
        }
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        transform.header.stamp = node_->get_clock()->now();

        transform.transform.translation.x = x_input_->text().toDouble();
        transform.transform.translation.y = y_input_->text().toDouble();
        transform.transform.translation.z = z_input_->text().toDouble();

        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, 0.0); // Identity rotation
        transform.transform.rotation = tf2::toMsg(quat);

        tf_broadcaster_->sendTransform(transform);
    }

} // namespace rviz_interactive_markers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_interactive_markers::MTRVizUI, rviz_common::Panel)
//ff