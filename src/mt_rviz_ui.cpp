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
#include <QGroupBox>
#include <QFormLayout>

namespace rviz_interactive_markers
{
    MTRVizUI::MTRVizUI(QWidget *parent)
        : rviz_common::Panel(parent), is_box_(true)
    {
        // Initialize ROS 2 node
        node_ = std::make_shared<rclcpp::Node>("mt_rviz_ui");

        // Initialize interactive marker server
        marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "interactive_marker_server", node_);

        // Initialize input fields for frames
        child_frame_input_ = new QLineEdit(this);
        child_frame_input_->setPlaceholderText("Enter Child Frame Name");

        parent_frame_input_ = new QLineEdit(this);
        parent_frame_input_->setPlaceholderText("Enter Parent Frame Name");

        // Group for frame inputs
        QGroupBox *frame_group = new QGroupBox("Frames", this);
        QFormLayout *frame_layout = new QFormLayout;
        frame_layout->addRow("Child Frame:", child_frame_input_);
        frame_layout->addRow("Parent Frame:", parent_frame_input_);

        // Initialize the broadcast button and add to the frame layout
        broadcast_button_ = new QPushButton("Broadcast Transform", this);
        connect(broadcast_button_, &QPushButton::clicked, this, &MTRVizUI::broadcastTransform);
        frame_layout->addRow(broadcast_button_); // Add the button here

        frame_group->setLayout(frame_layout);

        // Initialize input fields for cylinder dimensions
        height_input_ = new QLineEdit(this);
        height_input_->setPlaceholderText("Enter Cylinder Height");

        radius_input_ = new QLineEdit(this);
        radius_input_->setPlaceholderText("Enter Cylinder Radius");

        // Group for cylinder dimensions
        QGroupBox *cylinder_group = new QGroupBox("Cylinder Dimensions(mm)", this);
        QFormLayout *cylinder_layout = new QFormLayout;
        cylinder_layout->addRow("Height:", height_input_);
        cylinder_layout->addRow("Radius:", radius_input_);
        cylinder_group->setLayout(cylinder_layout);

        // Initialize the toggle marker button
        toggle_marker_button_ = new QPushButton("Create/Toggle Marker", this);
        connect(toggle_marker_button_, &QPushButton::clicked, this, &MTRVizUI::toggleMarker);

        // Set up layout for the panel
        QVBoxLayout *main_layout = new QVBoxLayout;
        main_layout->addWidget(frame_group);    // Frames group including the button
        main_layout->addWidget(cylinder_group); // Cylinder dimensions group
        main_layout->addWidget(toggle_marker_button_);

        setLayout(main_layout);

        // Set fixed size for the panel
        setFixedSize(500, 300);
    }
    void MTRVizUI::onInitialize()
    {
        // Create a node for the plugin if not already created
        if (!node_)
        {
            node_ = rclcpp::Node::make_shared("mt_rviz_ui_node");
        }

        // spin node in extra thread to allow communication with ROS
        spin_thread_ = std::thread([this]()
                                   { rclcpp::spin(node_); });

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
        const std::vector<std::string> frames = {
            "a0_storage_link", "a1_storage_link", "a2_storage_link", "a3_storage_link", "a4_storage_link",
            "b0_storage_link", "b1_storage_link", "b2_storage_link", "b3_storage_link", "c0_storage_link",
            "c1_storage_link", "c2_storage_link", "c3_storage_link", "c4_storage_link", "d0_storage_link",
            "d1_storage_link", "d2_storage_link", "d3_storage_link", "e0_storage_link", "e1_storage_link",
            "e2_storage_link", "e3_storage_link", "e4_storage_link", "f0_storage_link", "f1_storage_link",
            "f2_storage_link", "f3_storage_link", "g0_storage_link", "g1_storage_link", "g2_storage_link",
            "g3_storage_link", "g4_storage_link", "h0_storage_link", "h1_storage_link", "h2_storage_link",
            "h3_storage_link", "i0_storage_link", "i1_storage_link", "i2_storage_link", "i3_storage_link",
            "i4_storage_link", "j0_storage_link", "j1_storage_link", "j2_storage_link", "j3_storage_link",
            "k0_storage_link", "k1_storage_link", "k2_storage_link", "k3_storage_link", "k4_storage_link"};
        
       
        marker_states_.clear(); // Clear previous states

        // Initialize marker states (default to cube)
        marker_states_ = std::vector<std::vector<bool>>(frames.size(), std::vector<bool>(frames.size(), true));

        // Clear existing markers from the server
        marker_server_->clear();

        // Create markers for each frame
        for (size_t i = 0; i < frames.size(); ++i)
        {                                        // Cast the index `i` to int when passing it to createInteractiveMarker
            int marker_id = static_cast<int>(i); // Explicit cast to int
            double x = 0.0;                      // Customize marker positioning as needed
            double y = 0.0;                      
            double z = 0.0;                      
            auto int_marker = createInteractiveMarker(marker_id, marker_id, x, y, z, frames[i]);
            marker_server_->insert(int_marker, std::bind(&MTRVizUI::processFeedback, this, std::placeholders::_1));
        }

        marker_server_->applyChanges();
        RCLCPP_INFO(node_->get_logger(), "Created markers on specified frames.");
    }
    visualization_msgs::msg::InteractiveMarker MTRVizUI::createInteractiveMarker(int i, int j, double x, double y, double z, const std::string &frame_id)
    {
        visualization_msgs::msg::InteractiveMarker int_marker;

        // Use the provided frame ID
        int_marker.header.frame_id = frame_id;

        int_marker.name = "marker_" + std::to_string(i) + "_" + std::to_string(j);
        int_marker.description = "Marker";
        int_marker.scale = 1.0;

        // Set marker position
        int_marker.pose.position.x = x;
        int_marker.pose.position.y = y;
        int_marker.pose.position.z = z;

        // Create a marker (cube or cylinder based on state)
        visualization_msgs::msg::Marker marker;
        marker.type = marker_states_[i][j] ? visualization_msgs::msg::Marker::CUBE : visualization_msgs::msg::Marker::CYLINDER;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
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
        button_control.markers.push_back(marker);
        int_marker.controls.push_back(button_control);

        return int_marker;
    }

    void MTRVizUI::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        try
        {
            // Identify the clicked marker by its name
            std::string marker_name = feedback->marker_name;
            RCLCPP_INFO(node_->get_logger(), "Marker clicked: %s, Frame: %s", marker_name.c_str(), feedback->header.frame_id.c_str());

            // Extract grid location from the marker name
            size_t underscore_pos = marker_name.find('_');
            if (underscore_pos == std::string::npos)
            {
                RCLCPP_ERROR(node_->get_logger(), "Invalid marker name format: %s", marker_name.c_str());
                return;
            }

            int i = std::stoi(marker_name.substr(7, underscore_pos - 7)); // Extract row index
            int j = std::stoi(marker_name.substr(underscore_pos + 1));    // Extract column index

            if (i < 0 || j < 0 || i >= static_cast<int>(marker_states_.size()) || j >= static_cast<int>(marker_states_[0].size()))
            {
                RCLCPP_ERROR(node_->get_logger(), "Grid location out of range: i=%d, j=%d", i, j);
                return;
            }

            // Toggle the marker type (cube/cylinder)
            marker_states_[i][j] = !marker_states_[i][j];

            // Retrieve user inputs for dimensions
            bool height_valid = false, radius_valid = false;
            double height = height_input_->text().toDouble(&height_valid);
            double radius = radius_input_->text().toDouble(&radius_valid);

            if (!height_valid || !radius_valid || height <= 0 || radius <= 0)
            {
                RCLCPP_ERROR(node_->get_logger(), "Invalid dimensions provided: height=%.2f, radius=%.2f", height, radius);
                return;
            }
            // Update the marker dimensions and type
            visualization_msgs::msg::InteractiveMarker int_marker = createInteractiveMarker(i, j, 0.0, 0.0, 0.0, feedback->header.frame_id);

            for (auto &control : int_marker.controls)
            {
                for (auto &marker : control.markers)
                {
                    marker.type = marker_states_[i][j] ? visualization_msgs::msg::Marker::CUBE : visualization_msgs::msg::Marker::CYLINDER;
                    marker.scale.x = radius * 2.0 / 1000.0; // Convert from mm to meters
                    marker.scale.y = radius * 2.0 / 1000.0;
                    marker.scale.z = height / 1000.0;
                    marker.color.r = 1.0; // Highlight the updated marker
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.color.a = 1.0;
                }
            }

            // Update the marker in the server and apply changes
            marker_server_->insert(int_marker);
            marker_server_->applyChanges();

            RCLCPP_INFO(node_->get_logger(), "Marker updated: i=%d, j=%d, type=%s, height=%.2fmm, radius=%.2fmm",
                        i, j, marker_states_[i][j] ? "CUBE" : "CYLINDER", height, radius);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Exception in processFeedback: %s", e.what());
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

        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, 0.0); // Identity rotation
        transform.transform.rotation = tf2::toMsg(quat);

        tf_broadcaster_->sendTransform(transform);
    }

} // namespace rviz_interactive_markers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_interactive_markers::MTRVizUI, rviz_common::Panel)
// ff dfg firldkfk kifg 