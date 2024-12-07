#include "rviz_interactive_markers/mt_rviz_ui.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit> 
#include <QLabel>

namespace rviz_interactive_markers
{
    MTRVizUI::MTRVizUI(QWidget *parent)
        : rviz_common::Panel(parent), marker_spacing_(1.0), grid_rows_(4), grid_cols_(4),
          cylinder_radius_(0.05), cylinder_height_(0.2)
    {
        // Initialize ROS 2 node
        node_ = std::make_shared<rclcpp::Node>("mt_rviz_ui");

        // Initialize marker server and TF broadcaster
        marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("mt_marker_server", node_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

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

        // Add "Create Grid" button
        create_grid_button_ = new QPushButton("Create Grid of Markers");
        layout->addWidget(create_grid_button_);
        connect(create_grid_button_, &QPushButton::clicked, this, &MTRVizUI::createGrid);

        // Add "Add 5x10 Interactive Markers" button
        add_5x10_grid_button_ = new QPushButton("Add 5x10 Interactive Markers");
        layout->addWidget(add_5x10_grid_button_);
        connect(add_5x10_grid_button_, &QPushButton::clicked, this, &MTRVizUI::add5x10InteractiveMarkers);

        setLayout(layout);
        setFixedSize(400, 300); // Or any other size depending on your layout
    }

    MTRVizUI::~MTRVizUI()
    {
        marker_server_.reset();
        tf_broadcaster_.reset();
    }

    void MTRVizUI::onInitialize()
    {
        Panel::onInitialize();
        createGrid();
    }

    void MTRVizUI::createGrid()
    {
        double marker_size = 0.5; // Define marker size here
        for (int row = 0; row < grid_rows_; ++row)
        {
            for (int col = 0; col < grid_cols_; ++col)
            {
                createBoxMarker(row, col, marker_size);
            }
        }
    }

    void MTRVizUI::createBoxMarker(int row, int col, double marker_size)
    {
        visualization_msgs::msg::InteractiveMarker marker;
        marker.header.frame_id = "world";
        marker.name = "box_marker_" + std::to_string(row) + "_" + std::to_string(col);
        marker.description = marker.name;
        marker.pose.position.x = col * marker_spacing_;
        marker.pose.position.y = row * marker_spacing_;
        marker.pose.position.z = 0.0;

        visualization_msgs::msg::InteractiveMarkerControl control;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
        control.always_visible = true;

        visualization_msgs::msg::Marker box;
        box.type = visualization_msgs::msg::Marker::CUBE;
        box.scale.x = marker_size; // Use the passed marker size
        box.scale.y = marker_size;
        box.scale.z = marker_size;
        box.color.r = 0.5;
        box.color.g = 0.5;
        box.color.b = 0.5;
        box.color.a = 1.0;

        control.markers.push_back(box);
        marker.controls.push_back(control);

        marker_server_->insert(marker, std::bind(&MTRVizUI::processFeedback, this, std::placeholders::_1));
        marker_server_->applyChanges();
    }

    void MTRVizUI::add5x10InteractiveMarkers()
    {
        // Create a grid of 5 rows and 10 columns of interactive markers (small boxes)
        int rows = 5;             // Number of rows
        int cols = 10;            // Number of columns
        double marker_size = 0.2; // Small box size (adjust as needed)

        // Loop through rows and columns to create the markers
        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                createBoxMarker(row, col, marker_size);
            }
        }
    }

    void MTRVizUI::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK)
        {
            toggleCylinderVisibility(feedback->marker_name);
        }
    }

    void MTRVizUI::toggleCylinderVisibility(const std::string &marker_name)
    {
        bool &is_visible = cylinder_visibility_[marker_name];
        is_visible = !is_visible;

        visualization_msgs::msg::InteractiveMarker marker;
        if (marker_server_->get(marker_name, marker))
        {
            marker.controls.clear();

            visualization_msgs::msg::InteractiveMarkerControl control;
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
            control.always_visible = true;

            if (is_visible)
            {
                visualization_msgs::msg::Marker cylinder;
                cylinder.type = visualization_msgs::msg::Marker::CYLINDER;
                cylinder.scale.x = cylinder_radius_ * 2;
                cylinder.scale.y = cylinder_radius_ * 2;
                cylinder.scale.z = cylinder_height_;
                cylinder.color.r = 0.2f;
                cylinder.color.g = 0.8f;
                cylinder.color.b = 0.2f;
                cylinder.color.a = 1.0f;

                control.markers.push_back(cylinder);
            }

            marker.controls.push_back(control);
            marker_server_->insert(marker);
            marker_server_->applyChanges();
        }
    }

    void MTRVizUI::broadcastTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = parent_frame_input_->text().toStdString();
        transform.child_frame_id = child_frame_input_->text().toStdString();
        transform.header.stamp = node_->get_clock()->now();

        transform.transform.translation.x = x_input_->text().toDouble();
        transform.transform.translation.y = y_input_->text().toDouble();
        transform.transform.translation.z = z_input_->text().toDouble();

        // Use a default quaternion with no rotation (identity quaternion)
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, 0.0); // No rotation (identity)
        transform.transform.rotation = tf2::toMsg(quat);

        tf_broadcaster_->sendTransform(transform);
    }

} // namespace rviz_interactive_markers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_interactive_markers::MTRVizUI, rviz_common::Panel)
