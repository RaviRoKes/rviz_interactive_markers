#include "rviz_interactive_markers/mt_rviz_ui.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QLabel>
#include <QString>
#include <QPushButton>
#include <QTabWidget>
#include <QFormLayout>
#include <QWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <QMessageBox>

#include <iomanip> // setprecision
#include <sstream>
#include <regex>

// Helper function to format a double to a string with 4 decimal places
std::string formatDouble(double value)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << value;
    return stream.str();
}
// Helper function to format a float to a string with 4 decimal places
std::string formatFloat(float value)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1) << value;
    return stream.str();
}

namespace rviz_interactive_markers
{
    MTRVizUI::MTRVizUI(QWidget *parent)
        : rviz_common::Panel(parent), is_box_(true)
    {
        node_ = std::make_shared<rclcpp::Node>("mt_rviz_ui"); // Initialize ROS 2 node
        marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "interactive_marker_server", node_); // Initialize interactive marker server
        // Input fields for frames
        child_frame_input_ = new QLineEdit(this);
        child_frame_input_->setPlaceholderText("Enter Child Frame Name");
        parent_frame_input_ = new QLineEdit(this);
        parent_frame_input_->setPlaceholderText("Enter Parent Frame Name");

        // Broadcast button
        broadcast_button_ = new QPushButton("Broadcast Transform", this);
        connect(broadcast_button_, &QPushButton::clicked, this, &MTRVizUI::broadcastTransform);
        // Input fields for cylinder dimensions
        height_input_ = new QLineEdit(this);
        height_input_->setPlaceholderText("Enter Cylinder Height(mm)");
        diameter_input_ = new QLineEdit(this);
        diameter_input_->setPlaceholderText("Enter Cylinder Diameter(mm)");
        // Input fields for marker dimensions
        length_input_ = new QLineEdit(this);
        length_input_->setPlaceholderText("Enter Marker Length(mm)");
        width_input_ = new QLineEdit(this);
        width_input_->setPlaceholderText("Enter Marker Width(mm)");

        // Toggle marker button
        toggle_marker_button_ = new QPushButton("Create Marker", this);
        connect(toggle_marker_button_, &QPushButton::clicked, this, &MTRVizUI::toggleMarker);
        // Reset button
        reset_button_ = new QPushButton("Reset Markers", this);
        connect(reset_button_, &QPushButton::clicked, this, &MTRVizUI::resetMarkers);

        // Create the Save and Load buttons
        QPushButton *save_button_ = new QPushButton("Save Marker State", this);
        QPushButton *load_button_ = new QPushButton("Load Marker State", this);

        // Connect button clicks to the respective functions
        connect(save_button_, &QPushButton::clicked, this, &MTRVizUI::saveMarkerStateDialog);
        connect(load_button_, &QPushButton::clicked, this, &MTRVizUI::loadMarkerStateDialog);

        // Frames Tab
        QWidget *frames_tab = new QWidget(this);
        QFormLayout *frame_layout = new QFormLayout;
        frame_layout->addRow("Child Frame:", child_frame_input_);
        frame_layout->addRow("Parent Frame:", parent_frame_input_);
        frame_layout->addRow(broadcast_button_);
        frames_tab->setLayout(frame_layout);

        // Cylinder Dimensions Tab
        QWidget *cylinder_tab = new QWidget(this);
        QFormLayout *cylinder_layout = new QFormLayout;
        cylinder_layout->addRow("Height:", height_input_);
        cylinder_layout->addRow("Diameter:", diameter_input_);
        cylinder_tab->setLayout(cylinder_layout);

        // Additional tab example (optional, e.g., Marker Controls)
        QWidget *marker_tab = new QWidget(this);
        QFormLayout *marker_layout = new QFormLayout;
        marker_layout->addRow("Length:", length_input_);
        marker_layout->addRow("Width:", width_input_);
        marker_layout->addWidget(toggle_marker_button_);
        marker_tab->setLayout(marker_layout);

        // Create the Tab Widget
        QTabWidget *tab_widget = new QTabWidget(this);
        tab_widget->addTab(frames_tab, "Frames");
        tab_widget->addTab(cylinder_tab, "Cylinder Dimensions(mm)");
        tab_widget->addTab(marker_tab, "Marker Controls");

        // Main layout
        QVBoxLayout *main_layout = new QVBoxLayout;
        main_layout->addWidget(tab_widget);
        main_layout->addWidget(reset_button_);
        main_layout->addWidget(save_button_);
        main_layout->addWidget(load_button_);
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
    void MTRVizUI::resetMarkers()
    {
        // Show a confirmation dialog
        QMessageBox::StandardButton reply = QMessageBox::question(
            nullptr,
            "Reset Markers",
            "Are you sure you want to reset all markers? This action cannot be undone.",
            QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes)
        {
            marker_server_->clear(); // Clear all markers from the interactive marker server
            marker_server_->applyChanges();
            RCLCPP_INFO(node_->get_logger(), "All markers have been reset.");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Marker reset canceled by the user.");
        }
    }
    void MTRVizUI::toggleMarker()
    {
        // Initialize the TF buffer and listener
        tf2_ros::Buffer tf_buffer(node_->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);
        rclcpp::sleep_for(std::chrono::milliseconds(100)); // small delay to allow TF data to be populated
        std::vector<std::string> all_frames;               // Variable to store all frames

        try
        {
            std::string frames_yaml = tf_buffer.allFramesAsYAML(); // Get all frames from tf tree as a YAML string
            std::stringstream ss(frames_yaml);
            std::string line;
            RCLCPP_INFO(node_->get_logger(), "All Frames:\n%s", frames_yaml.c_str());
            // Parse each line to extract frame IDs
            while (std::getline(ss, line))
            {
                // Each frame is listed as "<frame>: { ... }"
                size_t colon_pos = line.find(':');
                if (colon_pos != std::string::npos)
                {
                    all_frames.push_back(line.substr(0, colon_pos));
                }
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get frames from TF: %s", ex.what());
            return;
        }

        // Filter frames using regex
        std::regex frame_regex(R"(b_mt_store_[0-9]_[a-z][0-9])");

        std::vector<std::string> filtered_frames;
        for (const auto &frame : all_frames)
        {
            RCLCPP_INFO(node_->get_logger(), "- %s", frame.c_str()); // Log each input frame
            if (std::regex_match(frame, frame_regex))
            {
                filtered_frames.push_back(frame);
            }
        }
        // Log all frame names
        RCLCPP_INFO(node_->get_logger(), "Filtered frames:");
        for (const auto &frame : filtered_frames)
        {
            RCLCPP_INFO(node_->get_logger(), "- %s", frame.c_str());
        }

        marker_states_.clear();

        // initial marker state to 'true' for CUBE
        marker_states_ = std::vector<std::vector<bool>>(filtered_frames.size(), std::vector<bool>(filtered_frames.size(), true));
        marker_server_->clear();

        // Created interactive markers for each filtered frame
        for (size_t i = 0; i < filtered_frames.size(); ++i)
        {
            int marker_id = static_cast<int>(i);
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;

            // Create marker for the filtered frame
            auto int_marker = createInteractiveMarker(marker_id, marker_id, x, y, z, filtered_frames[i]);
            RCLCPP_INFO(node_->get_logger(), "Marker Created: Name = %s, Frame = %s",
                        int_marker.name.c_str(), int_marker.header.frame_id.c_str());

            marker_server_->insert(int_marker, std::bind(&MTRVizUI::processFeedback, this, std::placeholders::_1));
        }
        marker_server_->applyChanges();
        RCLCPP_INFO(node_->get_logger(), "Created markers on filtered frames.");
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

        // Customize marker orientation
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, 0.78);
        tf2::convert(orientation, int_marker.pose.orientation);

        // Create a marker (cube or cylinder based on state)
        visualization_msgs::msg::Marker marker;
        marker.type = marker_states_[i][j] ? visualization_msgs::msg::Marker::CUBE : visualization_msgs::msg::Marker::CYLINDER;

        // Get length and width from the input fields
        bool length_valid = false, width_valid = false;
        double length = length_input_->text().toDouble(&length_valid);
        double width = width_input_->text().toDouble(&width_valid);

        if (!length_valid || !width_valid || length <= 0 || width <= 0)
        {
            length = 50.0;
            width = 50.0; // Default width
        }

        marker.scale.x = length / 1000.0; // Convert from mm to meters
        marker.scale.y = width / 1000.0;
        marker.scale.z = 0.05; // Set a fixed height
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
            // Process only BUTTON_CLICK events
            if (feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK)
            {
                return;
            }

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

            // toggling between cube and cylinder...
            double original_x = 0.0; // Initial position of the cube (0,0,0)
            double original_y = 0.0;
            double original_z = 0.0;

            if (marker_states_[i][j]) // true represents cube
            {
                marker_states_[i][j] = false;

                // Retrieve user inputs (height and diameter)
                bool height_valid = false, diameter_valid = false;
                double height = height_input_->text().toDouble(&height_valid);
                double diameter = diameter_input_->text().toDouble(&diameter_valid);

                if (!height_valid || !diameter_valid || height <= 0 || diameter <= 0)
                {
                    QMessageBox::warning(nullptr, "Invalid Input", "Please enter valid height and diameter values greater than 0.");
                    RCLCPP_ERROR(node_->get_logger(), "Invalid dimensions provided: height=%.2f, diameter=%.2f", height, diameter);
                    return;
                }

                // shifting of cylinder
                const double L = 62.5; // mm
                double radius = diameter / 2.0;
                double shift = std::sqrt(2 * std::pow(L - radius, 2)) / 1000.0; // mm to meters

                // Update the new position
                double new_x = feedback->pose.position.x; // Move along the x-axis
                double new_y = feedback->pose.position.y - shift;
                // Create the interactive marker for the cylinder
                visualization_msgs::msg::InteractiveMarker int_marker = createInteractiveMarker(i, j, new_x, new_y, original_z, feedback->header.frame_id);

                for (auto &control : int_marker.controls)
                {
                    for (auto &marker : control.markers)
                    {
                        marker.type = visualization_msgs::msg::Marker::CYLINDER;
                        marker.scale.x = diameter / 1000.0; // mm to meters
                        marker.scale.y = diameter / 1000.0;
                        marker.scale.z = height / 1000.0;
                        marker.color.r = 1.0; // red
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                        marker.color.a = 1.0;
                    }
                }

                // Update the marker in the server and apply changes
                marker_server_->insert(int_marker);
                marker_server_->applyChanges();

                RCLCPP_INFO(node_->get_logger(), "Marker updated to Cylinder: i=%d, j=%d, new_x=%.2f, new_y=%.2f, diameter=%.2fmm", i, j, new_x, new_y, diameter);
            }
            else // If cylinder, change to cube
            {

                marker_states_[i][j] = true; // true represents cube

                // Create the interactive marker for the cube, restoring its original position at (0, 0, 0)
                visualization_msgs::msg::InteractiveMarker int_marker = createInteractiveMarker(i, j, original_x, original_y, original_z, feedback->header.frame_id);

                for (auto &control : int_marker.controls)
                {
                    for (auto &marker : control.markers)
                    {
                        // marker.type = visualization_msgs::msg::Marker::CUBE;
                        // marker.scale.x = 0.1; // Default cube size (can adjust as needed)
                        // marker.scale.y = 0.1;
                        // marker.scale.z = 0.05;
                        // marker.color.r = 0.0; // blue
                        // marker.color.g = 0.0;
                        // marker.color.b = 1.0;
                        // marker.color.a = 1.0;
                        // Set marker position

                        // orientation
                        tf2::Quaternion orientation;
                        orientation.setRPY(0.0, 0.0, 0.78);
                        tf2::convert(orientation, int_marker.pose.orientation);

                        // Get length and width from the input fields
                        bool length_valid = false, width_valid = false;
                        double length = length_input_->text().toDouble(&length_valid);
                        double width = width_input_->text().toDouble(&width_valid);

                        if (!length_valid || !width_valid || length <= 0 || width <= 0)
                        {
                            length = 50.0;
                            width = 50.0; // Default width
                        }

                        marker.scale.x = length / 1000.0; // Convert from mm to meters
                        marker.scale.y = width / 1000.0;
                        marker.scale.z = 0.05; // Set a fixed height
                        marker.color.r = 0.0;
                        marker.color.g = 0.0;
                        marker.color.b = 1.0;
                        marker.color.a = 1.0;
                    }
                }

                // Update the marker in the server and apply changes
                marker_server_->insert(int_marker);
                marker_server_->applyChanges();

                RCLCPP_INFO(node_->get_logger(), "Marker Cube: i=%d, j=%d, original_x=%.2f, original_y=%.2f", i, j, original_x, original_y);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Exception in processFeedback: %s", e.what());
        }
    }

    void MTRVizUI::saveMarkerStateDialog()
    {
        // Open a file dialog to select the save file
        QString filename = QFileDialog::getSaveFileName(
            this, "Save Marker State", QDir::homePath(), "YAML Files (*.yaml)");

        if (!filename.isEmpty())
        {
            saveMarkerState(filename.toStdString());
            QMessageBox::information(this, "Save Successful", "Marker state saved successfully!");
        }
        else
        {
            QMessageBox::warning(this, "Save Failed", "No file selected. Marker state was not saved.");
        }
    }
    void MTRVizUI::loadMarkerStateDialog()
    {
        // Open a file dialog to select the load file
        QString filename = QFileDialog::getOpenFileName(
            this, "Load Marker State", QDir::homePath(), "YAML Files (*.yaml)");

        if (!filename.isEmpty())
        {
            loadMarkerState(filename.toStdString());
            QMessageBox::information(this, "Load Successful", "Marker state loaded successfully!");
        }
        else
        {
            QMessageBox::warning(this, "Load Failed", "No file selected. Marker state was not loaded.");
        }
    }

    void MTRVizUI::saveMarkerState(const std::string &filename)
    {
        YAML::Emitter out;
        out << YAML::BeginMap;

        // Add the SolidPrimitive Msg comments
        out << YAML::Comment("SolidPrimitive Msg") << YAML::Newline << YAML::Newline;
        out << YAML::Comment("# Defines box, sphere, cylinder, cone and prism.");
        out << YAML::Newline;
        out << YAML::Comment("# All shapes are defined to have their bounding boxes centered around 0,0,0.");
        out << YAML::Newline << YAML::Newline;
        out << YAML::Comment("uint8 BOX=1") << YAML::Newline;
        out << YAML::Comment("uint8 SPHERE=2") << YAML::Newline;
        out << YAML::Comment("uint8 CYLINDER=3") << YAML::Newline;
        out << YAML::Comment("uint8 CONE=4") << YAML::Newline;
        out << YAML::Comment("uint8 PRISM=5") << YAML::Newline << YAML::Newline;
        out << YAML::Comment("# The type of the shape") << YAML::Newline;
        out << YAML::Comment("uint8 type") << YAML::Newline << YAML::Newline;
        out << YAML::Comment("# The dimensions of the shape") << YAML::Newline;
        out << YAML::Comment("float64[<=3] dimensions  # At no point will dimensions have a length > 3.");
        out << YAML::Newline;
        out << YAML::Newline;
        out << YAML::Comment("# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array.");
        out << YAML::Newline;
        out << YAML::Newline;
        out << YAML::Comment("# For type BOX, the X, Y, and Z dimensions are the length of the corresponding sides of the box.");
        out << YAML::Newline;
        out << YAML::Comment("uint8 BOX_X=0") << YAML::Newline;
        out << YAML::Comment("uint8 BOX_Y=1") << YAML::Newline;
        out << YAML::Comment("uint8 BOX_Z=2") << YAML::Newline << YAML::Newline;
        out << YAML::Comment("# For the SPHERE type, only one component is used, and it gives the radius of the sphere.");
        out << YAML::Newline;
        out << YAML::Comment("uint8 SPHERE_RADIUS=0") << YAML::Newline << YAML::Newline;
        out << YAML::Comment("# For the CYLINDER and CONE types, the center line is oriented along the Z axis.");
        out << YAML::Newline;
        out << YAML::Comment("# Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component of dimensions gives the");
        out << YAML::Newline;
        out << YAML::Comment("# height of the cylinder (cone).");
        out << YAML::Newline;
        out << YAML::Comment("# The CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the radius of");
        out << YAML::Newline;
        out << YAML::Comment("# the base of the cylinder (cone).");
        out << YAML::Newline;
        out << YAML::Comment("# Cone and cylinder primitives are defined to be circular. The tip of the cone");
        out << YAML::Newline;
        out << YAML::Comment("# is pointing up, along +Z axis.") << YAML::Newline << YAML::Newline;
        out << YAML::Comment("uint8 CYLINDER_HEIGHT=0") << YAML::Newline;
        out << YAML::Comment("uint8 CYLINDER_RADIUS=1") << YAML::Newline << YAML::Newline;
        out << YAML::Comment("uint8 CONE_HEIGHT=0") << YAML::Newline;
        out << YAML::Comment("uint8 CONE_RADIUS=1") << YAML::Newline << YAML::Newline;
        out << YAML::Comment("# For the type PRISM, the center line is oriented along Z axis.");
        out << YAML::Newline;
        out << YAML::Comment("# The PRISM_HEIGHT component of dimensions gives the");
        out << YAML::Newline;
        out << YAML::Comment("# height of the prism.") << YAML::Newline;
        out << YAML::Comment("# The polygon defines the Z axis centered base of the prism.");
        out << YAML::Newline;
        out << YAML::Comment("# The prism is constructed by extruding the base in +Z and -Z");
        out << YAML::Newline;
        out << YAML::Comment("# directions by half of the PRISM_HEIGHT");
        out << YAML::Newline;
        out << YAML::Comment("# Only x and y fields of the points are used in the polygon.");
        out << YAML::Newline;
        out << YAML::Comment("# Points of the polygon are ordered counter-clockwise.");
        out << YAML::Newline;
        out << YAML::Newline;
        out << YAML::Comment("uint8 PRISM_HEIGHT=0") << YAML::Newline;
        out << YAML::Comment("geometry_msgs/Polygon polygon") << YAML::Newline << YAML::Newline << YAML::Newline;

        // Add workspace_manager and object_properties
        out << YAML::Key << "workspace_manager";
        out << YAML::Value << YAML::BeginMap;

        out << YAML::Key << "object_properties";
        out << YAML::Value << YAML::BeginMap;

        // Add core_properties
        out << YAML::Key << "core_properties";
        out << YAML::Value << YAML::Anchor("core") << YAML::BeginMap;
        out << YAML::Key << "id" << YAML::Value << 0;
        out << YAML::Key << "parent"
            << YAML::Value << 0
            << YAML::Comment("0 always refers to the workspace, Parent is the id of the parent_object");

        out << YAML::Key << "pos"
            << YAML::Value << YAML::Flow
            << YAML::BeginSeq << formatFloat(0.0) << formatFloat(0.0) << formatFloat(0.0)
            << YAML::EndSeq
            << YAML::Comment("Defined in the parents base position and rotation");
        out << YAML::Key << "rotation"
            << YAML::Value << YAML::Flow
            << YAML::BeginSeq << formatFloat(0.0) << formatFloat(0.0) << formatFloat(0.0)
            << YAML::EndSeq
            << YAML::Comment("Defined in the parents base position and rotation, Defined in RPY");

        out << YAML::Key << "frame" << YAML::Value << "" << YAML::Comment("If left empty, will take the workspace_frame");
        out << YAML::EndMap; // Close core_properties

        // Add collision_properties
        out << YAML::Key << "collision_properties";
        out << YAML::Value << YAML::Anchor("collision") << YAML::BeginMap;
        out << YAML::Key << "shape";
        out << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "shape_pose"
            << YAML::Value << YAML::Flow << YAML::BeginSeq
            << formatFloat(0.0) << formatFloat(0.0) << formatFloat(0.0)
            << YAML::EndSeq << YAML::Comment("Defined in the instance's base position and rotation");
        out << YAML::Key << "shape_rotation"
            << YAML::Value << YAML::Flow << YAML::BeginSeq
            << formatFloat(0.0) << formatFloat(0.0) << formatFloat(0.0)
            << YAML::EndSeq << YAML::Comment("Defined in the instance's base position and rotation");
        out << YAML::Key << "mesh";
        out << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "package" << YAML::Value << "";
        out << YAML::Key << "path" << YAML::Value << "";
        out << YAML::Key << "scale"
            << YAML::Value << YAML::Flow << YAML::BeginSeq
            << formatFloat(0.0) << formatFloat(0.0) << formatFloat(0.0)
            << YAML::EndSeq << YAML::Comment("Doubles as length, width, height");
        out << YAML::EndMap; // Close mesh
        out << YAML::Key << "primitive";
        out << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "type" << YAML::Value << 0 << YAML::Comment("uint8 value according to shape_msgs/msg/SolidPrimitive. e.g., \"type: 3  # shape_msgs::msg::SolidPrimitive::CYLINDER\"");
        out << YAML::Key << "dimensions" << YAML::Value << YAML::Flow << YAML::BeginSeq << YAML::EndSeq;
        out << YAML::EndMap; // Close primitive
        out << YAML::EndMap; // Close shape
        out << YAML::EndMap; // Close collision_properties
        out << YAML::EndMap;
        // Add object_types
        out << YAML::Key << "object_types";
        out << YAML::Value << YAML::BeginMap;

        out << YAML::Key << "workspace";
        out << YAML::Value << YAML::Anchor("workspace") << YAML::BeginMap;
        out << YAML::Key << "id" << YAML::Value << 0;
        out << YAML::Key << "base_frame" << YAML::Value << "";
        out << YAML::Key << "predicates" << YAML::Value << YAML::Null;
        out << YAML::EndMap; // Close workspace

        out << YAML::Key << "goal";
        out << YAML::Value << YAML::Anchor("goal") << YAML::BeginMap;
        out << YAML::Key << "object_type" << YAML::Value << 0;
        out << YAML::Key << "<<" << YAML::Value << YAML::Alias("core");
        out << YAML::Key << "interaction_point" << YAML::Value << YAML::Flow << YAML::BeginSeq << formatFloat(0.0) << formatFloat(0.0) << formatFloat(0.0) << YAML::EndSeq << YAML::Comment("The point where to place the gripper");
        out << YAML::Key << "interaction_approach_direction" << YAML::Value << YAML::Flow << YAML::BeginSeq << formatFloat(0.0) << formatFloat(0.0) << formatFloat(0.0) << YAML::EndSeq << YAML::Comment("The direction from where to approach the object");
        out << YAML::Key << "approach_distance" << YAML::Value << formatFloat(0.1f);
        out << YAML::EndMap; // Close goal

        out << YAML::Key << "cluster";
        out << YAML::Value << YAML::Anchor("storage") << YAML::BeginMap;
        out << YAML::Key << "object_type" << YAML::Value << 2;
        out << YAML::Key << "<<" << YAML::Value << YAML::Alias("core");
        out << YAML::Key << "cluster_type" << YAML::Value << -1 << YAML::Comment("If this only contains specific types, give it another value, otherwise, it can contain anything arbitrary");
        out << YAML::Key << "cluster_storage" << YAML::Value << YAML::Flow << YAML::BeginSeq << YAML::EndSeq << YAML::Comment("contains the id numbers of each stored instance");
        out << YAML::EndMap; // Close cluster

        out << YAML::Key << "object_instance";
        out << YAML::Value << YAML::Anchor("instance") << YAML::BeginMap;
        out << YAML::Key << "object_type" << YAML::Value << 1;
        out << YAML::Key << "<<" << YAML::Value << YAML::Alias("core");
        out << YAML::Key << "id" << YAML::Value << 0;
        out << YAML::Key << "parent" << YAML::Value << 0;
        out << YAML::Key << "interaction_point" << YAML::Value << YAML::Flow << YAML::BeginSeq << formatFloat(0.0) << formatFloat(0.0) << formatFloat(0.0) << YAML::EndSeq;
        out << YAML::Key << "interaction_approach_direction" << YAML::Value << YAML::Flow << YAML::BeginSeq << formatFloat(0.0) << formatFloat(0.0) << formatFloat(0.0) << YAML::EndSeq << YAML::Comment("RPY");
        out << YAML::Key << "approach_distance" << YAML::Value << formatFloat(0.1f);
        out << YAML::Key << "<<" << YAML::Value << YAML::Alias("collision");
        out << YAML::EndMap; // Close object_instance

        out << YAML::EndMap; // Close object_types
        out << YAML::EndMap; // Close workspace_manager

        out << YAML::Key << "workspace_object" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "<<" << YAML::Value << YAML::Alias("workspace");
        out << YAML::Key << "base_frame" << YAML::Value << "base_link";

        // Add objects
        out << YAML::Key << "objects";
        out << YAML::Value << YAML::BeginMap;

        // out << YAML::Comment("Example of a mesh shape object");
        // out << YAML::Key << "part_1";
        // out << YAML::Value << YAML::BeginMap;
        // out << YAML::Key << "<<" << YAML::Value << YAML::Alias("instance");
        // out << YAML::Key << "frame" << YAML::Value << "_1positioner_1";
        // out << YAML::Key << "pos" << YAML::Value << YAML::Flow << YAML::BeginSeq << formatFloat(0.0) << formatFloat(0.0) << formatDouble(0.001) << YAML::EndSeq;
        // out << YAML::Key << "rotation" << YAML::Value << YAML::Flow << YAML::BeginSeq << formatFloat(0.0) << formatFloat(0.0) << formatFloat(0.0) << YAML::EndSeq;
        // out << YAML::Key << "approach_distance" << YAML::Value << 0.2;
        // out << YAML::Key << "shape" << YAML::Value << YAML::BeginMap;
        // out << YAML::Key << "shape_pose" << YAML::Value << YAML::Flow << YAML::BeginSeq << formatFloat(0.0) << formatFloat(0.0) << 0.0 << YAML::EndSeq;
        // out << YAML::Key << "shape_rotation" << YAML::Value << YAML::Flow << YAML::BeginSeq << formatFloat(0.0) << formatFloat(0.0) << formatFloat(0.0) << YAML::EndSeq;
        // out << YAML::Key << "mesh" << YAML::Value << YAML::BeginMap;
        // out << YAML::Key << "package" << YAML::Value << "template_mt_cell_configuration";
        // out << YAML::Key << "path" << YAML::Value << "meshes/plates/visual/NSE_138_2.stl";
        // out << YAML::Key << "scale" << YAML::Value << YAML::Flow << YAML::BeginSeq << formatDouble(0.001) << formatDouble(0.001) << formatDouble(0.001) << YAML::EndSeq;
        // out << YAML::EndMap; // Close mesh
        // out << YAML::EndMap; // Close shape
        // out << YAML::Key << "id" << YAML::Value << 1;
        // out << YAML::Key << "interaction_point" << YAML::Value << YAML::Flow << YAML::BeginSeq << formatFloat(-0.06f) << formatFloat(0.0) << formatFloat(0.12f) << YAML::EndSeq;
        // out << YAML::Key << "interaction_approach_direction" << YAML::Value << YAML::Flow << YAML::BeginSeq << formatFloat(0.0) << formatFloat(-0.5f) << formatFloat(1.0f) << YAML::EndSeq;
        // out << YAML::EndMap; // Close part_1

        for (size_t i = 0; i < marker_states_.size(); ++i)
        {
            for (size_t j = 0; j < marker_states_[i].size(); ++j)
            {

                std::string marker_name = "marker_" + std::to_string(i) + "_" + std::to_string(j);
                visualization_msgs::msg::InteractiveMarker marker;

                if (marker_server_->get(marker_name, marker))
                {
                    std::string frame_name = marker.header.frame_id;
                    uint8_t shape_type = marker_states_[i][j] ? shape_msgs::msg::SolidPrimitive::BOX : shape_msgs::msg::SolidPrimitive::CYLINDER;

                    // Add named part (e.g., part_1, part_2, etc.)
                    std::string part_name = "part_" + std::to_string(i + 1); // Adjust naming as needed
                    out << YAML::Key << part_name;
                    out << YAML::Value << YAML::BeginMap;
                    out << YAML::Key << "<<" << YAML::Value << YAML::Alias("instance");

                    // Add frame, position, rotation, etc.
                    out << YAML::Key << "frame" << YAML::Value << YAML::DoubleQuoted << frame_name;
                    out << YAML::Key << "pos" << YAML::Value << YAML::Flow << YAML::BeginSeq << 0.0 << 0.0 << 0.001 << YAML::EndSeq;
                    out << YAML::Key << "rotation" << YAML::Value << YAML::Flow << YAML::BeginSeq << 0.0 << 0.0 << 0.0 << YAML::EndSeq;
                    out << YAML::Key << "approach_distance" << YAML::Value << formatFloat(0.2f);

                    // Add shape info
                    out << YAML::Key << "shape" << YAML::Value << YAML::BeginMap;
                    out << YAML::Key << "shape_pose" << YAML::Value << YAML::Flow << YAML::BeginSeq
                        << formatDouble(marker.pose.position.x)
                        << formatDouble(marker.pose.position.y)
                        << formatDouble(marker.pose.position.z)
                        << YAML::EndSeq;

                    tf2::Quaternion q(marker.pose.orientation.x, marker.pose.orientation.y,
                                      marker.pose.orientation.z, marker.pose.orientation.w);
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

                    out << YAML::Key << "shape_rotation" << YAML::Value << YAML::Flow
                        << YAML::BeginSeq
                        << formatDouble(roll)
                        << formatDouble(pitch)
                        << formatDouble(yaw)
                        << YAML::EndSeq;

                    out << YAML::Key << "primitive" << YAML::Value << YAML::BeginMap;
                    out << YAML::Key << "type" << YAML::Value << static_cast<int>(shape_type);
                    if (shape_type == shape_msgs::msg::SolidPrimitive::BOX)
                    {
                        out << YAML::Comment(" shape_msgs::msg::SolidPrimitive::BOX");
                    }
                    else if (shape_type == shape_msgs::msg::SolidPrimitive::CYLINDER)
                    {
                        out << YAML::Comment(" shape_msgs::msg::SolidPrimitive::CYLINDER");
                    }
                    out << YAML::Key << "dimensions" << YAML::Value << YAML::Flow << YAML::BeginSeq;
                    if (shape_type == shape_msgs::msg::SolidPrimitive::BOX)
                    {
                        out << formatDouble(length_input_->text().toDouble())
                            << formatDouble(width_input_->text().toDouble())
                            << formatDouble(0.05); // Fixed height for cube
                    }
                    else if (shape_type == shape_msgs::msg::SolidPrimitive::CYLINDER)
                    {
                        out << formatDouble(height_input_->text().toDouble())
                            << formatDouble(diameter_input_->text().toDouble());
                    }
                    out << YAML::EndSeq;
                    out << YAML::EndMap; // Close primitive
                    out << YAML::EndMap; // Close shape

                    // Add marker ID
                    //  out << YAML::Key << "id" << YAML::Value << static_cast<int>(i);
                    out << YAML::Key << "id" << YAML::Value << marker_name;
                    // Add interaction point
                    out << YAML::Key << "interaction_point" << YAML::Value << YAML::Flow << YAML::BeginSeq
                        << formatDouble(-0.05) << formatDouble(0.0) << formatDouble(0.15)
                        << YAML::EndSeq;
                    // Add interaction approach direction
                    out << YAML::Key << "interaction_approach_direction" << YAML::Value << YAML::Flow << YAML::BeginSeq
                        << formatDouble(0.0) << formatDouble(-0.5) << formatDouble(1.0)
                        << YAML::EndSeq;
                    out << YAML::EndMap; // Close part entry
                }
            }
        }

        out << YAML::EndMap; // Close objects
        out << YAML::EndMap; // Close workspace_object
        out << YAML::EndMap; // Close root

        std::ofstream fout(filename);
        fout << out.c_str();
    }

    void MTRVizUI::loadMarkerState(const std::string &filename)
    {
        try
        {
            YAML::Node node = YAML::LoadFile(filename);

            if (!node["workspace_object"] || !node["workspace_object"]["objects"])
            {
                RCLCPP_ERROR(node_->get_logger(), "Invalid file format: 'objects' key not found.");
                return;
            }

            marker_server_->clear();
            marker_states_.clear();

            const YAML::Node &objects = node["workspace_object"]["objects"];
            size_t i = 0;

            for (const auto &object : objects)
            {
                std::string frame_name = object.second["frame"].as<std::string>(); // Extract frame name

                // Extract position
                const YAML::Node &position = object.second["shape"]["shape_pose"];
                double x = position[0].as<double>();
                double y = position[1].as<double>();
                double z = position[2].as<double>();

                // Extract orientation
                const YAML::Node &orientation = object.second["shape"]["shape_rotation"];
                double roll = orientation[0].as<double>();
                double pitch = orientation[1].as<double>();
                double yaw = orientation[2].as<double>();

                uint8_t shape_type = object.second["shape"]["primitive"]["type"].as<uint8_t>(); // Extract shape info

                //  list of boxes (marker_states_) and its type box/cy
                // checks if the list marker_states_ has enough space to store information for the current marker (i).
                if (i >= marker_states_.size())
                    marker_states_.resize(i + 1);

                marker_states_[i].resize(1);
                // store shape type
                marker_states_[i][0] = (shape_type == shape_msgs::msg::SolidPrimitive::BOX);

                // Extract dimensions
                const YAML::Node &dimensions = object.second["shape"]["primitive"]["dimensions"];
                double dim1 = dimensions[0].as<double>();
                double dim2 = dimensions[1].as<double>();

                // Convert roll, pitch, yaw to quaternion
                tf2::Quaternion q;
                q.setRPY(roll, pitch, yaw);

                // Create InteractiveMarker
                visualization_msgs::msg::InteractiveMarker int_marker = createInteractiveMarker(static_cast<int>(i), 0, x, y, z, frame_name);
                int_marker.pose.orientation.x = q.x();
                int_marker.pose.orientation.y = q.y();
                int_marker.pose.orientation.z = q.z();
                int_marker.pose.orientation.w = q.w();

                // Set marker shape (cube or cylinder)
                for (auto &control : int_marker.controls)
                {
                    for (auto &marker : control.markers)
                    {
                        if (shape_type == shape_msgs::msg::SolidPrimitive::BOX)
                        {
                            marker.type = visualization_msgs::msg::Marker::CUBE;
                            marker.scale.x = dim1 / 1000.0;
                            marker.scale.y = dim2 / 1000.0;
                            marker.scale.z = 0.05; // Fixed height for cube
                        }
                        else if (shape_type == shape_msgs::msg::SolidPrimitive::CYLINDER)
                        {
                            marker.type = visualization_msgs::msg::Marker::CYLINDER;
                            marker.scale.x = dim2 / 1000.0;
                            marker.scale.y = dim2 / 1000.0;
                            marker.scale.z = dim1 / 1000.0;
                        }
                    }
                }

                // Insert into marker server
                marker_server_->insert(int_marker);
                i++;
            }

            marker_server_->applyChanges();
            RCLCPP_INFO(node_->get_logger(), "Marker state loaded from file: %s", filename.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Exception in loadMarkerState: %s", e.what());
        }
    }

    void MTRVizUI::broadcastTransform()
    {
        std::string parent_frame = parent_frame_input_->text().toStdString();
        std::string child_frame = child_frame_input_->text().toStdString();

        // Check if either frame name is empty or contains spaces
        if (parent_frame.empty() || child_frame.empty() || parent_frame.find(" ") != std::string::npos || child_frame.find(" ") != std::string::npos)
        {
            QMessageBox::warning(this, "Invalid Frame Names", "Parent or Child frame is empty or contains spaces.");
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
// working before mesh