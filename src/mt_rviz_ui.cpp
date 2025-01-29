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
#include <QTabWidget>
#include <QFormLayout>
#include <QWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <fstream>
#include <yaml-cpp/yaml.h>
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
        radius_input_ = new QLineEdit(this);
        radius_input_->setPlaceholderText("Enter Cylinder Radius(mm)");
        // Input fields for marker dimensions
        length_input_ = new QLineEdit(this);
        length_input_->setPlaceholderText("Enter Marker Length(mm)");
        width_input_ = new QLineEdit(this);
        width_input_->setPlaceholderText("Enter Marker Width(mm)");

        // Toggle marker button
        toggle_marker_button_ = new QPushButton("Create/Toggle Marker", this);
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
        cylinder_layout->addRow("Radius:", radius_input_);
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
        marker_server_->clear(); // Clear all markers from the interactive marker server
        marker_server_->applyChanges();
        RCLCPP_INFO(node_->get_logger(), "All markers have been reset.");
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

        marker_states_.clear();
        // Initialize marker states (default true= cube)
        marker_states_ = std::vector<std::vector<bool>>(frames.size(), std::vector<bool>(frames.size(), true));
        marker_server_->clear();
        // Create markers for each frame in the list
        for (size_t i = 0; i < frames.size(); ++i)
        {                                        // Cast the index i to int when passing it to createInteractiveMarker
            int marker_id = static_cast<int>(i); // Explicit cast to int
            double x = 0.0;                      // Customize marker positioning as needed
            double y = 0.0;
            double z = 0.0;
            //

            auto int_marker = createInteractiveMarker(marker_id, marker_id, x, y, z, frames[i]);
            RCLCPP_INFO(node_->get_logger(), "Marker Created: Name = %s, Frame = %s",
                        int_marker.name.c_str(), int_marker.header.frame_id.c_str());
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

        // Customize marker orientation
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, 40.0); // Set the desired roll, pitch, and yaw values
        tf2::convert(orientation, int_marker.pose.orientation);

        // Create a marker (cube or cylinder based on state)
        visualization_msgs::msg::Marker marker;
        marker.type = marker_states_[i][j] ? visualization_msgs::msg::Marker::CUBE : visualization_msgs::msg::Marker::CYLINDER;

        // Get the current length and width values from the input fields
        bool length_valid = false, width_valid = false;
        double length = length_input_->text().toDouble(&length_valid);
        double width = width_input_->text().toDouble(&width_valid);

        if (!length_valid || !width_valid || length <= 0 || width <= 0)
        {
            length = 50.0; // Default to 50mm if invalid
            width = 50.0;  // Default to 50mm if invalid
        }

        marker.scale.x = length / 1000.0; // Convert from mm to meters
        marker.scale.y = width / 1000.0;  // Convert from mm to meters
        marker.scale.z = 0.05;            // Set a fixed height for the cube

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
            // Change the marker state to cylinder (no toggling)
            marker_states_[i][j] = false; // false represents cylinder

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
                    marker.type = visualization_msgs::msg::Marker::CYLINDER;
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
        out << YAML::BeginMap; // store key value pairs in a map
        out << YAML::Key << "markers" << YAML::Value << YAML::BeginSeq;
        // marker_states_ is a list of list/(table)where each bool represents the state of a marker (cube or cyl)
        for (size_t i = 0; i < marker_states_.size(); ++i)
        {
            for (size_t j = 0; j < marker_states_[i].size(); ++j)
            {
                std::cout << "i: " << i << ", j: " << j << std::endl;
                std::string marker_name = "marker_" + std::to_string(i) + "_" + std::to_string(j);
                visualization_msgs::msg::InteractiveMarker marker;

                // Retrieve7check the marker by name from the server
                if (marker_server_->get(marker_name, marker))
                {
                    // frame id from marker and store in frame name
                    std::string frame_name = marker.header.frame_id;
                    std::string shape = (marker_states_[i][j] ? "cube" : "cylinder"); // Check the marker state

                    // Add marker properties to YAML
                    out << YAML::BeginMap;
                    out << YAML::Key << "name" << YAML::Value << marker_name;
                    out << YAML::Key << "frame" << YAML::Value << frame_name;
                    out << YAML::Key << "state" << YAML::Value << shape;

                    out << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
                    out << YAML::Key << "x" << YAML::Value << marker.pose.position.x;
                    out << YAML::Key << "y" << YAML::Value << marker.pose.position.y;
                    out << YAML::Key << "z" << YAML::Value << marker.pose.position.z;
                    out << YAML::EndMap;
                    out << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
                    out << YAML::Key << "x" << YAML::Value << marker.pose.orientation.x;
                    out << YAML::Key << "y" << YAML::Value << marker.pose.orientation.y;
                    out << YAML::Key << "z" << YAML::Value << marker.pose.orientation.z;
                    out << YAML::Key << "w" << YAML::Value << marker.pose.orientation.w;
                    out << YAML::EndMap;

                    if (shape == "cube")
                    {
                        // Cube dimensions
                        out << YAML::Key << "dimensions" << YAML::Value << YAML::BeginMap;
                        out << YAML::Key << "length" << YAML::Value << length_input_->text().toDouble();
                        out << YAML::Key << "width" << YAML::Value << width_input_->text().toDouble();
                        out << YAML::EndMap;
                    }
                    else if (shape == "cylinder")
                    {
                        // Cylinder dimensions
                        out << YAML::Key << "dimensions" << YAML::Value << YAML::BeginMap;
                        out << YAML::Key << "radius" << YAML::Value << radius_input_->text().toDouble();
                        out << YAML::Key << "height" << YAML::Value << height_input_->text().toDouble();
                        out << YAML::EndMap;
                    }
                    out << YAML::EndMap;
                }
            }
        }

        out << YAML::EndSeq;
        out << YAML::EndMap;

        // Write the YAML content to the file
        std::ofstream fout(filename);
        fout << out.c_str();
    }
    void MTRVizUI::loadMarkerState(const std::string &filename)
    {
        //  you have saved .yaml file in saveMarkerState() function above
        //   now you should load that file and update the markers accordingly
        try
        {
            YAML::Node node = YAML::LoadFile(filename);

            if (!node["markers"])
            {
                RCLCPP_ERROR(node_->get_logger(), "Invalid file format: 'markers' key not found.");
                return;
            }

            marker_server_->clear(); // Clear existing markers
            marker_states_.clear();  // Clear previous states

            const YAML::Node &markers = node["markers"]; // Get the markers node from loaded YAML
            for (const auto &marker_node : markers)      // Iterate over each marker node
            {
                // Extract the marker properties ,name..etc from the marker node
                std::string marker_name = marker_node["name"].as<std::string>();
                std::string frame_name = marker_node["frame"].as<std::string>();
                std::string shape = marker_node["state"].as<std::string>();

                // Extract numerical values from the marker's name
                int i = std::stoi(marker_name.substr(7, marker_name.find('_') - 7));
                int j = std::stoi(marker_name.substr(marker_name.find('_') + 1));
                // Resize the marker states array if necessary
                if (i >= static_cast<int>(marker_states_.size()))
                {
                    marker_states_.resize(i + 1);
                }
                if (j >= static_cast<int>(marker_states_[i].size()))
                {
                    marker_states_[i].resize(j + 1);
                }

                marker_states_[i][j] = (shape == "cube");

                double x = marker_node["position"]["x"].as<double>(); // Extract marker position
                double y = marker_node["position"]["y"].as<double>();
                double z = marker_node["position"]["z"].as<double>();
                // Create the interactive marker based on the loaded properties
                visualization_msgs::msg::InteractiveMarker int_marker = createInteractiveMarker(i, j, x, y, z, frame_name);

                //  Loop through each control and marker associated with the interactive marker
                for (auto &control : int_marker.controls)
                {
                    for (auto &marker : control.markers)
                    {
                        if (shape == "cube")
                        {
                            marker.type = visualization_msgs::msg::Marker::CUBE;
                            marker.scale.x = marker_node["dimensions"]["length"].as<double>() / 1000.0;
                            marker.scale.y = marker_node["dimensions"]["width"].as<double>() / 1000.0;
                            marker.scale.z = 0.05;
                        }
                        else if (shape == "cylinder")
                        {
                            marker.type = visualization_msgs::msg::Marker::CYLINDER;
                            marker.scale.x = marker_node["dimensions"]["radius"].as<double>() * 2.0 / 1000.0;
                            marker.scale.y = marker_node["dimensions"]["radius"].as<double>() * 2.0 / 1000.0;
                            marker.scale.z = marker_node["dimensions"]["height"].as<double>() / 1000.0;
                        }
                    }
                }

                marker_server_->insert(int_marker);
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
// ff working with   df