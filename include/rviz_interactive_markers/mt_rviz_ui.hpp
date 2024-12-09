#ifndef RVIZ_INTERACTIVE_MARKERS_MT_RVIZ_UI_HPP
#define RVIZ_INTERACTIVE_MARKERS_MT_RVIZ_UI_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.hpp>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <QLineEdit>

#include <map>
#include <string>

namespace rviz_interactive_markers
{

  // Custom RViz Panel for managing interactive markers and broadcasting transforms
  class MTRVizUI : public rviz_common::Panel
  {
    Q_OBJECT

  public:
    explicit MTRVizUI(QWidget *parent = nullptr); // Constructor
    ~MTRVizUI() override;                        // Destructor

    void onInitialize() override; // Initialize panel with default setup

  private Q_SLOTS:
    // Slots for button actions
    void broadcastTransform();           // Broadcast a transform using input values
    void createGrid();                   // Create a grid of interactive markers

  private:
    // Methods for marker management
    void createBoxMarker(int row, int col, double marker_size); // Add a box marker to the grid
    void toggleCylinderVisibility(const std::string &marker_name); // Toggle cylinder visibility on click
    void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback); // Handle marker feedback

    // ROS Node and resources
    rclcpp::Node::SharedPtr node_;                                      // ROS Node for communication
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;    // Broadcast transforms
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_; // Manage interactive markers

    // Cylinder visibility map
    std::map<std::string, bool> cylinder_visibility_; // Tracks marker visibility

    // UI Elements for user inputs
    QLineEdit *child_frame_input_; // Input field for child frame
    QLineEdit *parent_frame_input_; // Input field for parent frame
    QLineEdit *x_input_;            // Input field for X translation
    QLineEdit *y_input_;            // Input field for Y translation
    QLineEdit *z_input_;            // Input field for Z translation
    QLineEdit *roll_input_;         // Input field for roll (optional, unused)
    QLineEdit *pitch_input_;        // Input field for pitch (optional, unused)
    QLineEdit *yaw_input_;          // Input field for yaw (optional, unused)
    QPushButton *broadcast_button_; // Button to broadcast transforms

    // Buttons for marker grid creation
    QPushButton *create_grid_button_;   // Button to create a marker grid
  

    // Marker grid configuration
    double marker_spacing_; // Spacing between markers
    int grid_rows_;         // Number of rows in the grid
    int grid_cols_;         // Number of columns in the grid

    // Cylinder marker parameters
    double cylinder_radius_;  // Radius of the cylinder marker
    double cylinder_height_;  // Height of the cylinder marker
  };

} // namespace rviz_interactive_markers

#endif // RVIZ_INTERACTIVE_MARKERS_MT_RVIZ_UI_HPP