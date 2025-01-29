#ifndef RVIZ_INTERACTIVE_MARKERS_MT_RVIZ_UI_HPP
#define RVIZ_INTERACTIVE_MARKERS_MT_RVIZ_UI_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>
#include <rviz_common/panel.hpp>
#include <QPushButton>
#include <QLineEdit>
#include <map>
#include <string>
#include <thread>
#include <unordered_map>
namespace rviz_interactive_markers
{

  // Custom RViz Panel for managing interactive markers and broadcasting transforms
  class MTRVizUI : public rviz_common::Panel
  {
    Q_OBJECT

  public:
    explicit MTRVizUI(QWidget *parent = nullptr); // Constructor
    ~MTRVizUI() override;                         // Destructor

    void onInitialize() override; // Initialize panel with default setup
                                  // Add methods to save and load the marker state
    void saveMarkerState(const std::string &filename);
    void loadMarkerState(const std::string &filename);
  private Q_SLOTS:
    // Slots for button actions
    void broadcastTransform(); // Broadcast a transform using input values
    void toggleMarker();       // Toggle between box and sphere markers
    void resetMarkers();       // Reset all markers in the grid
    void saveMarkerStateDialog();
    void loadMarkerStateDialog();

  private:
    visualization_msgs::msg::InteractiveMarker createInteractiveMarker(int i, int j, double x, double y, double z, const std::string &frame_id); // Create a toggleable interactive marker
    void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);                                    // Handle marker feedback

    // ROS Node and resourcese
    rclcpp::Node::SharedPtr node_;                                                // ROS Node for communication
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;               // Broadcast transforms
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_; // Manage interactive markers
    std::thread spin_thread_;                                                     // Thread for spinning the ROS node

    // UI Elements for user inputs
    QLineEdit *child_frame_input_;      // Input field for child frame
    QLineEdit *parent_frame_input_;     // Input field for parent frame
    QLineEdit *height_input_;           // Input field for cylinder height
    QLineEdit *radius_input_;           // Input field for cylinder radius
    QPushButton *broadcast_button_;     // Button to broadcast transforms
    QPushButton *toggle_marker_button_; // Button to toggle the marker shape
    QLineEdit *length_input_;           // Input field for marker length
    QLineEdit *width_input_;            // Input field for marker width
    QPushButton *reset_button_;         // Button to reset the marker grid

    // Marker toggle state
    std::vector<std::vector<bool>> marker_states_; // Grid of marker states
    bool is_box_;                                  // Tracks whether the marker is currently a box or sphere

  };

} // namespace rviz_interactive_markers

#endif // RVIZ_INTERACTIVE_MARKERS_MT_RVIZ_UI_HPP
       // dfdfd dfgj  hgsd