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

  class MTRVizUI : public rviz_common::Panel
  {
    Q_OBJECT

  public:
    explicit MTRVizUI(QWidget *parent = nullptr);
    ~MTRVizUI() override;

    void onInitialize() override;

  private Q_SLOTS:
    void broadcastTransform();
    void createGrid();
    void add5x10InteractiveMarkers();

  private:
    // Methods for marker creation and handling
    void createBoxMarker(int row, int col, double marker_size);
    void toggleCylinderVisibility(const std::string &marker_name);
    void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

    // ROS Node
    rclcpp::Node::SharedPtr node_;

    // ROS TF Broadcaster and Marker Server
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;

    // Cylinder visibility map
    std::map<std::string, bool> cylinder_visibility_;

    // UI Elements
    QLineEdit *child_frame_input_;
    QLineEdit *parent_frame_input_;
    QLineEdit *x_input_;
    QLineEdit *y_input_;
    QLineEdit *z_input_;
    QLineEdit *roll_input_;
    QLineEdit *pitch_input_;
    QLineEdit *yaw_input_;
    QPushButton *broadcast_button_;

    // New buttons for grid creation
    QPushButton *create_grid_button_;   // Button to create a grid of markers
    QPushButton *add_5x10_grid_button_; // Button to add 5x10 interactive markers

    // Marker grid configuration
    double marker_spacing_;
    int grid_rows_;
    int grid_cols_;

    // Cylinder parameters
    double cylinder_radius_;
    double cylinder_height_;
  };

} // namespace rviz_interactive_markers

#endif // RVIZ_INTERACTIVE_MARKERS_MT_RVIZ_UI_HPP
