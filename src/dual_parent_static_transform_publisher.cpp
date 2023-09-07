#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

class DualParentTransformPublisher
{
public:
  DualParentTransformPublisher() : nh_(), pnh_("~"), tf_listener_(tf_buf_) {
    namespace rp = ros::param;
    namespace rn = ros::names;
    parent_parent_frame_ = rp::param<std::string>(rn::append("~", "parent_parent_frame"), "world");
    parent_child_frame_  = rp::param<std::string>(rn::append("~", "parent_child_frame"), "odom");
    child_parent_frame_  = rp::param<std::string>(rn::append("~", "child_parent_frame"), "onix");
    child_child_frame_  = rp::param<std::string>(rn::append("~", "child_child_frame"), "body");
    ROS_INFO_STREAM("parent_parent_frame: " << parent_parent_frame_);
    ROS_INFO_STREAM("parent_child_frame: " << parent_child_frame_);
    ROS_INFO_STREAM("child_parent_frame: " << child_parent_frame_);
    ROS_INFO_STREAM("child_child_frame: " << child_child_frame_);
    hz_                  = rp::param<double>(rn::append("~", "hz"), 10.);

    timer_ = nh_.createTimer(ros::Duration(1 / hz_), &DualParentTransformPublisher::timerCallback, this);
  }

 private:
  void timerCallback(const ros::TimerEvent &e) {
    (void) e;
    Eigen::Isometry3d parent_pose, child_pose;
    try {
      child_pose = tf2::transformToEigen(
                    tf_buf_.lookupTransform(parent_child_frame_, child_child_frame_, ros::Time(0)));
      parent_pose = tf2::transformToEigen(
                      tf_buf_.lookupTransform(parent_parent_frame_, child_parent_frame_, ros::Time(0)));
      
      Eigen::Isometry3d diff_pose;
      diff_pose.linear() << child_pose.linear().inverse() * parent_pose.linear();
      diff_pose.translation() << parent_pose.translation() - diff_pose.linear() * child_pose.translation();
      geometry_msgs::TransformStamped trans = tf2::eigenToTransform(diff_pose);
      trans.header.frame_id = parent_parent_frame_;
      trans.header.stamp = ros::Time::now();
      trans.child_frame_id = parent_child_frame_;

      tf_broadcaster_.sendTransform(trans);

    } catch (tf2::LookupException &e) {
      ROS_ERROR("%s", e.what());
    }
  }
 private:
  ros::NodeHandle nh_, pnh_;
  ros::Timer timer_;
  tf2_ros::Buffer tf_buf_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  double hz_;
  std::string parent_parent_frame_, parent_child_frame_, child_parent_frame_, child_child_frame_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dual_parent_transform_publisher");
  DualParentTransformPublisher node;
  ros::spin();

  return 0;
}