#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <chrono>
#include <functional>
#include <memory>

using ClockMsg = rosgraph_msgs::msg::Clock;
using Clock = std::chrono::steady_clock;

/** \brief Quick clock server implementation to simulate time and publish to /clock topic for other nodes
 * \param first_meas_time  Unix time (seconds since epoch) to start our simulated time at
 * \param playback_rate    Speed multiplier to playback time faster than real time (doesn't currently support slow-mo)
 * \param publish_rate     How often per second our node should publish to the /clock topic
 * */
class ClockServer : public rclcpp::Node {
 public:
  ClockServer() : Node("clock_server") {
    publisher_ = this->create_publisher<ClockMsg>("clock", 10);

    this->declare_parameter("first_meas_time", 1613419600);    // approximately 15c
    this->declare_parameter("playback_rate", 5);
    this->declare_parameter("publish_rate", 20);

    first_meas_time_ = this->get_parameter("first_meas_time").as_int() * (long)1e9;
    playback_rate_ = this->get_parameter("playback_rate").as_int();
    publish_rate_ = this->get_parameter("publish_rate").as_int();
    auto period = std::chrono::milliseconds(1000/publish_rate_);

    timer_ = this->create_wall_timer(period, std::bind(&ClockServer::timer_callback, this));
    start_time_ = Clock::now();
  }

 private:
  void timer_callback() {
    ClockMsg msg;
    auto elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start_time_).count();
    auto sim_time_ns = elapsed_time * playback_rate_ + first_meas_time_;
    msg.clock.set__sec(sim_time_ns / ns_in_s_);
    msg.clock.set__nanosec(sim_time_ns % ns_in_s_);
    publisher_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ClockMsg>::SharedPtr publisher_;
  unsigned long first_meas_time_;
  unsigned int playback_rate_;
  unsigned int publish_rate_;
  std::chrono::time_point<Clock> start_time_;
  const long ns_in_s_ = 1e9;
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockServer>());
  rclcpp::shutdown();
  return 0;
}