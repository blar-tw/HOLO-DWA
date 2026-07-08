// replay_pose.cpp - stream model poses into a running gz sim world.
//
// Reads lines "t x y z yaw" (gz world frame, seconds/meters/radians) from
// stdin and calls /world/<world>/set_pose on a single persistent
// gz-transport node, pacing wall time by the t column (divided by speed).
// One in-process node keeps the per-call latency at the millisecond level,
// where spawning `gz service` per pose costs ~280 ms - far too slow for a
// 20 Hz trajectory replay.
//
// Built and driven by replay_demo.py; not meant to be run by hand.
//
//   g++ -O2 replay_pose.cpp -o replay_pose \
//       $(pkg-config --cflags --libs gz-transport12 gz-msgs9)
#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/boolean.pb.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: replay_pose <world> <model> [speed]" << std::endl;
    return 1;
  }
  const std::string world = argv[1];
  const std::string model = argv[2];
  const double speed = argc > 3 ? std::stod(argv[3]) : 1.0;

  gz::transport::Node node;
  const std::string srv = "/world/" + world + "/set_pose";

  double t_prev = -1.0;
  auto wall_next = std::chrono::steady_clock::now();
  std::string line;
  long count = 0, failed = 0;

  while (std::getline(std::cin, line)) {
    std::istringstream ss(line);
    double t, x, y, z, yaw;
    if (!(ss >> t >> x >> y >> z >> yaw))
      continue;

    if (t_prev >= 0.0) {
      const double dt = (t - t_prev) / speed;
      wall_next += std::chrono::microseconds(
          static_cast<long long>(dt > 0.0 ? dt * 1e6 : 0.0));
      std::this_thread::sleep_until(wall_next);
    } else {
      wall_next = std::chrono::steady_clock::now();
    }
    t_prev = t;

    gz::msgs::Pose req;
    req.set_name(model);
    req.mutable_position()->set_x(x);
    req.mutable_position()->set_y(y);
    req.mutable_position()->set_z(z);
    req.mutable_orientation()->set_w(std::cos(yaw / 2.0));
    req.mutable_orientation()->set_z(std::sin(yaw / 2.0));

    gz::msgs::Boolean rep;
    bool result = false;
    if (!node.Request(srv, req, 300u, rep, result) || !result)
      ++failed;
    ++count;
  }
  std::cerr << "replay_pose: streamed " << count << " poses ("
            << failed << " failed)" << std::endl;
  return 0;
}
