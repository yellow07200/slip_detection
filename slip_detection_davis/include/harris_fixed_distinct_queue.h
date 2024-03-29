#pragma once

#include <deque>
#include <Eigen/Dense>

namespace slip_detection_davis
{

class HarrisFixedDistinctQueue
{
public:
	HarrisFixedDistinctQueue(int window, int queue);

  bool isFull() const;

  void addNew(int x, int y);
  Eigen::MatrixXi getWindow() const;

private:
  // contains index of queue element if occupied, negative value otherwise
  Eigen::MatrixXi window_;
  // contains one event
  struct QueueEvent
  {
    int prev, next;
    int x, y;
  };
  std::vector<QueueEvent> queue_;
  int first_, last_;
  int queue_max_;
};


} // namespace
