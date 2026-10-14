#pragma once

#include <memory>

#include <mujoco/mujoco.h>

class FramePosePublisher
{
public:
  explicit FramePosePublisher(const mjModel *model);
  ~FramePosePublisher();

  FramePosePublisher(const FramePosePublisher &) = delete;
  FramePosePublisher &operator=(const FramePosePublisher &) = delete;

  void advance(const mjModel *model, const mjData *data);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
