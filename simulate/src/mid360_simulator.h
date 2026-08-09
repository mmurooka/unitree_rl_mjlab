#pragma once

#include <memory>

#include <mujoco/mujoco.h>

class Mid360Simulator
{
public:
  Mid360Simulator(const mjModel *model, const char *scan_pattern_path);
  ~Mid360Simulator();

  Mid360Simulator(const Mid360Simulator &) = delete;
  Mid360Simulator &operator=(const Mid360Simulator &) = delete;

  bool valid() const;
  void advance(const mjModel *model, mjData *data);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
