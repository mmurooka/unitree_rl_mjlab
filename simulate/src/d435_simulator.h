#pragma once

#include <memory>

#include <mujoco/mujoco.h>

class D435Simulator
{
public:
  explicit D435Simulator(const mjModel *model);
  ~D435Simulator();

  D435Simulator(const D435Simulator &) = delete;
  D435Simulator &operator=(const D435Simulator &) = delete;

  bool valid() const;
  void render(const mjModel *model, mjData *data, mjrContext *context);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
