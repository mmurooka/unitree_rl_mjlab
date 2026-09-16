"""MotionPrompt NPZ fields shared by offline conversion and online validation."""

import numpy as np


def read_motion_prompt_fps(data) -> float:
    if "fps" not in data:
        raise ValueError("MotionPrompt NPZ is missing 'fps'; regenerate it with current TaskPromptRL")
    fps = np.asarray(data["fps"])
    if fps.shape != () or fps.dtype.kind not in "fiu":
        raise ValueError("MotionPrompt fps must be a numeric scalar")
    value = float(fps)
    if not np.isfinite(value) or value <= 0:
        raise ValueError("MotionPrompt fps must be finite and positive")
    return value
