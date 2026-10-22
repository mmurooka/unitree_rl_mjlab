"""MotionPrompt NPZ fields shared by offline conversion and online validation."""

import io

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


def validate_prompt(payload: bytes, dofs: int = 29):
    with np.load(io.BytesIO(payload), allow_pickle=False) as data:
        read_motion_prompt_fps(data)
        if set(data.files) != {"format_version", "fps", "q_ref", "foot_contact"}:
            raise ValueError(
                "Expected only format_version, fps, q_ref and foot_contact"
            )
        if data["format_version"].shape != () or data["format_version"].item() != 1:
            raise ValueError("Unsupported MotionPrompt format_version")
        q, contacts = data["q_ref"], data["foot_contact"]
        if q.ndim != 2 or q.shape[1] != 7 + dofs or len(q) < 4:
            raise ValueError(f"q_ref must have shape [N>=4, {7 + dofs}]")
        if contacts.shape != (len(q), 2):
            raise ValueError("foot_contact must have shape [N, 2]")
        if not np.isfinite(q).all() or not np.isfinite(contacts).all():
            raise ValueError("Non-finite MotionPrompt values")
        if not np.allclose(np.linalg.norm(q[:, 3:7], axis=1), 1, atol=1e-3):
            raise ValueError("q_ref contains a non-unit quaternion")
