import os
from pathlib import Path


def resolve_checkpoint_path(checkpoint, sam2_module_file, cwd=None):
    """Resolve a checkpoint from the working directory or SAM 2 checkout."""
    raw_path = Path(os.path.expandvars(str(checkpoint))).expanduser()
    candidates = []

    if raw_path.is_absolute():
        candidates.append(raw_path)
    else:
        working_directory = Path.cwd() if cwd is None else Path(cwd)
        candidates.append(working_directory / raw_path)

        if sam2_module_file:
            sam2_root = Path(sam2_module_file).resolve().parent.parent
            candidates.append(sam2_root / raw_path)

    checked = []
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved in checked:
            continue
        checked.append(resolved)
        if resolved.is_file():
            return str(resolved)

    checked_text = ", ".join(str(path) for path in checked)
    raise FileNotFoundError(
        f"SAM 2 checkpoint '{checkpoint}' was not found. Checked: {checked_text}. "
        "Set the ROS 'checkpoint' parameter to the checkpoint's absolute path."
    )
