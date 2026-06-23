import os
import shutil
from pathlib import Path
import argparse
from ultralytics import YOLO


def get_yolo_model(
    model: str = "yolo12n",
    task: str = None,
) -> YOLO:
    if not model.endswith(".pt") or not model.endswith(".yaml"):
        model += ".pt"  # use default weights
    return YOLO(
        model=model,
        task=task,
    )


def save_yolo_model(
    model: str = "yolo12n",
    task: str = None,
    format: str = "onnx",
    path: str = None,
) -> None:
    model: YOLO = get_yolo_model(
        model=model,
        task=task,
    )
    model_export: str = model.export(
        format=format,
        device=0,
    )
    if not path:
        default_path: Path = Path(Path(__file__).parents[2] / "files/yolo/models").resolve()
        default_path.mkdir(parents=True, exist_ok=True)
        path: str = default_path.as_posix()
    shutil.move(
        src=model_export,
        dst=os.path.join(path, model_export),
    )
    model_torch: str = model_export[:model_export.rfind(".")] + ".pt"
    shutil.move(
        src=model_torch,
        dst=os.path.join(path, model_torch),
    )
    print(f"[INFO] YOLO model saved to: {path}")


if __name__ == "__main__":
    save_yolo_model()
