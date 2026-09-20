import os
import glob
import shutil
from pathlib import Path
from ultralytics import YOLO


def get_yolo_model(
    model: str = "yolo26m",
    task: str = "detect",
) -> YOLO:
    if not model.endswith(".pt") or not model.endswith(".yaml"):
        model += ".pt"  # use default weights
    return YOLO(
        model=model,
        task=task,
        verbose=True
    )


def save_yolo_model(
    model: str = "yolo26m",
    task: str = "detect",
    format: str = "onnx",
    path: str = None,
) -> None:

    for file in glob.glob("yolo*"):
        if os.path.isfile(file):
            os.remove(file)

    model: YOLO = get_yolo_model(
        model=model,
        task=task,
    )
    model_export: str = model.export(
        format=format,
        device=0,
    )

    suffix = ""
    if not path:
        default_path: Path = Path(Path(__file__).parents[1] / "files/yolo/models").resolve()
        default_path.mkdir(parents=True, exist_ok=True)
        path: str = default_path.as_posix()
    shutil.move(
        src=model_export,
        dst=os.path.join(path, model_export.rstrip(".onnx") + suffix + ".onnx"),
    )
    model_torch: str = model_export[:model_export.rfind(".")] + ".pt"
    shutil.move(
        src=model_torch,
        dst=os.path.join(path, model_torch.rstrip(".pt") + suffix + ".pt"),
    )
    print(f"[INFO] YOLO model saved to: {path}")


if __name__ == "__main__":
    save_yolo_model(
        model="yolo26m"
    )
