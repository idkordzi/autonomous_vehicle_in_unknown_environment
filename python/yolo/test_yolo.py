from pathlib import Path
from ultralytics import YOLO
from ultralytics.engine.results import Results as YOLOResults
import cv2
import numpy as np
import time
from typing import Union


def prep_image():

    yolo_files: Path = Path(__file__).parents[1].resolve() / "files/yolo"
    image_path: Path = yolo_files / ...
    output_path: Path = yolo_files / "dataset/inputs/test_image.jpg"

    img: np.ndarray = cv2.imread(image_path.as_posix(), cv2.IMREAD_COLOR) # 4096 x 3072 (4:3)

    cut_size = int((3072 / 4) / 2) # 384
    img_cut = img[cut_size:-cut_size, :, :]

    img_resized = cv2.resize(img_cut, (1280, 720))
    cv2.imwrite(output_path, img_resized)


def test_yolo():

    yolo_files: Path = Path(__file__).parents[1].resolve() / "files/yolo"

    model_path: Path = yolo_files / "models/yolo26m.onnx"
    image_path: Path = yolo_files / "dataset/inputs/frame.png"
    labels_path: Path = yolo_files / "dataset/labels/coco.names"

    MIN_CONF: float = 0.2
  
    model: YOLO = YOLO(model_path, task="detect")
    # model.info() # use only when using *.pt model

    img: np.ndarray = cv2.imread(image_path.as_posix(), cv2.IMREAD_COLOR)
    cv2.imshow("Input preview", img)
    cv2.waitKey(0)

    cimg = img.copy()
    labels  = {}
    with open(labels_path.as_posix(), "r") as f:
        for ci, line in enumerate(f.read().split('\n')):
            if len(line) > 1:
                labels[ci] = line

    results: list[YOLOResults] = model.predict(source=img, device=0)
    for result in results:
        for di in range(len(result)):
            xyxy = result.boxes.xyxy[di].cpu().numpy()
            name = result.boxes.cls.int()[di].cpu().numpy()
            conf = result.boxes.conf[di].cpu().numpy()

            if conf < MIN_CONF: continue

            xyxy = [int(el) for el in xyxy]
            tx = f"{labels[int(name)]} [{int(name)}]: {conf*100:0.2f}%"
            ts = cv2.getTextSize(tx, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]

            cv2.rectangle(cimg, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0,0,255), 1)
            cv2.rectangle(cimg, (xyxy[0], xyxy[1]), (xyxy[0]+ts[0], xyxy[1]-ts[1]), (0,0,255), -1)
            cv2.putText(cimg, tx, (xyxy[0], xyxy[1]-1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

    cv2.imshow("Output preview", cimg)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def run_loops(model: YOLO, image: np.ndarray, device: Union[str, int], loops: int):

        print(f"[TEST] Run test on {loops:d} loops")
        tic = time.perf_counter_ns()
        for _ in range(loops):
            _ = model.predict(source=image, device=device, verbose=False)
        toc = time.perf_counter_ns()
        average_time = (toc - tic) / loops * 1e-9
        print(f"> average time: {average_time:0.6f} [s]")


def test_yolo_timing():

    # GPU: 0
    # CPU: "cpu"
    DEVICE = 0

    yolo_files: Path = Path(__file__).parents[1].resolve() / "files/yolo"
    
    model_path: Path = yolo_files / "models/yolo26m.onnx"
    image_path: Path = yolo_files / "dataset/inputs/test_image.jpg"

    print("[INFO] Initialize YOLO model")
    test_model: YOLO = YOLO(model_path, task="detect")

    print("[INFO] Read test image")
    test_image: np.ndarray = cv2.imread(image_path.as_posix(), cv2.IMREAD_COLOR)

    print("[INFO] Run warmup inference")
    for _ in range(5):
        _ = test_model.predict(source=test_image, device=DEVICE, verbose=False)

    run_loops(model=test_model, image=test_image, device=DEVICE, loops=10)
    run_loops(model=test_model, image=test_image, device=DEVICE, loops=100)
    run_loops(model=test_model, image=test_image, device=DEVICE, loops=1000)


if __name__ == "__main__":
    # prep_image()
    test_yolo()
    # test_yolo_timing()
