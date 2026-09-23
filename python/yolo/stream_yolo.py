from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.engine.results import Results as YOLOResults


def stream_yolo() -> None:

    DEVICE = 0
    MIN_CONF = 0.5

    yolo_files: Path = Path(__file__).parents[1].resolve() / "files/yolo"
    model_path: Path = yolo_files / "models/yolo26m.onnx"
    image_path: Path = yolo_files / "dataset/inputs/test_image.jpg"
    video_path: Path = yolo_files / "videos/capture.mp4"
    labels_path: Path = yolo_files / "dataset/labels/coco.names"

    model: YOLO = YOLO(model_path, task="detect")
    img: np.ndarray = cv2.imread(image_path.as_posix(), cv2.IMREAD_COLOR)
    labels  = {}
    with open(labels_path.as_posix(), "r") as f:
        for ci, line in enumerate(f.read().split('\n')):
            if len(line) > 1:
                labels[ci] = line
    cv2.namedWindow("stream")

    for _ in range(5):
        _ = model.predict(source=img, device=DEVICE, verbose=False)

    capture = cv2.VideoCapture(video_path)
    while True:
        ret, frame = capture.read()
        if not ret:
            break
        frame_copy = frame

        results: list[YOLOResults] = model.predict(source=frame_copy, device=DEVICE, verbose=False)
        for result in results:
            for di in range(len(result)):
                xyxy = result.boxes.xyxy[di].cpu().numpy()
                name = result.boxes.cls.int()[di].cpu().numpy()
                conf = result.boxes.conf[di].cpu().numpy()
    
                if conf < MIN_CONF: continue
    
                xyxy = [int(el) for el in xyxy]
                tx = f"{labels[int(name)]} [{int(name)}]: {conf*100:0.2f}%"
                ts = cv2.getTextSize(tx, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    
                cv2.rectangle(frame_copy, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0,0,255), 1)
                cv2.rectangle(frame_copy, (xyxy[0], xyxy[1]), (xyxy[0]+ts[0], xyxy[1]-ts[1]), (0,0,255), -1)
                cv2.putText(frame_copy, tx, (xyxy[0], xyxy[1]-1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        cv2.imshow("stream", frame_copy)
        k = cv2.waitKey(20)
        if (k & 0xFF == 27) or (k & 0xFF == ord("q")): # ESC pressed
            break
        if k & 0xFF == 32:
            cv2.waitKey(0)

    cv2.destroyAllWindows()
    capture.release()


if __name__ == "__main__":
    stream_yolo()