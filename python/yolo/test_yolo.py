from pathlib import Path
from ultralytics import YOLO
import cv2
import numpy as np


def test_yolo():

    yolo_files: Path = Path(__file__).parents[2].resolve() / "files/yolo"

    model_path: Path = yolo_files / "models/yolo12n.pt"
    image_path: Path = yolo_files / "dataset/inputs/000000017627.jpg"
    labels_path: Path = yolo_files / "dataset/labels/coco.names"

    MIN_CONF: float = 0.5
  
    model: YOLO = YOLO(model_path).eval()
    model.info()

    img: np.ndarray = cv2.imread(image_path.as_posix(), cv2.IMREAD_COLOR)
    cv2.imshow("Input preview", img)
    cv2.waitKey(0)

    cimg = img.copy()
    labels  = {}
    with open(labels_path.as_posix(), "r") as f:
        for ci, line in enumerate(f.read().split('\n')):
            if len(line) > 1:
                labels[ci] = line

    results: list = model(img)
    for result in results:
        for di in range(len(result)):
            xyxy = result.boxes.xyxy[di].cpu().numpy()
            name = result.boxes.cls.int()[di].cpu().numpy()
            conf = result.boxes.conf[di].cpu().numpy()

            if conf < MIN_CONF: continue

            xyxy = [int(el) for el in xyxy]
            tx = f"{labels[int(name)]}: {conf*100:0.2f}%"
            ts = cv2.getTextSize(tx, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]

            cv2.rectangle(cimg, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0,0,255), 1)
            cv2.rectangle(cimg, (xyxy[0], xyxy[1]), (xyxy[0]+ts[0], xyxy[1]-ts[1]), (0,0,255), -1)
            cv2.putText(cimg, tx, (xyxy[0], xyxy[1]-1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

    cv2.imshow("Output preview", cimg)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
  

if __name__ == "__main__":
    test_yolo()