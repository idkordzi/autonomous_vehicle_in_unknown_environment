from pathlib import Path
import numpy as np
import cv2
import matplotlib.pyplot as plt


if __name__ == "__main__":
    video_path: Path = Path(__file__).resolve().parents[1] / "files" / "yolo" / "videos" / "capture.mp4"

    capture = cv2.VideoCapture(str(video_path))
    ret, frame = capture.read()
    if not ret:
        raise ValueError("No frame in video")

    cv2.namedWindow("frame")
    cv2.imshow("frame", frame)
    cv2.waitKey(0)

    frame_ycc = cv2.cvtColor(frame, cv2.COLOR_RGB2YCrCb)
    cv2.imshow("frame", frame_ycc)
    cv2.waitKey(0)

    # frame_ycc_mpl = cv2.cvtColor(frame_ycc, cv2.COLOR_BGR2RGB)
    # plt.imshow(frame_ycc)
    # plt.axis("off")
    # plt.show()

    cb_mean = 110
    cb_dev = 20
    cr_mean = 185
    cr_dev = 20

    frame_m = (
        np.array(
            (frame_ycc[:,:,1] > cb_mean - cb_dev)
            & (frame_ycc[:,:,1] < cb_mean + cb_dev)
            & (frame_ycc[:,:,2] > cr_mean - cr_dev)
            & (frame_ycc[:,:,2] < cr_mean + cr_dev)
        ) * 255
    ).astype(np.uint8)

    cv2.imshow("frame", frame_m)
    cv2.waitKey(0)

    while True:
        ret, frame = capture.read()
        if not ret:
            break
        frame_copy = frame.copy()

        frame_ycc = cv2.cvtColor(frame, cv2.COLOR_RGB2YCrCb)
        det_mask = np.array(
            (frame_ycc[:,:,1] > cb_mean - cb_dev)
            & (frame_ycc[:,:,1] < cb_mean + cb_dev)
            & (frame_ycc[:,:,2] > cr_mean - cr_dev)
            & (frame_ycc[:,:,2] < cr_mean + cr_dev)
        )
        if np.any(det_mask):
            det_w = np.array(np.sum(det_mask, axis=0) > 0)
            det_h = np.array(np.sum(det_mask, axis=1) > 0)
            x = np.where(det_w)[0][0]
            y = np.where(det_h)[0][0]
            w = np.sum(det_w)
            h = np.sum(det_h)
            cv2.rectangle(
                frame_copy,
                (x, y),
                (x+w, y+h),
                (0,0,255), 2
            )

        cv2.imshow("frame", frame_copy)
        k = cv2.waitKey(20)
        if k & 0xFF == ord("q"):
            break
        if k & 0xFF == 32:
            cv2.waitKey(0)

    cv2.destroyAllWindows()
    capture.release()
