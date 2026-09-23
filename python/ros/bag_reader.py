from pathlib import Path
import numpy as np
import cv2
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


if __name__ == "__main__":

    bagpath = Path(...)

    # Create a type store to use if the bag has no message definitions.
    typestore = get_typestore(Stores.ROS2_JAZZY)

    window = cv2.namedWindow("stream")
    recorder = cv2.VideoWriter("capture.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 30, (640, 480), True)

    # Create reader instance and open for reading.
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == '/camera/color/image_raw']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            encoding, width, height, step =  msg.encoding, msg.width, msg.height, msg.step
            frame = np.asarray(msg.data, dtype=np.uint8).reshape((height, width, 3))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cv2.imshow("stream", frame)
            key = cv2.waitKey(20)
            if key & 0xFF == ord("q"):
                break
            if key & 0xFF == 32: # SPACE:
                cv2.waitKey(0)
            if key & 0xFF == ord("s"):
                cv2.imwrite("frame.png", frame)
            recorder.write(frame)
    cv2.destroyAllWindows()
    recorder.release()
