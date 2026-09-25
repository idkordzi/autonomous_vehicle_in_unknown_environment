from pathlib import Path
import numpy as np
import cv2
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def read_camera() -> None:

    bagpath = Path(...)
    typestore = get_typestore(Stores.ROS2_JAZZY)

    cv2.namedWindow("stream")
    recorder = cv2.VideoWriter("ros_camera.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 30, (640, 480), True)

    ros_topic = "/camera/color/image_raw"

    with AnyReader([bagpath], default_typestore=typestore) as reader:

        connections = [x for x in reader.connections if x.topic == ros_topic]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            encoding, width, height, step =  msg.encoding, msg.width, msg.height, msg.step
            frame = np.asarray(msg.data, dtype=np.uint8).reshape((height, width, 3))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            cv2.imshow("stream", frame)
            key = cv2.waitKey(20)
            if key & 0xFF == ord("q"):
                break
            if key & 0xFF == 32:
                cv2.waitKey(0)
            if key & 0xFF == ord("s"):
                cv2.imwrite("frame.png", frame)

            recorder.write(frame)

    cv2.destroyAllWindows()
    recorder.release()


def read_position() -> None:

    bagpath = Path(...)
    typestore = get_typestore(Stores.ROS2_JAZZY)

    position_robot = []
    position_target = []

    ros_topic_robot = "/robot/ekf/pose"  # type PoseStamped
    ros_topi_target = "/robot/vision/target"  # type Vector3DStamped

    with AnyReader([bagpath], default_typestore=typestore) as reader:

        connections = [x for x in reader.connections if x.topic == ros_topic_robot]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            position_robot.append(
                np.array(
                    [
                        float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e9,
                        msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z,
                    ]
                )
            )

        connections = [x for x in reader.connections if x.topic == ros_topi_target]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            position_target.append(
                np.array(
                    [
                        float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e9,
                        msg.vector.x,
                        msg.vector.y,
                        msg.vector.z,
                    ]
                )
            )

    position_robot = np.array(position_robot)
    position_target = np.array(position_target)

    np.savez("ros_robot_0.npz", position_robot)
    np.savez("ros_target_0.npz", position_target)


if __name__ == "__main__":

    read_camera()
    read_position()
