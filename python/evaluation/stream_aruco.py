import numpy as np
import matplotlib.pyplot as plt
import cv2
from dataclasses import dataclass



def get_aruco_marker(id: int, size: int = 100) -> np.ndarray:

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_id = id % 50
    marker_size = size
    marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    return marker


@dataclass
class ArucoStreamerConfig:
    aruco_variant: int
    arena_img_size: tuple[int,int]
    arena_size: tuple[int, int]
    corners_ids: list[int]
    robot_id: int
    target_id: int


class ArucoStreamer:

    def __init__(self, config: ArucoStreamerConfig):
        self.aruco_variant: int = config.aruco_variant
        self.arena_img_size: tuple[int, int] = config.arena_img_size
        self.arena_size: tuple[int, int] = config.arena_size
        self.corners_ids: list[int] = config.corners_ids
        self.robot_id: int = config.robot_id
        self.target_id: int = config.target_id

    def unwrap_from_image(self, image: np.ndarray) -> np.ndarray:

        # detect aruco markers in the corners and find transformation matrix
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_variant)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        corners, ids, _ = detector.detectMarkers(image)
        if ids is None or len(ids) < 4:
            raise ValueError("Not enough ArUco markers detected in the image.")

        # Sort corners based on marker IDs
        sorted_indices = np.argsort(ids.flatten())
        sorted_corners = [corners[i][0][0] for i in sorted_indices if ids[i][0] in self.corners_ids]
        sorted_corners = np.array(sorted_corners, dtype=np.float32)
        if len(sorted_corners) < len(self.corners_ids):
            raise ValueError("Not enough ArUco markers detected for arena corners.")

        # Define the destination points for the transformation
        dst_points = np.array([
            [0, 0],
            [self.arena_img_size[0], 0],
            [0, self.arena_img_size[1]],
            [self.arena_img_size[0], self.arena_img_size[1]]
        ], dtype=np.float32)

        # Compute the perspective transformation matrix
        M = cv2.getPerspectiveTransform(sorted_corners, dst_points)

        # Apply the perspective transformation to the image
        transformed_image = cv2.warpPerspective(image, M, self.arena_img_size)

        return transformed_image

    def get_aruco_position(self, image: np.ndarray, id: int, padding: int = 0) -> tuple[np.ndarray, np.ndarray]:
        
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_variant)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        image_padded = cv2.copyMakeBorder(
            image,
            padding, padding, padding, padding,
            cv2.BORDER_CONSTANT,
            value=255
        )
        corners, ids, _ = detector.detectMarkers(image_padded)
        if ids is None or len(ids) == 0:
            raise ValueError("Not enough ArUco markers detected in the image.")

        img_position = None
        for i in range(len(ids)):
            if ids[i][0] == id:
                img_position = np.mean(corners[i][0], axis=0)
        if img_position is None:
            raise ValueError("Marker not found in the image.")

        img_position -= np.array([padding, padding])
        real_position = np.array(
            [
                (img_position[0] / self.arena_img_size[0]) * self.arena_size[0],
                (img_position[1] / self.arena_img_size[1]) * self.arena_size[1],
            ]
        )

        return img_position, real_position

    def detect_on_image(self, image: np.ndarray) -> None:
        image_unwrapped = self.unwrap_from_image(image)

        robot_img_position, robot_real_position = self.get_aruco_position(image_unwrapped, self.robot_id, padding=10)
        target_img_position, target_real_position = self.get_aruco_position(image_unwrapped, self.target_id, padding=10)

        cv2.putText(
            image_unwrapped,
            f"ROBOT ({self.robot_id:d}): ({robot_real_position[0]:.2f}, {robot_real_position[1]:.2f})",
            (robot_img_position + np.array([20, -20])).astype(int).tolist(),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2
        )
        cv2.putText(
            image_unwrapped,
            f"TARGET ({self.target_id:d}): ({target_real_position[0]:.2f}, {target_real_position[1]:.2f})",
            (target_img_position + np.array([20, -20])).astype(int).tolist(),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2
        )

        plt.imshow(image_unwrapped)
        plt.axis("off")
        plt.show()

    def detect_on_stream(self, capture: cv2.VideoCapture) -> None:

        cv2.namedWindow("stream")
        while True:
            ret, frame = capture.read()
            if not ret:
                break

            frame_unwrapped = self.unwrap_from_image(frame)
    
            robot_img_position, robot_real_position = self.get_aruco_position(frame_unwrapped, self.robot_id, padding=10)
            target_img_position, target_real_position = self.get_aruco_position(frame_unwrapped, self.target_id, padding=10)
    
            cv2.putText(
                frame_unwrapped,
                f"ROBOT ({self.robot_id:d}): ({robot_real_position[0]:.2f}, {robot_real_position[1]:.2f})",
                (robot_img_position + np.array([20, -20])).astype(int).tolist(),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2
            )
            cv2.putText(
                frame_unwrapped,
                f"TARGET ({self.target_id:d}): ({target_real_position[0]:.2f}, {target_real_position[1]:.2f})",
                (target_img_position + np.array([20, -20])).astype(int).tolist(),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2
            )

            cv2.imshow("test", frame_unwrapped)
            k = cv2.waitKey(1)
            if k % 256 == 27:
                # ESC pressed
                break


def _get_mock_arena() -> np.ndarray:

    arena_size = (int(6*1e2), int(4*1e2))  # cm
    arena_image = np.ones((arena_size[1], arena_size[0]), dtype=np.uint8) * 255

    arena_image[:, ::50] = 127
    arena_image[:, -1] = 127
    arena_image[::50, :] = 127
    arena_image[-1, :] = 127

    aruco_1 = get_aruco_marker(id=1, size=20)
    aruco_2 = get_aruco_marker(id=2, size=20)
    aruco_3 = get_aruco_marker(id=3, size=20)
    aruco_4 = get_aruco_marker(id=4, size=20)

    aruco_2 = cv2.rotate(aruco_2, cv2.ROTATE_90_CLOCKWISE)
    aruco_3 = cv2.rotate(aruco_3, cv2.ROTATE_90_COUNTERCLOCKWISE)
    aruco_4 = cv2.rotate(aruco_4, cv2.ROTATE_180)

    aruco_robot = get_aruco_marker(id=5, size=20)
    aruco_target = get_aruco_marker(id=6, size=20)

    arena_image[0:20, 0:20] = aruco_1
    arena_image[0:20, 580:600] = aruco_2
    arena_image[380:400, 0:20] = aruco_3
    arena_image[380:400, 580:600] = aruco_4

    robot_position = (285, 175)
    target_position = (475, 345)

    target_image_size = (1200, 800)
    print(np.array(robot_position) / np.array(arena_size) * (np.array(target_image_size)))
    print(np.array(target_position) / np.array(arena_size) * (np.array(target_image_size)))

    arena_image[robot_position[1]-10:robot_position[1]+10, robot_position[0]-10:robot_position[0]+10] = aruco_robot
    arena_image[target_position[1]-10:target_position[1]+10, target_position[0]-10:target_position[0]+10] = aruco_target

    arena_image = cv2.copyMakeBorder(arena_image, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=255)

    arena_image = cv2.resize(arena_image, target_image_size, interpolation=cv2.INTER_NEAREST)

    return arena_image


def _get_mock_arena_tilted() -> np.ndarray:

    arena_image = _get_mock_arena()
    rows, cols = arena_image.shape
    pts1 = np.float32([[0, 0], [cols, 0], [0, rows], [cols, rows]])
    pts2 = np.float32([[160, 50], [cols-160, 50], [10, rows-50], [cols-10, rows-50]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    tilted_arena_image = cv2.warpPerspective(arena_image, M, (cols, rows))

    return tilted_arena_image


def _debug_aruco_detection(image: np.ndarray) -> None:

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    corners, ids, _ = detector.detectMarkers(image)
    if ids is None or len(ids) < 1:
        raise ValueError("No ArUco markers detected in the image.")

    image_debug = image.copy()
    for i in range(len(corners)):
        cv2.polylines(
            image_debug,
            [np.int32(corners[i])],
            True, (0, 255, 0), 2
        )
        cv2.putText(
            image_debug,
            str(ids[i][0]),
            tuple(np.int32(np.mean(corners[i][0], axis=0) + np.array([-5, 5]))),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2
        )
        for j in range(4):
            cv2.circle(
                image_debug,
                tuple(np.int32(corners[i][0][j])),
                5, (0, 0, 255), -1
            )
            cv2.putText(
                image_debug,
                f"{j+1}",
                tuple(np.int32(corners[i][0][j] + np.array([5, -5]))),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1
            )

    plt.imshow(image_debug)
    plt.axis("off")
    plt.show()


if __name__ == "__main__":

    # aruco = get_aruco_marker(id=2, size=100)
    # plt.imshow(aruco, cmap="gray")
    # plt.axis("off")
    # plt.show()

    config: ArucoStreamerConfig = ArucoStreamerConfig(
        aruco_variant=cv2.aruco.DICT_4X4_50,
        arena_img_size=(1200, 800),
        arena_size=(1.2, 0.8),
        corners_ids=[1, 2, 3, 4],
        robot_id=5,
        target_id=6,
    )
    aruco_streamer: ArucoStreamer = ArucoStreamer(config)

    arena_tilted = _get_mock_arena_tilted()
    arena_RGB = cv2.cvtColor(arena_tilted, cv2.COLOR_GRAY2RGB)
    print(np.shape(arena_RGB))
    plt.imshow(arena_RGB)
    plt.axis("off")
    plt.show()

    _debug_aruco_detection(arena_RGB)

    aruco_streamer.detect_on_image(arena_RGB)

    # capture = cv2.VideoCapture(0)
