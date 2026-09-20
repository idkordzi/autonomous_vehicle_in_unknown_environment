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
    arena_size: tuple[float, float]

    corners_ids: list[int]
    robot_id: int
    target_id: int


class ArucoStreamer:

    def __init__(self, config: ArucoStreamerConfig):
        self.aruco_variant: int = config.aruco_variant

        self.arena_img_size: tuple[int, int] = config.arena_img_size
        self.arena_img_corners: np.ndarray = np.array(
            [
                [0, 0],
                [self.arena_img_size[0], 0],
                [0, self.arena_img_size[1]],
                [self.arena_img_size[0], self.arena_img_size[1]]
            ]
        ).astype(int)
        self.arena_size: tuple[float, float] = config.arena_size
        self.arena_corners: np.ndarray = np.array(
            [
                [0, 0],
                [self.arena_size[0], 0],
                [0, self.arena_size[1]],
                [self.arena_size[0], self.arena_size[1]]
            ]
        ).astype(float)

        self.corners_ids: list[int] = config.corners_ids
        self.robot_id: int = config.robot_id
        self.target_id: int = config.target_id

    def detect_markers(self, image: np.ndarray, padding: int = 0) -> tuple[list[int], list[np.ndarray]]:

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
        pad_off = np.array([[padding, padding]])

        marker_ids = [id[0] for id in ids]
        marker_corners = [np.array(im_coordinated[0]) - pad_off for im_coordinated in corners]
        sorted_indices = np.argsort(marker_ids).astype(int)
        marker_ids = [marker_ids[i] for i in sorted_indices]
        marker_corners = [marker_corners[i] for i in sorted_indices]

        return marker_ids, marker_corners

    def unwrap_image(self, image: np.ndarray, src_points: np.ndarray) -> np.ndarray:
        dst_points = self.arena_img_corners
        M = cv2.getPerspectiveTransform(src_points.astype(np.float32), dst_points.astype(np.float32))
        image_transformed = cv2.warpPerspective(image, M, self.arena_img_size)
        return image_transformed

    def detect_on_image(self, image: np.ndarray, show_index: bool = False) -> np.ndarray:

        image_copy = image.copy()

        aruco_ids_original, aruco_corners_original = self.detect_markers(image)
        for id, corners in zip(aruco_ids_original, aruco_corners_original):
            cv2.polylines(
                image_copy,
                [np.int32(corners)],
                True, (0, 255, 0), 2
            )
            if show_index:
                cv2.putText(
                    image_copy,
                    str(id),
                    tuple(np.int32(np.mean(corners, axis=0) + np.array([-5, 5]))),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2
                )
            for i in range(4):
                cv2.circle(
                    image_copy,
                    tuple(np.int32(corners[i])),
                    5, (0, 0, 255), -1
                )
                if show_index:
                    cv2.putText(
                        image_copy,
                        f"{i+1}",
                        tuple(np.int32(corners[i] + np.array([5, -5]))),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1
                    )

        if not all(corner_id in aruco_ids_original for corner_id in self.corners_ids):
            missing = [id for id in self.corners_ids if id not in aruco_ids_original]
            txt = f"Cannot detect corner markers, missing: {missing}"
            tsz = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 1)[0]
            cv2.rectangle(image_copy, (0, 0), (tsz[0]+2, tsz[1]+2), (0,0,0), -1)
            cv2.putText(image_copy, txt, (1, tsz[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            return image_copy

        corner_points = np.array([aruco_corners_original[aruco_ids_original.index(id)][0] for id in self.corners_ids])
        image_unwrapped = self.unwrap_image(image, corner_points)

        aruco_ids_unwrapped, aruco_corners_unwrapped = self.detect_markers(image_unwrapped, padding=10)
        if not self.robot_id in aruco_ids_unwrapped or not self.target_id in aruco_ids_unwrapped:
            missing = [id for id in [self.robot_id, self.target_id] if id not in aruco_ids_unwrapped]
            txt = f"Cannot detect movable markers, missing: {missing}"
            tsz = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 1)[0]
            cv2.rectangle(image_copy, (0, 0), (tsz[0]+2, tsz[1]+2), (0,0,0), -1)
            cv2.putText(image_copy, txt, (1, tsz[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            return image_copy

        robot_position_image = np.mean(aruco_corners_original[aruco_ids_original.index(self.robot_id)], axis=0)
        robot_position_unwrapped = np.mean(aruco_corners_unwrapped[aruco_ids_unwrapped.index(self.robot_id)], axis=0)
        robot_position_real = (robot_position_unwrapped / np.array(self.arena_img_size)) * np.array(self.arena_size)

        target_position_image = np.mean(aruco_corners_original[aruco_ids_original.index(self.target_id)], axis=0)
        target_position_unwrapped = np.mean(aruco_corners_unwrapped[aruco_ids_unwrapped.index(self.robot_id)], axis=0)
        target_position_real = (target_position_unwrapped / np.array(self.arena_img_size)) * np.array(self.arena_size)

        cv2.putText(
            image_copy,
            f"ROBOT ({robot_position_real[0]:.3f}, {robot_position_real[1]:.3f})",
            (robot_position_image + np.array([20, -20])).astype(int).tolist(),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2
        )
        cv2.putText(
            image_copy,
            f"TARGET ({target_position_real[0]:.2f}, {target_position_real[1]:.2f})",
            (target_position_image + np.array([20, -20])).astype(int).tolist(),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2
        )

        txt = f"Status: OK"
        tsz = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 1)[0]
        cv2.rectangle(image_copy, (0, 0), (tsz[0]+2, tsz[1]+2), (0,0,0), -1)
        cv2.putText(image_copy, txt, (1, tsz[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        return image_copy

    def detect_on_stream(self, capture: cv2.VideoCapture, show_index: bool = False) -> None:

        cv2.namedWindow("stream")
        while True:
            ret, frame = capture.read()
            if not ret:
                break

            frame_copy = self.detect_on_image(frame, show_index)

            cv2.imshow("stream", frame_copy)
            k = cv2.waitKey(1)
            if k % 256 == 27: # ESC pressed
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
            [np.int32(corners[i][0])],
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

    aruco_image = aruco_streamer.detect_on_image(arena_RGB)
    plt.imshow(aruco_image)
    plt.axis("off")
    plt.show()

    # capture = cv2.VideoCapture(0)
    # aruco_streamer.detect_on_stream(capture)
