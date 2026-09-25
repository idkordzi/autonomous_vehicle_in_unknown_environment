import numpy as np
import matplotlib.pyplot as plt


def plot_results_single(
    aruco_robot: np.ndarray,
    aruco_target: np.ndarray,
    ros_robot: np.ndarray,
    ros_target: np.ndarray,
    include_z: bool = True,
) -> None:

    _ = plt.figure(figsize=(10, 11), dpi=100)

    ax1 = plt.subplot2grid((14, 10),  (0, 0), rowspan=8, colspan=10)
    ax2 = plt.subplot2grid((14, 10), (10, 0), rowspan=4, colspan=10)

    ax1.plot(aruco_robot[:, 1], aruco_robot[:, 2], "-", label="robot (aruco)", color="tab:red")
    ax1.plot(aruco_target[:, 1], aruco_target[:, 2], "-", label="cel (aruco)", color="tab:blue")
    ax1.plot(ros_robot[:, 1], ros_robot[:, 2], "--", label="robot (ros)", color="tab:orange")
    ax1.plot(ros_target[:, 1], ros_target[:, 2], "--", label="cel (ros)", color="tab:cyan")
    ax1.plot(
        [
            aruco_robot[0, 1],
            aruco_target[0, 1],
            ros_robot[0, 1],
            ros_target[0, 1],
        ],
        [
            aruco_robot[0, 2],
            aruco_target[0, 2],
            ros_robot[0, 2],
            ros_target[0, 2],
        ],
        "x", label="pozycja startowa", color="k"
    )
    ax1.set(xlabel="X [m]", ylabel="Y [m]", title="pozycja XY [m]", xlim=(0, 4), ylim=(0, 2.8))
    ax1.grid(alpha=0.2)
    ax1.legend(loc="upper left")

    end: int = 3 if include_z else 2
    aruco_error: np.ndarray = np.linalg.norm(aruco_target[:, 1:end] - aruco_robot[:, 1:end], axis=1)
    ros_error: np.ndarray = np.linalg.norm(ros_target[:, 1:end] - ros_robot[:, 1:end], axis=1)
    timestamps: np.ndarray = aruco_robot[:, 0]

    ax2.plot(timestamps, aruco_error, "-", label="aruco", color="tab:red")
    ax2.plot(timestamps, ros_error, "-", label="aruco", color="tab:blue")

    txt: str = (
        "Różnica odległości [m]"
        + f"\naruco: AVG: {np.mean(aruco_error):0.2f}, STD: {np.std(aruco_error):0.2f}, Q95: {np.quantile(aruco_error, 0.95):0.2f}"
        + f"\nros: AVG: {np.mean(ros_error):0.2f}, STD: {np.std(ros_error):0.2f}, Q95: {np.quantile(ros_error, 0.95):0.2f}"
    )
    ax2.set(xlabel="czas [s]", ylabel="$\Delta$d [m]", title=txt, xlim=(0, timestamps[-1]), ylim=(0, 1))
    ax2.grid(alpha=0.2)
    ax2.hlines(y=np.mean(aruco_error), xmin=0, xmax=timestamps[-1], colors="r", linestyles="dotted", label="średnia (aruco)")
    ax2.hlines(y=np.mean(ros_error), xmin=0, xmax=timestamps[-1], colors="b", linestyles="dotted", label="średnia (ros)")
    ax2.legend(loc="upper left")

    plt.show()


def plot_results_multi(
    aruco_robot_list: list[np.ndarray],
    aruco_target_list: list[np.ndarray],
    ros_robot_list: list[np.ndarray],
    ros_target_list: list[np.ndarray],
) -> None:

    axes: list[list[plt.Axes]] = []
    nrows = 4
    ncols = 2
    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(10, 13), dpi=100)
    for j, axr in enumerate(axes):
        for i, ax in enumerate(axr):
            ij = j * ncols + i
            aruco_robot = aruco_robot_list[ij]
            aruco_target = aruco_target_list[ij]
            ros_robot = ros_robot_list[ij]
            ros_target = ros_target_list[ij]
            if all(data is not None for data in [aruco_robot, aruco_target, ros_robot, ros_target]):
                ax.plot(aruco_robot[:, 1], aruco_robot[:, 2], "-", lw=0.8, label="robot (aruco)", color="tab:red")
                ax.plot(aruco_target[:, 1], aruco_target[:, 2], "-", lw=0.8, label="cel (aruco)", color="tab:blue")
                ax.plot(ros_robot[:, 1], ros_robot[:, 2], "--", lw=0.8, label="robot (ros)", color="tab:orange")
                ax.plot(ros_target[:, 1], ros_target[:, 2], "--", lw=0.8, label="cel (ros)", color="tab:cyan")
            if i == 0:
                ax.set(ylabel="x [m]")
            if j == nrows-1:
                ax.set(xlabel="y [m]")
            if ij == 0:
                ax.legend(loc="center left",  bbox_to_anchor=(0.31, 1.1), ncol=4)
            ax.set(xlim=(0, 4), ylim=(0, 2.8))
            ax.grid(alpha=0.2)

            ax.text(0.01, 2.55, f" #{ij+1}", fontsize=12, color="black")

    plt.show()


def _get_result_mock_data() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:

    log_time: float = 100.0
    period: float = 0.01
    samples: int = int(log_time / period)
    np.random.seed(42)

    mock_timestamps: np.ndarray = np.arange(0.0, log_time, period)

    trajectory_a: np.ndarray = np.array(
        [
            np.linspace(3.8, 2.0, int(samples/4)),
            np.ones((int(samples/4),)) * 0.2
        ]
    )
    trajectory_b: np.ndarray = np.array(
        [
            np.sin(np.linspace(np.pi, np.pi*2, int(samples/2))) * 1.2 + 2.0,
            np.cos(np.linspace(np.pi, np.pi*2, int(samples/2))) * 1.2 + 1.4
        ]
    )
    trajectory_c: np.ndarray = np.array(
        [
            np.linspace(2.0, 3.8, int(samples/4)),
            np.ones((int(samples/4),)) * 2.6
        ]
    )
    trajectory: np.ndarray = np.concatenate((trajectory_a, trajectory_b, trajectory_c), axis=1)
    noise_1: np.ndarray = np.array(
        [
            np.sin(np.linspace(0.0, np.pi*3, samples)) * 0.10,
            np.sin(np.linspace(1.0, np.pi*3 + 1.0, samples)) * 0.05
        ]
    )
    noise_2: np.ndarray = np.array(
        [
            np.cos(np.linspace(0.0, np.pi*3, samples)) * 0.10,
            np.cos(np.linspace(1.0, np.pi*3 + 1.0, samples)) * 0.05
        ]
    )

    timestamps: np.ndarray = mock_timestamps[int(samples*0.05):int(samples*0.75)]
    timestamps -= timestamps[0]

    trajectory_aruco_target: np.ndarray = trajectory[:, int(samples*0.10):int(samples*0.80)]
    trajectory_aruco_robot: np.ndarray = (
        trajectory[:, int(samples*0.05):int(samples*0.75)]
        + noise_1[:, int(samples*0.05):int(samples*0.75)]   # add noise
        + np.array([0.05, -0.15]).reshape(2, 1)             # move starting point
    )
    trajectory_ros_target: np.ndarray = trajectory_aruco_target + np.array([0.0, 0.1]).reshape(2, 1)
    trajectory_ros_robot: np.ndarray = (
        trajectory[:, int(samples*0.05):int(samples*0.75)]
        + noise_2[:, int(samples*0.05):int(samples*0.75)]   # add noise
        + np.array([0.05, -0.05]).reshape(2, 1)             # move starting point
    )

    mock_aruco_target = np.concatenate(
        [
            timestamps.reshape(1, -1),
            trajectory_aruco_target,
            np.full((1, len(timestamps)), np.nan),
        ],
        axis=0,
    ).T
    mock_aruco_robot = np.concatenate(
        [
            timestamps.reshape(1, -1),
            trajectory_aruco_robot,
            np.full((1, len(timestamps)), np.nan),
        ],
        axis=0,
    ).T
    mock_ros_target = np.concatenate(
        [
            timestamps.reshape(1, -1),
            trajectory_ros_target,
            np.full((1, len(timestamps)), np.nan),
        ],
        axis=0,
    ).T
    mock_ros_robot = np.concatenate(
        [
            timestamps.reshape(1, -1),
            trajectory_ros_robot,
            np.full((1, len(timestamps)), np.nan),
        ],
        axis=0,
    ).T

    return mock_aruco_target, mock_aruco_robot, mock_ros_target, mock_ros_robot


if __name__ == "__main__":
    
    mock_aruco_target, mock_aruco_robot, mock_ros_target, mock_ros_robot = _get_result_mock_data()

    plot_results_single(
        aruco_robot=mock_aruco_robot,
        aruco_target=mock_aruco_target,
        ros_robot=mock_ros_robot,
        ros_target=mock_ros_target,
        include_z=False,
    )

    mock_aruco_robot_list = [mock_aruco_robot] + [None] *8
    mock_aruco_target_list = [mock_aruco_target] + [None] *8
    mock_ros_robot_list = [mock_ros_robot] + [None] *8
    mock_ros_target_list = [mock_ros_target] + [None] *8

    plot_results_multi(
        aruco_robot_list=mock_aruco_robot_list,
        aruco_target_list=mock_aruco_target_list,
        ros_robot_list=mock_ros_robot_list,
        ros_target_list=mock_ros_target_list,
    )
