import numpy as np
import matplotlib.pyplot as plt
import cv2


def plot_results(
    reference: np.ndarray,
    trajectory: np.ndarray,
    timestamps: np.ndarray,
) -> None:

    _ = plt.figure(figsize=(16, 12), dpi=100)

    ax1 = plt.subplot2grid((50, 50),  (0, 0), colspan=45, rowspan=16)
    ax2 = plt.subplot2grid((50, 50), (20, 0), colspan=45, rowspan=6)
    ax3 = plt.subplot2grid((50, 50), (30, 0), colspan=45, rowspan=6)
    ax4 = plt.subplot2grid((50, 50), (40, 0), colspan=45, rowspan=6)

    ax1.plot(reference[0], reference[1], label="cel", color="tab:red")
    ax1.plot(trajectory[0], trajectory[1], label="robot", color="tab:blue")
    ax1.set(xlabel="X [m]", ylabel="Y [m]", title="Pozycja XY", xlim=(0, 4), ylim=(0, 2.8))
    ax1.grid(alpha=0.2)
    ax1.legend(loc="center left", bbox_to_anchor=(1.0, 0.9))
    # ax1_bbox = ax1.get_position()
    # ax1.set_position([ax1_bbox.x0, ax1_bbox.y0, ax1_bbox.width * 0.9, ax1_bbox.height])

    ax2.plot(timestamps, reference[0], label="cel", color="tab:red")
    ax2.plot(timestamps, trajectory[0], label="robot", color="tab:blue")
    ax2.set(xlabel="czas [s]", ylabel="X [m]", title="Składowa pozycji X", xlim=(0, timestamps[-1]), ylim=(0, 5))
    ax2.grid(alpha=0.2)
    ax2.legend(loc="center left", bbox_to_anchor=(1.0, 0.9))

    ax3.plot(timestamps, reference[1], label="cel", color="tab:red")
    ax3.plot(timestamps, trajectory[1], label="robot", color="tab:blue")
    ax3.set(xlabel="czas [s]", ylabel="Y [m]", title="Składowa pozycji Y", xlim=(0, timestamps[-1]), ylim=(0, 5))
    ax3.grid(alpha=0.2)
    ax3.legend(loc="center left", bbox_to_anchor=(1.0, 0.9))

    error = np.linalg.norm(reference - trajectory, axis=0)
    ax4.plot(timestamps, error, label="odległość", color="tab:red")
    ax4.set(xlabel="czas [s]", ylabel="odległość [m]", title="Odległość do celu", xlim=(0, timestamps[-1]), ylim=(0, 5))
    ax4.grid(alpha=0.2)
    ax4.hlines(y=np.mean(error), xmin=0, xmax=timestamps[-1], colors="tab:green", linestyles="dashed", label="średnia odległość")
    ax4.legend(loc="center left", bbox_to_anchor=(1.0, 0.9))

    plt.show()


def plot_results_in_grid(
    reference_data_list: list[np.ndarray],
    test_data_list: list[np.ndarray],
    ts_data_list: list[np.ndarray],
) -> None:

    axes: list[list[plt.Axes]] = []
    nrows = 4
    ncols = 2
    _, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(10, 12), dpi=100)
    for j, axr in enumerate(axes):
        for i, ax in enumerate(axr):
            ij = j * ncols + i
            reference_data = reference_data_list[ij]
            test_data = test_data_list[ij]
            if reference_data is not None and test_data is not None:
                ax.plot(reference_data[0], reference_data[1], color="tab:blue")
                ax.plot(test_data[0], test_data[1], color="tab:red")
                ax.plot(reference_data[0,0], reference_data[1,0], "x", color="k")
                ax.plot(test_data[0,0], test_data[1,0], "x", color="k")
            if i == 0:
                ax.set(ylabel="x [m]")
            if j == nrows-1:
                ax.set(xlabel="y [m]")
            ax.set(xlim=(0, 4), ylim=(0, 2.8))
            ax.grid(alpha=0.2)

            # put box in upper left corner of the plot with text which number it is
            ax.text(0.05, 2.55, f" #{ij+1}", fontsize=12, color="black")

    plt.show()


# ################################################################################################################################
# Mock data generaiton
# ################################################################################################################################


def _get_result_mock_data() -> tuple[np.ndarray, np.ndarray, np.ndarray]:

    log_time = 100.0
    period = 0.01
    samples = int(log_time / period)
    np.random.seed(42)

    mock_timestamps = np.arange(0.0, log_time, period)

    trajectory_a = np.array(
        [
            np.linspace(3.8, 2.0, int(samples/4)),
            np.ones((int(samples/4),)) * 0.2
        ]
    )
    trajectory_b = np.array(
        [
            np.sin(np.linspace(np.pi, np.pi*2, int(samples/2))) * 1.2 + 2.0,
            np.cos(np.linspace(np.pi, np.pi*2, int(samples/2))) * 1.2 + 1.4
        ]
    )
    trajectory_c = np.array(
        [
            np.linspace(2.0, 3.8, int(samples/4)),
            np.ones((int(samples/4),)) * 2.6
        ]
    )
    trajectory = np.concatenate((trajectory_a, trajectory_b, trajectory_c), axis=1)
    noise = np.array(
        [
            np.sin(np.linspace(0.0, np.pi*3, samples)) * 0.10,
            np.sin(np.linspace(1.0, np.pi*3+1.0, samples)) * 0.05
        ]
    )

    mock_reference = trajectory[:, int(samples*0.10):int(samples*0.80)]
    mock_trajectory = (
        trajectory[:, int(samples*0.05):int(samples*0.75)]
        + noise[:, int(samples*0.05):int(samples*0.75)]
        + np.array([0.05, -0.15]).reshape(2,1)
    )
    timestamps = mock_timestamps[int(samples*0.05):int(samples*0.75)]
    timestamps -= timestamps[0]

    return timestamps, mock_reference, mock_trajectory


# ################################################################################################################################
# Main script
# ################################################################################################################################


if __name__ == "__main__":
    
    mock_timestamps, mock_reference, mock_test = _get_result_mock_data()
    # plot_results(
    #     reference=mock_reference,
    #     trajectory=mock_test,
    #     timestamps=mock_timestamps,
    # )

    mock_reference_list = [mock_reference] + [None]*8
    mock_test_list = [mock_test] + [None]*8
    mock_ts_list = [mock_timestamps] + [None]*8

    plot_results_in_grid(
        reference_data_list=mock_reference_list,
        test_data_list=mock_test_list,
        ts_data_list=mock_ts_list,
    )
