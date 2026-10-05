# SPDX-FileCopyrightText: 2026 Jinhang Dong
# SPDX-License-Identifier: MIT
import pytest
from rko_lio.dataloaders.sequencer import LidarIMUSequencer


def imu(time):
    return "imu", {"time": time}


def lidar(end_time):
    return "lidar", {"end_time_ns": end_time}


@pytest.mark.parametrize("frame_count", [1, 2, 3, 5])
def test_drain_all_covered_frames_at_end_of_input(frame_count):
    first_imu = imu(5)
    frames = [lidar(10 * (i + 1)) for i in range(frame_count)]
    trailing_imu = imu(100)
    sequencer = LidarIMUSequencer([first_imu, *frames, trailing_imu])

    assert list(sequencer) == [first_imu, *frames]


def test_imu_samples_are_emitted_once_before_their_frames():
    first_imu, boundary_imu, last_imu = imu(5), imu(10), imu(15)
    first_frame, second_frame = lidar(10), lidar(20)
    sequencer = LidarIMUSequencer([first_imu, boundary_imu, first_frame, last_imu, second_frame, imu(30)])

    assert list(sequencer) == [first_imu, first_frame, boundary_imu, last_imu, second_frame]


def test_uncovered_tail_is_not_emitted():
    first_imu, first_frame, uncovered_frame = imu(5), lidar(10), lidar(30)
    sequencer = LidarIMUSequencer([first_imu, first_frame, uncovered_frame, imu(20)])

    assert list(sequencer) == [first_imu, first_frame]


def test_continue_reading_after_ready_buffers_are_drained():
    first_imu, second_imu, third_imu = imu(5), imu(30), imu(35)
    frames = [lidar(10), lidar(20), lidar(40)]
    sequencer = LidarIMUSequencer([first_imu, *frames[:2], second_imu, third_imu, frames[2], imu(50)])

    assert list(sequencer) == [first_imu, frames[0], frames[1], second_imu, third_imu, frames[2]]


@pytest.mark.parametrize("entries", [[], [imu(10)], [lidar(10)], [lidar(10), imu(10)]])
def test_no_frame_without_strictly_later_imu_coverage(entries):
    assert list(LidarIMUSequencer(entries)) == []
