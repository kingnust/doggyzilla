"""Tests for matching live CameraInfo to the installed calibration."""

from types import SimpleNamespace

from dogzilla_slam.camera_validate import compare_camera_info


def _intrinsics():
    return {
        'distortion_model': 'plumb_bob',
        'camera_matrix': {
            'data': [
                500.0, 0.0, 320.0,
                0.0, 500.0, 240.0,
                0.0, 0.0, 1.0,
            ],
        },
        'distortion_coefficients': {
            'data': [0.01, -0.02, 0.0, 0.0, 0.0],
        },
        'rectification_matrix': {
            'data': [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
            ],
        },
        'projection_matrix': {
            'data': [
                500.0, 0.0, 320.0, 0.0,
                0.0, 500.0, 240.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ],
        },
    }


def _message(intrinsics):
    return SimpleNamespace(
        distortion_model=intrinsics['distortion_model'],
        k=list(intrinsics['camera_matrix']['data']),
        d=list(intrinsics['distortion_coefficients']['data']),
        r=list(intrinsics['rectification_matrix']['data']),
        p=list(intrinsics['projection_matrix']['data']),
    )


def test_matching_camera_info_is_accepted():
    intrinsics = _intrinsics()
    assert compare_camera_info(_message(intrinsics), intrinsics) == []


def test_changed_live_matrix_is_rejected():
    intrinsics = _intrinsics()
    message = _message(intrinsics)
    message.k[0] += 1.0
    assert compare_camera_info(message, intrinsics) == [
        'CameraInfo K matrix does not match camera.yaml'
    ]
