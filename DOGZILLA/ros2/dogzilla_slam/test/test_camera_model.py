"""Safety tests for camera intrinsic and mounting calibration gates."""

from pathlib import Path

import pytest
import yaml

from dogzilla_slam.camera_model import CameraModelError
from dogzilla_slam.camera_model import validate_extrinsics
from dogzilla_slam.camera_model import validate_intrinsics
from dogzilla_slam.camera_model import write_extrinsics


def _intrinsics():
    return {
        'image_width': 640,
        'image_height': 480,
        'camera_name': 'dogzilla_mono',
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1,
            'cols': 5,
            'data': [0.01, -0.02, 0.0, 0.0, 0.0],
        },
        'rectification_matrix': {
            'rows': 3,
            'cols': 3,
            'data': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        },
        'projection_matrix': {
            'rows': 3,
            'cols': 4,
            'data': [
                500.0, 0.0, 320.0, 0.0,
                0.0, 500.0, 240.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ],
        },
    }


def _extrinsics():
    return {
        'schema_version': 1,
        'measured': True,
        'parent_frame': 'base_link',
        'child_frame': 'camera_link',
        'translation': {'x': 0.15, 'y': 0.0, 'z': 0.075},
        'rotation_rpy': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    }


def _write(path, document):
    path.write_text(yaml.safe_dump(document), encoding='utf-8')
    return path


def test_valid_camera_model_is_accepted(tmp_path):
    intrinsics = _write(tmp_path / 'camera.yaml', _intrinsics())
    extrinsics = _write(tmp_path / 'extrinsics.yaml', _extrinsics())
    validate_intrinsics(intrinsics)
    assert validate_extrinsics(extrinsics) == (
        0.15, 0.0, 0.075, 0.0, 0.0, 0.0,
    )


@pytest.mark.parametrize(
    ('field', 'value'),
    (
        ('image_width', 1280),
        ('image_height', 720),
        ('camera_name', 'default_cam'),
    ),
)
def test_wrong_stream_contract_is_rejected(tmp_path, field, value):
    document = _intrinsics()
    document[field] = value
    path = _write(tmp_path / 'camera.yaml', document)
    with pytest.raises(CameraModelError):
        validate_intrinsics(path)


def test_zero_focal_length_is_rejected(tmp_path):
    document = _intrinsics()
    document['camera_matrix']['data'][0] = 0.0
    path = _write(tmp_path / 'camera.yaml', document)
    with pytest.raises(CameraModelError):
        validate_intrinsics(path)


@pytest.mark.parametrize(
    ('field', 'index', 'value'),
    (
        ('camera_matrix', 2, 700.0),
        ('camera_matrix', 1, 0.25),
        ('projection_matrix', 3, 1.0),
    ),
)
def test_implausible_pinhole_layout_is_rejected(
    tmp_path,
    field,
    index,
    value,
):
    document = _intrinsics()
    document[field]['data'][index] = value
    path = _write(tmp_path / 'camera.yaml', document)
    with pytest.raises(CameraModelError):
        validate_intrinsics(path)


def test_unmeasured_example_is_rejected(tmp_path):
    document = _extrinsics()
    document['measured'] = False
    path = _write(tmp_path / 'extrinsics.yaml', document)
    with pytest.raises(CameraModelError):
        validate_extrinsics(path)


def test_implausible_translation_is_rejected(tmp_path):
    document = _extrinsics()
    document['translation']['x'] = 1.0
    path = _write(tmp_path / 'extrinsics.yaml', document)
    with pytest.raises(CameraModelError):
        validate_extrinsics(path)


def test_repository_example_cannot_pass_deployment_gate():
    example = Path(__file__).resolve().parents[3] / (
        'calibration/camera_extrinsics.example.yaml'
    )
    with pytest.raises(CameraModelError):
        validate_extrinsics(example)


def test_extrinsics_writer_converts_degrees_and_marks_measurement(tmp_path):
    output = tmp_path / 'camera_extrinsics.yaml'
    write_extrinsics(output, (0.15, -0.01, 0.075), (90.0, 0.0, -45.0))

    document = yaml.safe_load(output.read_text(encoding='utf-8'))
    assert document['measured'] is True
    assert validate_extrinsics(output) == pytest.approx((
        0.15,
        -0.01,
        0.075,
        1.5707963267948966,
        0.0,
        -0.7853981633974483,
    ))


def test_invalid_extrinsics_write_preserves_existing_file(tmp_path):
    output = tmp_path / 'camera_extrinsics.yaml'
    output.write_text('original\n', encoding='utf-8')
    with pytest.raises(CameraModelError):
        write_extrinsics(output, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert output.read_text(encoding='utf-8') == 'original\n'
