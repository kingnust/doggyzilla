"""Validate DOGZILLA camera intrinsics and measured mounting transform."""

import argparse
import math
import os
from pathlib import Path
import tempfile

import yaml


class CameraModelError(ValueError):
    """Raised when a camera calibration file is unsafe to deploy."""


def _mapping(value, label):
    if not isinstance(value, dict):
        raise CameraModelError(f'{label} must be a YAML mapping')
    return value


def _finite_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CameraModelError(f'{label} must be a number')
    result = float(value)
    if not math.isfinite(result):
        raise CameraModelError(f'{label} must be finite')
    return result


def _matrix(document, key, rows, columns):
    matrix = _mapping(document.get(key), key)
    if matrix.get('rows') != rows or matrix.get('cols') != columns:
        raise CameraModelError(
            f'{key} must be declared as {rows} rows by {columns} columns'
        )
    data = matrix.get('data')
    if not isinstance(data, list) or len(data) != rows * columns:
        raise CameraModelError(
            f'{key}.data must contain {rows * columns} numbers'
        )
    return [
        _finite_number(value, f'{key}.data[{index}]')
        for index, value in enumerate(data)
    ]


def _load_yaml(path):
    source = Path(path)
    if not source.is_file():
        raise CameraModelError(f'file does not exist: {source}')
    try:
        document = yaml.safe_load(source.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise CameraModelError(f'cannot read {source}: {exc}') from exc
    return _mapping(document, str(source))


def validate_intrinsics(path, width=640, height=480):
    """Validate a ROS camera_calibration YAML file for the deployed stream."""
    document = _load_yaml(path)
    if document.get('image_width') != width:
        raise CameraModelError(f'image_width must be {width}')
    if document.get('image_height') != height:
        raise CameraModelError(f'image_height must be {height}')
    if document.get('camera_name') != 'dogzilla_mono':
        raise CameraModelError('camera_name must be dogzilla_mono')

    camera = _matrix(document, 'camera_matrix', 3, 3)
    distortion = _mapping(
        document.get('distortion_coefficients'),
        'distortion_coefficients',
    )
    coefficients = distortion.get('data')
    if not isinstance(coefficients, list) or len(coefficients) < 4:
        raise CameraModelError(
            'distortion_coefficients.data must contain at least 4 numbers'
        )
    for index, value in enumerate(coefficients):
        _finite_number(value, f'distortion_coefficients.data[{index}]')
    if distortion.get('rows') != 1 or distortion.get('cols') != len(
        coefficients
    ):
        raise CameraModelError(
            'distortion_coefficients must be declared as one row'
        )
    rectification = _matrix(document, 'rectification_matrix', 3, 3)
    projection = _matrix(document, 'projection_matrix', 3, 4)

    if camera[0] <= 0.0 or camera[4] <= 0.0:
        raise CameraModelError('camera focal lengths must be positive')
    if not 0.1 * width <= camera[0] <= 10.0 * width:
        raise CameraModelError('camera fx is implausible for the image width')
    if not 0.1 * height <= camera[4] <= 10.0 * height:
        raise CameraModelError('camera fy is implausible for the image height')
    if not 0.0 <= camera[2] <= width or not 0.0 <= camera[5] <= height:
        raise CameraModelError('camera principal point is outside the image')
    if any(
        not math.isclose(camera[index], 0.0, abs_tol=1e-9)
        for index in (1, 3, 6, 7)
    ):
        raise CameraModelError('camera_matrix has an invalid pinhole layout')
    if not math.isclose(camera[8], 1.0, abs_tol=1e-6):
        raise CameraModelError('camera_matrix bottom-right value must be 1')
    if projection[0] <= 0.0 or projection[5] <= 0.0:
        raise CameraModelError('projection focal lengths must be positive')
    if not 0.0 <= projection[2] <= width:
        raise CameraModelError(
            'projection x principal point is outside the image'
        )
    if not 0.0 <= projection[6] <= height:
        raise CameraModelError(
            'projection y principal point is outside the image'
        )
    expected_projection_tail = (0.0, 0.0, 1.0, 0.0)
    if any(
        not math.isclose(value, expected, abs_tol=1e-9)
        for value, expected in zip(projection[8:], expected_projection_tail)
    ):
        raise CameraModelError('projection_matrix has an invalid final row')
    if not math.isclose(projection[3], 0.0, abs_tol=1e-9):
        raise CameraModelError(
            'monocular projection must have zero x baseline'
        )
    if not all(math.isfinite(value) for value in rectification):
        raise CameraModelError('rectification_matrix must be finite')
    if document.get('distortion_model') not in {
        'plumb_bob',
        'rational_polynomial',
        'equidistant',
    }:
        raise CameraModelError('distortion_model is unsupported')
    return document


def _validate_extrinsics_document(document):
    if document.get('schema_version') != 1:
        raise CameraModelError('camera extrinsics schema_version must be 1')
    if document.get('measured') is not True:
        raise CameraModelError('camera extrinsics are not marked measured')
    if document.get('parent_frame') != 'base_link':
        raise CameraModelError('parent_frame must be base_link')
    if document.get('child_frame') != 'camera_link':
        raise CameraModelError('child_frame must be camera_link')

    translation = _mapping(document.get('translation'), 'translation')
    rotation = _mapping(document.get('rotation_rpy'), 'rotation_rpy')
    pose = tuple(
        _finite_number(translation.get(axis), f'translation.{axis}')
        for axis in ('x', 'y', 'z')
    ) + tuple(
        _finite_number(rotation.get(axis), f'rotation_rpy.{axis}')
        for axis in ('roll', 'pitch', 'yaw')
    )
    if any(abs(value) >= 1.0 for value in pose[:3]):
        raise CameraModelError('camera translation must be within 1 metre')
    if any(abs(value) > math.pi for value in pose[3:]):
        raise CameraModelError(
            'camera RPY values must be radians in [-pi, pi]'
        )
    return pose


def validate_extrinsics(path):
    """Validate and return measured base_link-to-camera_link XYZ/RPY."""
    return _validate_extrinsics_document(_load_yaml(path))


def write_extrinsics(path, translation, rotation_degrees):
    """Atomically write an explicitly measured camera mount transform."""
    document = {
        'schema_version': 1,
        'measured': True,
        'parent_frame': 'base_link',
        'child_frame': 'camera_link',
        'translation': dict(zip(('x', 'y', 'z'), translation)),
        'rotation_rpy': dict(zip(
            ('roll', 'pitch', 'yaw'),
            (math.radians(value) for value in rotation_degrees),
        )),
    }
    _validate_extrinsics_document(document)

    destination = Path(path)
    if not destination.parent.is_dir():
        raise CameraModelError(
            f'parent directory does not exist: {destination.parent}'
        )
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            dir=destination.parent,
            prefix=f'.{destination.name}.',
            suffix='.tmp',
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            yaml.safe_dump(document, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        temporary_path.chmod(0o644)
        os.replace(temporary_path, destination)
    except OSError as exc:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        raise CameraModelError(f'cannot write {destination}: {exc}') from exc
    return document


def parse_arguments(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--intrinsics')
    parser.add_argument('--extrinsics')
    parser.add_argument('--print-pose', action='store_true')
    parser.add_argument('--write-extrinsics')
    parser.add_argument('--translation', nargs=3, type=float)
    parser.add_argument('--rotation-degrees', nargs=3, type=float)
    parser.add_argument('--confirm-measured', action='store_true')
    arguments = parser.parse_args(args)
    if arguments.write_extrinsics:
        if (
            arguments.intrinsics
            or arguments.extrinsics
            or arguments.print_pose
        ):
            parser.error('validation arguments cannot be used while writing')
        if (
            arguments.translation is None
            or arguments.rotation_degrees is None
        ):
            parser.error(
                '--translation and --rotation-degrees are required '
                'when writing'
            )
        if not arguments.confirm_measured:
            parser.error('--confirm-measured is required when writing')
    elif not arguments.intrinsics or not arguments.extrinsics:
        parser.error('--intrinsics and --extrinsics are required')
    return arguments


def main(args=None):
    arguments = parse_arguments(args)
    try:
        if arguments.write_extrinsics:
            write_extrinsics(
                arguments.write_extrinsics,
                arguments.translation,
                arguments.rotation_degrees,
            )
            print(f'Camera extrinsics written: {arguments.write_extrinsics}')
            return
        validate_intrinsics(arguments.intrinsics)
        pose = validate_extrinsics(arguments.extrinsics)
    except CameraModelError as exc:
        raise SystemExit(f'Camera deployment gate failed: {exc}') from exc
    if arguments.print_pose:
        print(' '.join(f'{value:.9g}' for value in pose))
    else:
        print('Camera intrinsics and measured extrinsics: VALID')


if __name__ == '__main__':
    main()
