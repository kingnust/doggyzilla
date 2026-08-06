"""Finish a Cartographer trajectory and save PBStream, PGM, and YAML maps."""

import argparse
from pathlib import Path
import subprocess
import sys

import rclpy
from cartographer_ros_msgs.srv import FinishTrajectory
from cartographer_ros_msgs.srv import WriteState
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


SERVICE_TIMEOUT_SECONDS = 15.0


def _map_stem(value):
    path = Path(value).expanduser()
    if path.suffix in ('.pbstream', '.pgm', '.yaml'):
        path = path.with_suffix('')
    return path.resolve()


def _wait_for_service(node, client, service_name):
    if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SECONDS):
        raise RuntimeError(
            f'{service_name} is unavailable. Is Cartographer mapping running?'
        )


def _call_service(node, client, request, service_name):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(
        node,
        future,
        timeout_sec=SERVICE_TIMEOUT_SECONDS,
    )
    if not future.done() or future.result() is None:
        raise RuntimeError(f'{service_name} did not return a response')
    response = future.result()
    if response.status.code != 0:
        raise RuntimeError(
            f'{service_name} failed: {response.status.message}'
        )
    return response


def _parse_arguments():
    parser = argparse.ArgumentParser(
        description='Save the active DOGZILLA Cartographer map.',
    )
    parser.add_argument(
        'map_prefix',
        nargs='?',
        default='/root/yahboomcar_ws/maps/dogzilla_map',
        help='Output path without an extension.',
    )
    parser.add_argument('--trajectory-id', type=int, default=0)
    parser.add_argument(
        '--skip-finish',
        action='store_true',
        help='Do not call /finish_trajectory before writing the state.',
    )
    parser.add_argument(
        '--include-unfinished-submaps',
        action='store_true',
        help='Include unfinished submaps in the PBStream.',
    )
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main(args=None):
    cli = _parse_arguments()
    map_stem = _map_stem(cli.map_prefix)
    map_stem.parent.mkdir(parents=True, exist_ok=True)
    pbstream_path = map_stem.with_suffix('.pbstream')

    rclpy.init(args=args)
    node = Node('dogzilla_map_saver')
    error = None

    try:
        if not cli.skip_finish:
            finish_client = node.create_client(
                FinishTrajectory,
                '/finish_trajectory',
            )
            _wait_for_service(
                node,
                finish_client,
                '/finish_trajectory',
            )
            finish_request = FinishTrajectory.Request()
            finish_request.trajectory_id = cli.trajectory_id
            _call_service(
                node,
                finish_client,
                finish_request,
                '/finish_trajectory',
            )
            node.get_logger().info(
                f'Finished Cartographer trajectory {cli.trajectory_id}'
            )

        write_client = node.create_client(WriteState, '/write_state')
        _wait_for_service(node, write_client, '/write_state')
        write_request = WriteState.Request()
        write_request.filename = str(pbstream_path)
        write_request.include_unfinished_submaps = (
            cli.include_unfinished_submaps
        )
        _call_service(
            node,
            write_client,
            write_request,
            '/write_state',
        )
        node.get_logger().info(f'Wrote {pbstream_path}')
    except RuntimeError as exc:
        error = str(exc)
        node.get_logger().error(error)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if error is not None:
        return 1

    # Cartographer's pbstream_to_ros_map binary in Yahboom's ARM64 Humble
    # image aborts inside Cairo. Save the live occupancy grid with Nav2 while
    # retaining the PBStream above for future Cartographer localization.
    occupancy_grid_saver = [
        'ros2',
        'run',
        'nav2_map_server',
        'map_saver_cli',
        '-t',
        '/map',
        '-f',
        str(map_stem),
        '--occ',
        '0.65',
        '--free',
        '0.25',
        '--fmt',
        'pgm',
        '--mode',
        'trinary',
        '--ros-args',
        '-p',
        'save_map_timeout:=10.0',
    ]
    result = subprocess.run(occupancy_grid_saver, check=False)
    if result.returncode != 0:
        print('Failed to save /map as PGM/YAML files.', file=sys.stderr)
        return result.returncode

    print(f'Saved {pbstream_path}')
    print(f'Saved {map_stem.with_suffix(".pgm")}')
    print(f'Saved {map_stem.with_suffix(".yaml")}')
    return 0
