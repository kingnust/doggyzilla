"""Static safety checks for the gated DOGZILLA robot description."""

from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
URDF_DIRECTORY = PACKAGE_ROOT / 'urdf'
ROBOT_XACRO = URDF_DIRECTORY / 'dogzilla_s2.urdf.xacro'
LEG_XACRO = URDF_DIRECTORY / 'dogzilla_leg.xacro'
RVIZ_CONFIG = PACKAGE_ROOT / 'rviz' / 'dogzilla_mapping.rviz'
SHADOW_RVIZ_CONFIG = PACKAGE_ROOT / 'rviz' / 'dogzilla_shadow.rviz'
XACRO_NAMESPACE = 'http://www.ros.org/wiki/xacro'


def _replace_bindings(value, bindings):
    for name, replacement in bindings.items():
        value = value.replace('${' + name + '}', replacement)
    return value


def _expanded_kinematic_graph():
    """Expand the limited leg macro names and edges without requiring ROS."""
    robot_root = ET.parse(ROBOT_XACRO).getroot()
    leg_root = ET.parse(LEG_XACRO).getroot()
    leg_tag = f'{{{XACRO_NAMESPACE}}}dogzilla_leg'

    links = [link.attrib['name'] for link in robot_root.findall('.//link')]
    joints = []
    for joint in robot_root.findall('.//joint'):
        joints.append((
            joint.attrib['name'],
            joint.attrib['type'],
            joint.find('parent').attrib['link'],
            joint.find('child').attrib['link'],
        ))

    macro_links = leg_root.findall('.//link')
    macro_joints = leg_root.findall('.//joint')
    for invocation in robot_root.findall(leg_tag):
        bindings = dict(invocation.attrib)
        links.extend(
            _replace_bindings(link.attrib['name'], bindings)
            for link in macro_links
        )
        for joint in macro_joints:
            joints.append((
                _replace_bindings(joint.attrib['name'], bindings),
                joint.attrib['type'],
                _replace_bindings(
                    joint.find('parent').attrib['link'],
                    bindings,
                ),
                _replace_bindings(
                    joint.find('child').attrib['link'],
                    bindings,
                ),
            ))
    return links, joints


def _joint(root, name):
    for joint in root.findall('.//joint'):
        if joint.attrib.get('name') == name:
            return joint
    raise AssertionError(f'joint not found: {name}')


def test_xacro_sources_are_well_formed_xml():
    ET.parse(ROBOT_XACRO)
    ET.parse(LEG_XACRO)


def test_real_xacro_expansion_when_processor_is_available():
    xacro = shutil.which('xacro')
    if xacro is None:
        return
    result = subprocess.run(
        [xacro, str(ROBOT_XACRO)],
        check=True,
        capture_output=True,
        text=True,
    )
    assert '${' not in result.stdout
    assert '$(' not in result.stdout
    root = ET.fromstring(result.stdout)
    assert len(root.findall('link')) == 21
    assert len(root.findall('joint')) == 20


def test_sensor_frames_can_be_omitted_without_changing_camera_tf():
    xacro = shutil.which('xacro')
    if xacro is None:
        return
    result = subprocess.run(
        [
            xacro,
            str(ROBOT_XACRO),
            'include_lidar:=false',
            'include_imu:=false',
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    root = ET.fromstring(result.stdout)
    links = {link.attrib['name'] for link in root.findall('link')}
    assert len(root.findall('link')) == 19
    assert len(root.findall('joint')) == 18
    assert 'laser_frame' not in links
    assert 'imu_link' not in links
    assert {'base_link', 'camera_link', 'camera_optical_frame'} <= links


def test_working_sensor_frames_and_transforms_are_preserved():
    root = ET.parse(ROBOT_XACRO).getroot()
    links = {link.attrib['name'] for link in root.findall('.//link')}
    assert {
        'base_link',
        'laser_frame',
        'imu_link',
        'camera_link',
        'camera_optical_frame',
    } <= links

    laser_origin = _joint(root, 'laser_mount_joint').find('origin')
    assert laser_origin.attrib['xyz'] == '0.000 0.000 0.180'
    assert laser_origin.attrib['rpy'] == '0 0 0'

    imu_origin = _joint(root, 'imu_mount_joint').find('origin')
    assert imu_origin.attrib['xyz'] == '0.085 0.000 0.070'
    assert imu_origin.attrib['rpy'] == '0 0 0'


def test_camera_has_a_rep103_optical_frame():
    root = ET.parse(ROBOT_XACRO).getroot()
    optical_joint = _joint(root, 'camera_optical_joint')
    assert optical_joint.find('parent').attrib['link'] == 'camera_link'
    assert optical_joint.find('child').attrib['link'] == (
        'camera_optical_frame'
    )
    assert optical_joint.find('origin').attrib['rpy'] == (
        '-1.57079632679 0 -1.57079632679'
    )


def test_all_twelve_vendor_motor_joint_names_are_generated():
    robot_root = ET.parse(ROBOT_XACRO).getroot()
    leg_tag = f'{{{XACRO_NAMESPACE}}}dogzilla_leg'
    legs = robot_root.findall(leg_tag)
    assert [leg.attrib['leg'] for leg in legs] == ['1', '2', '3', '4']

    leg_root = ET.parse(LEG_XACRO).getroot()
    joint_templates = {
        joint.attrib['name']
        for joint in leg_root.findall('.//joint')
        if joint.attrib.get('type') == 'revolute'
    }
    assert joint_templates == {
        'leg${leg}_motor1_joint',
        'leg${leg}_motor2_joint',
        'leg${leg}_motor3_joint',
    }

    expanded_names = {
        template.replace('${leg}', str(leg))
        for leg in range(1, 5)
        for template in joint_templates
    }
    assert expanded_names == {
        f'leg{leg}_motor{motor}_joint'
        for leg in range(1, 5)
        for motor in range(1, 4)
    }


def test_expanded_model_is_one_connected_acyclic_tree():
    links, joints = _expanded_kinematic_graph()
    joint_names = [name for name, _, _, _ in joints]
    children = [child for _, _, _, child in joints]

    assert len(links) == 21
    assert len(set(links)) == len(links)
    assert len(joints) == len(links) - 1
    assert len(set(joint_names)) == len(joint_names)
    assert len(set(children)) == len(children)
    assert all(parent in links for _, _, parent, _ in joints)
    assert all(child in links for _, _, _, child in joints)

    roots = set(links) - set(children)
    assert roots == {'base_link'}

    adjacency = {link: [] for link in links}
    for _, _, parent, child in joints:
        adjacency[parent].append(child)
    visited = set()
    pending = ['base_link']
    while pending:
        current = pending.pop()
        assert current not in visited
        visited.add(current)
        pending.extend(adjacency[current])
    assert visited == set(links)

    joint_types = [joint_type for _, joint_type, _, _ in joints]
    assert joint_types.count('revolute') == 12
    assert joint_types.count('fixed') == 8


def test_visualization_joint_limits_are_finite_and_ordered():
    leg_root = ET.parse(LEG_XACRO).getroot()
    for joint in leg_root.findall('.//joint'):
        if joint.attrib.get('type') != 'revolute':
            continue
        limit = joint.find('limit')
        lower = float(limit.attrib['lower'])
        upper = float(limit.attrib['upper'])
        effort = float(limit.attrib['effort'])
        velocity = float(limit.attrib['velocity'])
        assert lower < 0.0 < upper
        assert effort > 0.0
        assert velocity > 0.0


def test_all_provisional_dimensions_are_positive():
    root = ET.parse(ROBOT_XACRO).getroot()
    property_tag = f'{{{XACRO_NAMESPACE}}}property'
    dimensions = {
        element.attrib['name']: float(element.attrib['value'])
        for element in root.findall(property_tag)
    }
    assert dimensions
    assert all(value > 0.0 for value in dimensions.values())


def test_camera_pose_is_configurable_end_to_end():
    root = ET.parse(ROBOT_XACRO).getroot()
    argument_tag = f'{{{XACRO_NAMESPACE}}}arg'
    expected_arguments = {
        'camera_x',
        'camera_y',
        'camera_z',
        'camera_roll',
        'camera_pitch',
        'camera_yaw',
    }
    arguments = {
        argument.attrib['name']
        for argument in root.findall(argument_tag)
    }
    assert expected_arguments < arguments
    assert {'include_lidar', 'include_imu'} <= arguments

    camera_origin = _joint(root, 'camera_mount_joint').find('origin')
    origin_source = camera_origin.attrib['xyz'] + camera_origin.attrib['rpy']
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'robot_description.launch.py'
    ).read_text()
    for argument in expected_arguments:
        assert f'$(arg {argument})' in origin_source
        assert f"LaunchConfiguration('{argument}')" in launch_source


def test_description_is_packaged_but_not_in_operational_launches():
    setup_source = (PACKAGE_ROOT / 'setup.py').read_text()
    assert "glob('urdf/*.xacro')" in setup_source

    description_launch = (
        PACKAGE_ROOT / 'launch' / 'robot_description.launch.py'
    ).read_text()
    assert "'enabled',\n            default_value='false'" in (
        description_launch
    )
    assert 'condition=IfCondition(enabled)' in description_launch

    for launch_name in (
        'full_mapping.launch.py',
        'full_navigation.launch.py',
        'hardware.launch.py',
    ):
        operational_source = (
            PACKAGE_ROOT / 'launch' / launch_name
        ).read_text()
        assert 'robot_description.launch.py' not in operational_source


def test_operational_rviz_can_show_urdf_without_shadow_overlay():
    document = yaml.safe_load(RVIZ_CONFIG.read_text())
    displays = document['Visualization Manager']['Displays']
    by_name = {display['Name']: display for display in displays}

    model = by_name['DOGZILLA URDF']
    assert model['Class'] == 'rviz_default_plugins/RobotModel'
    assert model['Description Source'] == 'Topic'
    assert model['Description Topic']['Value'] == '/robot_description'
    assert model['Description Topic']['Durability Policy'] == (
        'Transient Local'
    )
    assert model['Enabled'] is True

    assert 'RTAB Shadow Occupancy' not in by_name
    assert by_name['Occupancy Map']['Topic']['Value'] == '/map'
    assert by_name['Occupancy Map']['Enabled'] is True


def test_shadow_rviz_uses_supported_isolated_map_and_frame():
    document = yaml.safe_load(SHADOW_RVIZ_CONFIG.read_text())
    manager = document['Visualization Manager']
    displays = manager['Displays']
    by_name = {display['Name']: display for display in displays}

    shadow_map = by_name['RTAB Shadow Occupancy']
    assert shadow_map['Class'] == 'rviz_default_plugins/Map'
    assert shadow_map['Topic']['Value'] == '/rtabmap_shadow/map'
    assert shadow_map['Topic']['Durability Policy'] == 'Transient Local'
    assert shadow_map['Update Topic']['Value'] == ''
    assert shadow_map['Enabled'] is True
    assert manager['Global Options']['Fixed Frame'] == 'rtabmap_shadow_map'
    assert by_name['RTAB Optimized Path']['Topic']['Value'] == (
        '/rtabmap_shadow/mapPath'
    )
