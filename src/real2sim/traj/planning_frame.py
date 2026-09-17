"""Sensor-center TCP derived from the current articulated robot, in meters.

The origin is the midpoint of the two declared sensor contact centers. Axes
follow link7. The offset changes with the gripper linkage; it is not a fixed
controller TCP. Hardware calibration records are deliberately not modified.
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from real2sim.contracts import rigid


def origin(node):
    T = np.eye(4)
    if node is not None:
        T[:3, 3] = np.fromstring(node.get('xyz', '0 0 0'), sep=' ')
        T[:3, :3] = Rotation.from_euler('xyz', np.fromstring(node.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return rigid(T, 'URDF origin')


class SensorTCP:
    definition = 'sensor_center'

    def __init__(self, case):
        case = Path(case)
        tree = ET.parse(case / 'inputs/xarm7_calibrated.urdf')
        # A fixed link_tcp at z=172 mm was the old controller TCP.  Keeping it
        # in the articulated model makes it too easy for a generic diagnostic
        # to silently report the obsolete frame, so current cases must remove
        # it before planning or replay.
        if tree.find("./joint[@name='joint_tcp']") is not None or tree.find("./link[@name='link_tcp']") is not None:
            raise ValueError('Legacy link_tcp/joint_tcp is present; regenerate the current robot without the fixed TCP')
        self.joints = tree.findall('./joint')
        self.mounts = {}
        for side, letter in [('left', 'L'), ('right', 'R')]:
            joint = tree.find(f"./joint[@name='custom_contact_{letter}_fix']")
            if joint is None or joint.get('type') != 'fixed' or joint.find('parent').get('link') != side + '_finger':
                raise ValueError(f'Missing fixed sensor center on {side}_finger; regenerate the current robot')
            self.mounts[side + '_finger'] = origin(joint.find('origin'))
            link = tree.find(f"./link[@name='{side}_finger']")
            for part in ('connector', 'fingertip', 'sensor'):
                name = f'custom_{letter}_{part}_in_finger_frame_m'
                if link is None or len(link.findall(f"collision[@name='{name}']")) != 1:
                    raise ValueError(f'{name} must have exactly one collider on {side}_finger')
        mapping = json.loads((case / 'inputs/gripper_mapping.json').read_text())
        self.gaps = np.asarray(mapping['gap_m'], float)
        self.drives = np.asarray(mapping['drive_rad'], float)
        self.open_width = float(mapping['sdk_open_m'])
        if (self.gaps.shape != self.drives.shape or len(self.gaps) < 2
                or not np.isfinite(self.gaps).all() or not np.isfinite(self.drives).all()
                or not np.all(np.diff(self.gaps) < 0) or not np.all(np.diff(self.drives) > 0)):
            raise ValueError('Invalid gripper command mapping')
        self.metadata = {'tcp_definition': self.definition, 'origin': 'midpoint of custom_contact_L/R',
                         'axes': 'link7', 'offset': 'opening dependent', 'quality': 'model; not hardware calibrated'}

    def drive(self, gripper):
        g = float(gripper)
        if not np.isfinite(g) or not 0 <= g <= 1:
            raise ValueError('Gripper must be in [0,1]')
        return float(np.interp(self.open_width * (1-g), self.gaps[::-1], self.drives[::-1]))

    def flange_transform(self, gripper):
        drive = self.drive(gripper)
        frames = {'link7': np.eye(4)}
        pending = list(self.joints)
        while pending:
            progress = False
            for joint in pending[:]:
                parent = joint.find('parent').get('link')
                if parent not in frames:
                    continue
                T = origin(joint.find('origin'))
                if joint.get('type') != 'fixed':
                    mimic = joint.find('mimic')
                    if joint.get('name') != 'drive_joint' and (mimic is None or mimic.get('joint') != 'drive_joint'):
                        raise ValueError('Unexpected movable joint below link7')
                    angle = drive if mimic is None else drive*float(mimic.get('multiplier', '1'))+float(mimic.get('offset', '0'))
                    axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                    A = np.eye(4); A[:3, :3] = Rotation.from_rotvec(axis*angle).as_matrix()
                    T = T @ A
                frames[joint.find('child').get('link')] = frames[parent] @ T
                pending.remove(joint); progress = True
            if not progress:
                break
        T = np.eye(4)
        T[:3, 3] = np.mean([(frames[name] @ mount)[:3, 3] for name, mount in self.mounts.items()], axis=0)
        return T

    def actual_pose(self, body_q, labels):
        def frame(name):
            matches = [i for i, label in enumerate(labels) if label.split('/')[-1] == name]
            if len(matches) != 1:
                raise ValueError(f'Expected unique body {name}')
            value = body_q[matches[0]]
            T = np.eye(4); T[:3, :3] = Rotation.from_quat(value[3:]).as_matrix(); T[:3, 3] = value[:3]
            return T
        T = frame('link7')
        T[:3, 3] = np.mean([(frame(name) @ mount)[:3, 3] for name, mount in self.mounts.items()], axis=0)
        return T

    def require_episode(self, data):
        if 'tcp_definition' not in data or str(np.asarray(data['tcp_definition']).item()) != self.definition:
            raise ValueError('Trajectory must declare tcp_definition=sensor_center; replan old TCP trajectories')


def recorded_tcp_poses(T_sim_base, actual_tcp):
    base = rigid(T_sim_base, 'T_sim_base')
    poses = np.array([np.linalg.solve(base, rigid(t, 'actual sensor TCP')) for t in actual_tcp])
    return {'tcp_m': poses[:, :3, 3], 'tcp_quat_xyzw': Rotation.from_matrix(poses[:, :3, :3]).as_quat(),
            'tcp_definition': np.asarray(SensorTCP.definition)}
