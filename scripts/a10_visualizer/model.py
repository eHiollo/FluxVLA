"""Load the A10_fixed meshes and compose their poses from URDF and feedback."""

import math
from pathlib import PurePosixPath
import struct
import xml.etree.ElementTree as ET
import zipfile


A10_LINKS = ('base_link', *(f'link{i}' for i in range(1, 7)))
A10_JOINT_SIGNS = (-1., -1., -1., 1., 1., 1.)


def rotate(point, quat):
    x, y, z = point
    qx, qy, qz, qw = quat
    tx = 2 * (qy * z - qz * y)
    ty = 2 * (qz * x - qx * z)
    tz = 2 * (qx * y - qy * x)
    return (x + qw * tx + qy * tz - qz * ty,
            y + qw * ty + qz * tx - qx * tz,
            z + qw * tz + qx * ty - qy * tx)


def compose(first, second):
    x, y, z, w = first[3:]
    u, v, t, s = second[3:]
    dx, dy, dz = rotate(second[:3], first[3:])
    return (first[0] + dx, first[1] + dy, first[2] + dz,
            w*u + x*s + y*t - z*v, w*v - x*t + y*s + z*u,
            w*t + x*v - y*u + z*s, w*s - x*u - y*v - z*t)


def axis_quaternion(axis, radians):
    scale = math.sin(radians / 2)
    return (*(a * scale for a in axis), math.cos(radians / 2))


def numbers(node, key, default):
    if node is None or node.get(key) is None:
        return default
    result = tuple(float(value) for value in node.attrib[key].split())
    if len(result) != len(default) or not all(math.isfinite(value) for value in result):
        raise ValueError(f'URDF 的 {key} 数值无效')
    return result


def origin_pose(node):
    xyz = numbers(node, 'xyz', (0., 0., 0.))
    roll, pitch, yaw = numbers(node, 'rpy', (0., 0., 0.))
    qx = (0., 0., 0., *axis_quaternion((1., 0., 0.), roll))
    qy = (0., 0., 0., *axis_quaternion((0., 1., 0.), pitch))
    qz = (*xyz, *axis_quaternion((0., 0., 1.), yaw))
    return compose(compose(qz, qy), qx)


class A10MeshModel:
    def __init__(self, archive):
        self.visuals = []
        self.joints = []
        self.meshes = {}
        with zipfile.ZipFile(archive) as package:
            root = ET.fromstring(package.read('A10_fixed/urdf/A10_fixed.urdf'))
            for index, name in enumerate(A10_LINKS):
                link = root.find(f"./link[@name='{name}']")
                visual = None if link is None else link.find('visual')
                mesh = None if visual is None else visual.find('geometry/mesh')
                if mesh is None:
                    raise ValueError(f'URDF 缺少 {name} 的实体网格')
                filename = PurePosixPath(mesh.attrib['filename']).name
                data = package.read(f'A10_fixed/meshes/{filename}')
                if len(data) < 84 or len(data) != 84 + 50 * struct.unpack_from('<I', data, 80)[0]:
                    raise ValueError(f'{filename} 不是有效的二进制 STL')
                url = f'/model/{name}.stl'
                self.meshes[url] = data
                color = numbers(visual.find('material/color'), 'rgba', (.81, .84, .88, 1.))
                scale = numbers(mesh, 'scale', (1., 1., 1.))
                if any(value <= 0 for value in scale):
                    raise ValueError(f'{filename} 网格比例必须为正数')
                self.visuals.append(dict(url=url, color=color[:3], scale=scale,
                                         pose=origin_pose(visual.find('origin'))))
                if index:
                    joint = root.find(f"./joint[@name='joint{index}']")
                    if (joint is None or joint.get('type') not in ('revolute', 'continuous')
                            or joint.find('parent').get('link') != A10_LINKS[index - 1]
                            or joint.find('child').get('link') != name):
                        raise ValueError(f'URDF 的 joint{index} 与 A10 连杆顺序不符')
                    axis = numbers(joint.find('axis'), 'xyz', (1., 0., 0.))
                    length = math.hypot(*axis)
                    if length < 1e-8:
                        raise ValueError(f'URDF 的 joint{index} 旋转轴无效')
                    self.joints.append((origin_pose(joint.find('origin')),
                                        tuple(value / length for value in axis)))

    def link_poses(self, actual_deg, base_pose):
        pose = base_pose
        result = [compose(pose, self.visuals[0]['pose'])]
        for index, (angle_deg, sign, (origin, axis)) in enumerate(
                zip(actual_deg, A10_JOINT_SIGNS, self.joints), start=1):
            spin = axis_quaternion(axis, math.radians(angle_deg * sign))
            pose = compose(compose(pose, origin), (0., 0., 0., *spin))
            result.append(compose(pose, self.visuals[index]['pose']))
        return result
