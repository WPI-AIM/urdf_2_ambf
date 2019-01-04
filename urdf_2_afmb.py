# Author: Adnan Munawar
# Email: amunawar@wpi.edu
# Lab: aimlab.wpi.edu

import os
import sys
import yaml
from enum import Enum
from pathlib import Path
import xml.etree.ElementTree as ET
from PyKDL import Frame, Vector, Rotation, dot

from collections import OrderedDict


def represent_dictionary_order(self, dict_data):
    return self.represent_mapping('tag:yaml.org,2002:map', dict_data.items())


def setup_yaml():
    yaml.add_representer(OrderedDict, represent_dictionary_order)


global urdf_filepath


# Enum Class for Mesh Type
class MeshType(Enum):
    meshSTL = 0
    meshOBJ = 1
    mesh3DS = 2
    meshPLY = 3


def get_extension(val):
    if val == MeshType.meshSTL.value:
        extension = '.STL'
    elif val == MeshType.meshOBJ.value:
        extension = '.OBJ'
    elif val == MeshType.mesh3DS.value:
        extension = '.3DS'
    elif val == MeshType.meshPLY.value:
        extension = '.PLY'
    else:
        extension = '.STL'

    return extension


def to_kdl_frame(urdf_frame):
    f = Frame()
    if urdf_frame is not None:
        if 'xyz' in urdf_frame.attrib:
            xyz = [float(i) for i in urdf_frame.attrib['xyz'].split(' ')]
            for i in range(0, 3):
                f.p[i] = xyz[i]
        if 'rpy' in urdf_frame.attrib:
            rpy = [float(i) for i in urdf_frame.attrib['rpy'].split(' ')]
            f.M = Rotation.RPY(rpy[0], rpy[1], rpy[2])

    return f


def to_kdl_vec(urdf_vec):
    # Default this vector to x axis consistent with the URDF convention
    v = Vector(1.0, 0, 0)
    if urdf_vec is not None:
        xyz = [float(i) for i in urdf_vec.attrib['xyz'].split(' ')]
        for i in range(0, 3):
            v[i] = xyz[i]
    return v


def assign_xyz(yaml_vec, vec):
    yaml_vec['x'] = round(vec.x(), 3)
    yaml_vec['y'] = round(vec.y(), 3)
    yaml_vec['z'] = round(vec.z(), 3)


def skew_mat(v):
    return Rotation(0, -v[2], v[1],
                    v[2], 0, -v[0],
                    -v[1], v[0], 0)


def compute_parent_pivot_and_axis(parent_body, joint):
    if parent_body.visual_offset != parent_body.collision_offset:
        print('%s WARNING: VISUAL AND COLLISION ORIGINS DONT MATCH' % parent_body.afmb_data['name'])
        print('Visual Mesh Name: %s' % parent_body.afmb_data['mesh'])
        print('Collision Mesh Name: %s' % parent_body.afmb_data['collision mesh'])
        print('VISUAL FRAME: %s' % parent_body.visual_offset.p)
        print('COLLISION FRAME: %s' % parent_body.collision_offset.p)
    parent_temp_frame = parent_body.visual_offset.Inverse() * joint.origin
    parent_pivot = parent_temp_frame.p
    parent_axis = parent_temp_frame.M * joint.axis
    return parent_pivot, parent_axis


# Since the URDF uses 4 frames for child links, 2 for our use, we have to account
# for that while computing child axis
def compute_child_pivot_and_axis(child_body, joint):
    if child_body.visual_offset != child_body.collision_offset:
        print('%s WARNING: VISUAL AND COLLISION ORIGINS DONT MATCH' % child_body.afmb_data['name'])
        print('Visual Mesh Name: %s' % child_body.afmb_data['mesh'])
        print('Collision Mesh Name: %s' % child_body.afmb_data['collision mesh'])
        print('VISUAL FRAME: %s' % child_body.visual_offset.p)
        print('COLLISION FRAME: %s' % child_body.collision_offset.p)
    child_temp_frame = child_body.visual_offset
    inv_child_temp_frame = child_temp_frame.Inverse()
    child_pivot = inv_child_temp_frame.p
    child_axis = inv_child_temp_frame.M * joint.axis
    return child_pivot, child_axis


def add_mat(mat1, mat2):
    out = Rotation()
    for i in range(0, 3):
        for j in range(0, 3):
            out[i, j] = mat1[i, j] + mat2[i, j]
    return out


def scalar_mul(mat, s):
    out = Rotation()
    for i in range(0, 3):
        for j in range(0, 3):
            out[i, j] = mat[i, j] * s
    return out


# https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/897677#897677
def rot_matrix_from_vecs(v1, v2):
    out = Rotation()
    vcross = v1 * v2
    vdot = dot(v1, v2)
    if 1.0 - vdot < 0.01:
        return out
    elif 1.0 + vdot < 0.01:
        print 'IMPLEMENT THIS'
    else:
        skew_v = skew_mat(vcross)
        out = add_mat(add_mat(Rotation(), skew_v), scalar_mul(
            skew_v * skew_v, (1 - vdot) / (vcross.Norm() ** 2)))
    return out


# Body Template for the some commonly used of afBody's data
class BodyTemplate:
    def __init__(self):
        self.afmb_data = {'name': "",
                          'mesh': "",
                          'mass': 0.0,
                          'scale': 1.0,
                          'location': {
                             'position': {'x': 0, 'y': 0, 'z': 0},
                             'orientation': {'r': 0, 'p': 0, 'y': 0}},
                          'color': 'random'}
        self.inertial_offset = Frame()
        self.visual_offset = Frame()
        self.collision_offset = Frame()


# Joint Template for the some commonly used of afJoint's data
class JointTemplate:
    def __init__(self):
        self.afmb_data = {'name': '',
                          'parent': '',
                          'child': '',
                          'parent axis': {'x': 0, 'y': 0.0, 'z': 1.0},
                          'parent pivot': {'x': 0, 'y': 0.0, 'z': 0},
                          'child axis': {'x': 0, 'y': 0.0, 'z': 1.0},
                          'child pivot': {'x': 0, 'y': 0.0, 'z': 0},
                          'joint limits': {'low': -1.2, 'high': 1.2},
                          'enable motor': 0,
                          'max motor impulse': 0.05}
        self.origin = Vector()
        self.axis = Vector(0.0, 0.0, 1.0)


class CreateAFYAML:

    def __init__(self, ignore_inertial_offset=True, ignore_inertias=True):
        self._afmb_yaml = None
        self._body_names_list = []
        self._joint_names_list = []
        self._bodies_map = {}
        self.mesh_resource_path = ''
        self.col_mesh_resource_path = ''
        self._ros_packages = {}
        self._ignore_inertial_offsets = ignore_inertial_offset
        self._ignore_inertias = ignore_inertias
        self._save_as = ''
        self.body_name_prefix = 'BODY '
        self.joint_name_prefix = 'JOINT '

    def get_body_prefixed_name(self, urdf_body_str):
        return self.body_name_prefix + urdf_body_str

    def get_joint_prefixed_name(self, urdf_joint_str):
        return self.joint_name_prefix + urdf_joint_str

    def get_path_from_user_input(self, filepath):
        if sys.version_info[0] < 3:
            package_path = raw_input("Mesh Filepath is relative to \"%s\" \n "
                                     "Please enter the full path to \"%s\" package: "
                                     % (filepath.parts[1], filepath.parts[1]))
        else:
            package_path = input("Mesh Filepath is relative to \"%s\" \n "
                                 "Please enter the full path to \"%s\" package: "
                                 % (filepath.parts[1], filepath.parts[1]))
        valid_path = False
        while not valid_path:
            if os.path.exists(package_path):
                valid_path = True
            else:
                if sys.version_info[0] < 3:
                    package_path = raw_input("Path %s doesn't exist, please re-enter the path "
                                             "to package %s, or enter \'x\' to leave it blank : "
                                             % (package_path, filepath.parts[1]))
                else:
                    package_path = input("Path %s doesn't exist, please re-enter the path"
                                         " to package %s, or enter \'x\' to leave it blank : "
                                         % (package_path, filepath.parts[1]))
                if package_path == 'x':
                    print('Please manually set the proper path in the \"%s\" config file' % self._save_as)
                    package_path = filepath.parts[0]
                    valid_path = True

        self._ros_packages[filepath.parts[1]] = package_path
        return package_path

    def get_path_and_file_name(self, filepath):
        global urdf_filepath
        filepath = Path(filepath)
        if filepath.parts[0] == 'package:':

            package_already_defined = False

            for packageName, packagePath in self._ros_packages.items():
                if packageName == filepath.parts[1]:
                    package_path = packagePath
                    package_already_defined = True
                    break

            if not package_already_defined:
                package_path = self.get_path_from_user_input(filepath)

            rel_path = '/'.join(filepath.parts[i] for i in range(2, len(filepath.parts)))
            abs_mesh_filepath = os.path.join(package_path, rel_path)
            abs_mesh_dir = os.path.dirname(abs_mesh_filepath) + '/'
            filename = filepath.name
            # print (abs_mesh_filepath)
        elif filepath.parts[0] == '/':
            abs_mesh_dir = os.path.dirname(str(filepath)) + '/'
            filename = filepath.name
        else:
            # This means that the part is relative to the location of the URDF
            abs_mesh_dir = os.path.join(os.path.dirname(urdf_filepath), os.path.dirname(str(filepath))) + '/'
            filename = filepath.name

        if not os.path.exists(os.path.join(abs_mesh_dir, filename)):
            print('WARNING, \"%s\" DOES NOT EXISTS ON THIS SYSTEM, PLEASE CORRECT'
                  ' THE PATH IN \"%s\"' % (os.path.join(abs_mesh_dir, filename), self._save_as))

        return abs_mesh_dir, filename

    def load_body_data(self, afmb_yaml, urdf_link):
        body = BodyTemplate()
        body_data = body.afmb_data
        body_data['name'] = urdf_link.attrib['name']

        #if urdf_link.attrib['name'] == 'world':
         #   return
        body_yaml_name = self.get_body_prefixed_name(urdf_link.attrib['name'])

        visual_urdf = urdf_link.find('visual')
        collision_urdf = urdf_link.find('collision')
        inertial_urdf = urdf_link.find('inertial')
        if visual_urdf is not None:
            abs_mesh_path, filename = self.get_path_and_file_name(visual_urdf.find('geometry').find('mesh').attrib['filename'])
            if collision_urdf is not None:
                abs_col_mesh_path, col_filename = self.get_path_and_file_name(collision_urdf.find('geometry').find('mesh').attrib['filename'])
            else:
                abs_col_mesh_path = abs_mesh_path
                col_filename = filename

            # Sanity check to include the high and low res paths for each body only if they are different
            if not self.mesh_resource_path:
                self.mesh_resource_path = abs_mesh_path
                afmb_yaml['high resolution path'] = abs_mesh_path

            elif self.mesh_resource_path != abs_mesh_path:
                body_data['high resolution path'] = abs_mesh_path

            if not self.col_mesh_resource_path:
                self.col_mesh_resource_path = abs_col_mesh_path
                afmb_yaml['low resolution path'] = abs_col_mesh_path

            elif self.col_mesh_resource_path != abs_col_mesh_path:
                body_data['low resolution path'] = abs_col_mesh_path

            temp_mesh_name = Path(filename)

            if temp_mesh_name.suffix in ('.ply', '.PLY', '.dae', '.DAE'):
                body_data['mesh'] = col_filename
            else:
                body_data['mesh'] = filename
            body_data['collision mesh'] = col_filename

            body.visual_offset = to_kdl_frame(visual_urdf.find('origin'))

            if collision_urdf is not None:
                body.collision_offset = to_kdl_frame(collision_urdf.find('origin'))

                if (body.visual_offset.p - body.collision_offset.p).Norm() >= 0.001 or\
                        (body.visual_offset.M != body.collision_offset.M):
                        print 'WARNING: BODY \"', body_data['name'],\
                            '\" - VISUAL AND COLLISION OFFSETs ARE DIFFERENT'
                        print 'Visual Offset: '
                        print body.visual_offset
                        print 'Collision Offset: '
                        print body.collision_offset

        if inertial_urdf is not None:
            mass = round(float(inertial_urdf.find('mass').attrib['value']), 3)
            if mass == 0.0:
                body_data['mass'] = 0.001
            else:
                body_data['mass'] = mass

            if inertial_urdf.find('inertia') is not None and not self._ignore_inertias:
                body_data['inertial'] = {'ix': 0.0, 'iy': 0.0, 'iz': 0.0}
                body_data['inertial']['ix'] = float(inertial_urdf.find('inertia').attrib['ixx'])
                body_data['inertial']['iy'] = float(inertial_urdf.find('inertia').attrib['iyy'])
                body_data['inertial']['iz'] = float(inertial_urdf.find('inertia').attrib['izz'])

            if inertial_urdf.find('origin') is not None and not self._ignore_inertial_offsets:
                body.inertial_offset = to_kdl_frame(inertial_urdf.find('origin'))

                body_data['inertial offset'] = {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
                inertial_off_pos = body_data['inertial offset']['position']
                assign_xyz(inertial_off_pos, body.inertial_offset.p)

                body_data['inertial offset'] = {'orientation': {'r': 0.0, 'p': 0.0, 'y': 0.0}}
                inertial_off_rot = body_data['inertial offset']['orientation']
                inertial_off_rot['r'] = round(body.inertial_offset.M.GetRPY()[0], 3)
                inertial_off_rot['p'] = round(body.inertial_offset.M.GetRPY()[1], 3)
                inertial_off_rot['y'] = round(body.inertial_offset.M.GetRPY()[2], 3)

        else:

            body_data['mass'] = 0.0

        self._bodies_map[urdf_link.attrib['name']] = body

        afmb_yaml[body_yaml_name] = body_data
        self._body_names_list.append(body_yaml_name)
        # print(body_data)

    def load_joint_data(self, afmb_yaml, urdf_joint):
        if urdf_joint.attrib['type'] == 'fixed':
            if urdf_joint.find('parent').attrib['link'] == 'world':
                afmb_yaml[self.get_body_prefixed_name(urdf_joint.find('child').attrib['link'])]['mass'] = 0.0
                print('Setting Mass of ', urdf_joint.find('child').attrib['link'], ' to 0.0')
                return

        joint_yaml_name = self.get_joint_prefixed_name(urdf_joint.attrib['name'])

        if urdf_joint.attrib['type'] in ['revolute', 'continuous', 'prismatic']:
            joint = JointTemplate()
            joint_data = joint.afmb_data
            parent_body = self._bodies_map[urdf_joint.find('parent').attrib['link']]
            child_body = self._bodies_map[urdf_joint.find('child').attrib['link']]
            joint_data['name'] = urdf_joint.attrib['name']
            joint_data['type'] = urdf_joint.attrib['type']
            joint_data['parent'] = self.get_body_prefixed_name(urdf_joint.find('parent').attrib['link'])
            joint_data['child'] = self.get_body_prefixed_name(urdf_joint.find('child').attrib['link'])
            joint.origin = to_kdl_frame(urdf_joint.find('origin'))
            joint.axis = to_kdl_vec(urdf_joint.find('axis'))

            parent_temp_frame = parent_body.visual_offset.Inverse() * joint.origin
            parent_pivot = parent_temp_frame.p
            parent_axis = parent_temp_frame.M * joint.axis
            parent_pivot_data = joint_data["parent pivot"]
            parent_axis_data = joint_data["parent axis"]

            assign_xyz(parent_pivot_data, parent_pivot)
            assign_xyz(parent_axis_data, parent_axis)

            child_temp_frame = child_body.visual_offset
            inv_child_temp_frame = child_temp_frame.Inverse()
            child_pivot = inv_child_temp_frame.p
            child_axis = inv_child_temp_frame.M * joint.axis

            # The use of pivot and axis does not fully define the connection and relative transform between two bodies
            # it is very likely that we need an additional offset of the child body as in most of the cases of URDF's
            # For this purpose, we calculate the offset as follows
            r_c_p_afmb = rot_matrix_from_vecs(child_axis, parent_axis)
            r_c_p_urdf = joint.origin.M * child_body.visual_offset.M

            r_angular_offset = r_c_p_afmb.Inverse() * r_c_p_urdf

            offset_axis_angle = r_angular_offset.GetRotAngle()
            # print(axis_angle[0]),
            # print(round(axis_angle[1][0], 1), round(axis_angle[1][1], 1), round(axis_angle[1][2], 1))

            if abs(offset_axis_angle[0]) > 0.001:
                # print '*****************************'
                # print joint_data['name']
                # print 'Joint Axis, '
                # print '\t', joint.axis
                # print 'Offset Axis'
                # print '\t', offset_axis_angle[1]
                offset_angle = offset_axis_angle[0]
                offset_angle = round(offset_angle, 3)
                # print 'Offset Angle: \t', offset_angle

                if abs(1.0 - dot(joint.axis, offset_axis_angle[1])) < 0.01:
                    joint_data['offset'] = offset_angle
                    # print ': SAME DIRECTION'
                elif abs(1.0 + dot(joint.axis, offset_axis_angle[1])) < 0.01:
                    joint_data['offset'] = -offset_angle
                    # print ': OPPOSITE DIRECTION'
                else:
                    print 'ERROR: SHOULD\'NT GET HERE'

            child_pivot_data = joint_data["child pivot"]
            child_axis_data = joint_data["child axis"]

            # There is a bug in bullet discussed here:
            # https: // github.com / bulletphysics / bullet3 / issues / 2031
            # As a work around, we want to tweak the axis or body masses just a bit
            # It's better to tweak masses than axes
            if parent_body.afmb_data['mass'] > 0.0:
                factA = 1.0 / parent_body.afmb_data['mass']
                if child_body.afmb_data['mass'] > 0.0:
                    factB = 1.0 / child_body.afmb_data['mass']
                    weighted_axis = factA * parent_axis + factB * child_axis
                    if weighted_axis.Norm() < 0.001:
                        print("Weighted Axis for joint \"%s\" is zero, to avoid breaking Bullet, "
                              "increasing the mass of parent body \"%s\" and decreasing the mass"
                              " of child body \"%s\" by 1%%"
                              % (joint.afmb_data['name'], parent_body.afmb_data['name'], child_body.afmb_data['name']))
                        parent_body.afmb_data['mass'] = parent_body.afmb_data['mass'] * 1.01
                        child_body.afmb_data['mass'] = child_body.afmb_data['mass'] * 0.99

            assign_xyz(child_pivot_data, child_pivot)
            assign_xyz(child_axis_data, child_axis)

            urdf_joint_limit = urdf_joint.find('limit')
            if urdf_joint_limit is not None:
                joint_limit_data = joint_data["joint limits"]
                joint_limit_data['low'] = round(float(urdf_joint_limit.attrib['lower']), 3)
                joint_limit_data['high'] = round(float(urdf_joint_limit.attrib['upper']), 3)

            joint_data['enable motor'] = 1

            afmb_yaml[joint_yaml_name] = joint_data
            self._joint_names_list.append(joint_yaml_name)
            # print(jointData)

    def generate_afmb_yaml(self, urdf_robot):
        urdf_links = urdf_robot.findall('link')
        urdf_joints = urdf_robot.findall('joint')

        self._afmb_yaml = OrderedDict()

        # For inorder processing, set the bodies and joints tag at the top of the map
        self._afmb_yaml['bodies'] = []
        self._afmb_yaml['joints'] = []

        self._afmb_yaml['high resolution path'] = ""
        self._afmb_yaml['low resolution path'] = ""

        self._afmb_yaml['ignore inter-collision'] = 'True'

        for urdf_link in urdf_links:
            self.load_body_data(self._afmb_yaml, urdf_link)

        for urdf_joint in urdf_joints:
            self.load_joint_data(self._afmb_yaml, urdf_joint)

        # Now populate the bodies and joints tag
        self._afmb_yaml['bodies'] = self._body_names_list
        self._afmb_yaml['joints'] = self._joint_names_list

        print('SUCCESSFULLY GENERATED')

    def save_afmb_yaml(self, output_file):
        self._save_as = output_file
        file_name = os.path.basename(self._save_as)
        save_path = os.path.dirname(self._save_as)
        if not file_name:
            file_name = 'default.yaml'
        output_file_name = os.path.join(save_path, file_name)
        output_file = open(output_file_name, 'w')
        yaml.dump(self._afmb_yaml, output_file)
        print('Saved to: \"%s\"', output_file_name)

    def print_afmb_yaml(self):
        print (self._afmb_yaml)


def main():
    global urdf_filepath
    setup_yaml()
    if len(sys.argv) > 1:
        urdf_filepath = sys.argv[1]
        if os.path.exists(urdf_filepath):
            print("Specified File: \"%s\"", urdf_filepath)
        else:
            print("Filepath: \"%s\" does not exist on this machine, exiting", urdf_filepath)
            exit()
    else:
        print("No URDF File Specified")
        exit()
    root = ET.parse(urdf_filepath)
    robot = root.getroot()
    # This flag is to set to ignore the collision between all the the bodies in this MultiBody
    af_multi_body_config = CreateAFYAML(ignore_inertial_offset=True)
    af_multi_body_config.generate_afmb_yaml(robot)

    # If two arguments already specified, save to the second argument
    save_to = ''
    if len(sys.argv) > 2:
        save_to = sys.argv[2]
        af_multi_body_config.save_afmb_yaml(save_to)
    elif save_to != '':
        af_multi_body_config.save_afmb_yaml(save_to)
    else:
        # Give one more chance to save to a file or give option to print to console
        if sys.version_info[0] < 3:
            save_to = raw_input("Specify filepath to save AFMB or enter \'x\' to print to console: ")
        else:
            save_to = input("Specify filepath to save AFMB or enter \'x\' to print to console: ")

        if save_to == 'x':
            af_multi_body_config.print_afmb_yaml()
        else:
            af_multi_body_config.save_afmb_yaml(save_to)


if __name__ == "__main__":
    main()
