# Author: Adnan Munawar
# Email: amunawar@wpi.edu
# Lab: aimlab.wpi.edu


import yaml
import os
from enum import Enum
from pathlib2 import Path
import xml.etree.ElementTree as ET

from PyKDL import Frame, Vector, Rotation
from collections import OrderedDict


def represent_dictionary_order(self, dict_data):
    return self.represent_mapping('tag:yaml.org,2002:map', dict_data.items())


def setup_yaml():
    yaml.add_representer(OrderedDict, represent_dictionary_order)


global urdf_path


# Enum Class for Mesh Type
class MeshType(Enum):
    meshSTL = 0
    meshOBJ = 1
    mesh3DS = 2
    meshPLY = 3


def getExtension(val):
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


# Body Template for the some commonly used of afBody's data
class BodyTemplate:
    def __init__(self):
        self.data = {'name': "",
                     'mesh': "",
                     'mass': 0.0,
                     'scale': 1.0,
                     'location': {
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'r': 0, 'p': 0, 'y': 0}},
                     'color': 'random'}
        self.origin = None
        self.inertial_offset = None
        self.visual_offset = None
        self.collision_offset = None


# Joint Template for the some commonly used of afJoint's data
class JointTemplate:
    def __init__(self):
        self.data = {'name': '',
                     'parent': '',
                     'child': '',
                     'parent axis': {'x': 0, 'y': 0.0, 'z': 1.0},
                     'parent pivot': {'x': 0, 'y': 0.0, 'z': 0},
                     'child axis': {'x': 0, 'y': 0.0, 'z': 1.0},
                     'child pivot': {'x': 0, 'y': 0.0, 'z': 0},
                     'joint limits': {'low': -1.2, 'high': 1.2},
                     'enable motor': 0,
                     'max motor impulse': 0}
        self.origin = None
        self.axis = None


class CreateAFYAML:

    def __init__(self):
        self.bodiesNameList = []
        self.jointNamesList = []
        self.bodiesMap = {}
        self.mesh_resource_path = ''
        self.col_mesh_resource_path = ''
        self._ros_packages = {}
        self._save_as = ''
        self.bodyPrefix = 'BODY '
        self.jointPrefix = 'JOINT '

    def computeLocalCOM(self, obj):
        vcos = [ v.co for v in obj.data.vertices ]
        findCenter = lambda l: (max(l) + min(l)) / 2
        x, y, z = [ [ v[i] for v in vcos ] for i in range(3) ]
        center = [ findCenter(axis) for axis in [x,y,z] ]
        return center

    def toKDLFrame(self, urdfFrame):
        f = Frame()
        if urdfFrame is not None:
            xyz = [float(i) for i in urdfFrame.attrib['xyz'].split(' ')]
            rpy = [float(i) for i in urdfFrame.attrib['rpy'].split(' ')]
            for i in range(0, 3):
                f.p[i] = xyz[i]
            f.M = Rotation.RPY(rpy[0], rpy[1], rpy[2])

        return f

    def toKDLVec(self, urdfVec):
        # Default this vector to x axis consistent with the URDF convention
        v = Vector(1.0, 0, 0)
        if urdfVec is not None:
            xyz = [float(i) for i in urdfVec.attrib['xyz'].split(' ')]
            for i in range(0, 3):
                v[i] = xyz[i]
        return v

    def getPathFromUserInput(self, filepath):
        package_path = raw_input("Mesh Filepath is relative to \"%s\" \n "
                                 "Please enter the full path to \"%s\" package: "
                                 % (filepath.parts[1], filepath.parts[1]))
        valid_path = False
        while not valid_path:
            if os.path.exists(package_path):
                valid_path = True
            else:
                package_path = raw_input("Path %s doesn't exist, please re-enter the path"
                                         " to package %s, or enter \'x\' to leave it blank : "
                                         % (package_path, filepath.parts[1]))
                if package_path == 'x':
                    print('Please manually set the proper path in the \"%s\" config file' % self._save_as)
                    package_path = filepath.parts[0]
                    valid_path = True

        self._ros_packages[filepath.parts[1]] = package_path
        return package_path

    def getPathandFileName(self, urdf_filepath):
        global urdf_path
        filepath = Path(urdf_filepath)
        if filepath.parts[0] == 'package:':

            package_already_defined = False

            for packageName, packagePath in self._ros_packages.iteritems():
                if packageName == filepath.parts[1]:
                    package_path = packagePath
                    package_already_defined = True
                    break

            if not package_already_defined:
                package_path = self.getPathFromUserInput(filepath)

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
            abs_mesh_dir = os.path.join(os.path.dirname(urdf_path), os.path.dirname(str(filepath))) + '/'
            filename = filepath.name

        if not os.path.exists(os.path.join(abs_mesh_dir, filename)):
            print('WARNING, \"%s\" DOES NOT EXISTS ON THIS SYSTEM, PLEASE CORRECT'
                  ' THE PATH IN \"%s\"' % (os.path.join(abs_mesh_dir, filename), self._save_as))

        return abs_mesh_dir, filename

    def loadBodyData(self, afYAML, urdfLink):
        body = BodyTemplate()
        bodyData = body.data
        bodyData['name'] = urdfLink.attrib['name']
        if urdfLink.attrib['name'] == 'world':
            return
        visualUrdf = urdfLink.find('visual')
        collisionUrdf = urdfLink.find('collision')
        inertialUrdf = urdfLink.find('inertial')
        abs_mesh_path, filename = self.getPathandFileName(visualUrdf.find('geometry').find('mesh').attrib['filename'])
        abs_col_mesh_path, col_filename = self.getPathandFileName(collisionUrdf.find('geometry').find('mesh').attrib['filename'])

        # Sanity check to include the high and low res paths to each body only if they are different
        if not self.mesh_resource_path:
            self.mesh_resource_path = abs_mesh_path
            afYAML['high resolution path'] = abs_mesh_path
        elif self.mesh_resource_path != abs_mesh_path:
            bodyData['high resolution path'] = abs_mesh_path

        if not self.col_mesh_resource_path:
            self.col_mesh_resource_path = abs_col_mesh_path
            afYAML['low resolution path'] = abs_col_mesh_path
        elif self.col_mesh_resource_path != abs_col_mesh_path:
            bodyData['low resolution path'] = abs_col_mesh_path

        temp_mesh_name = Path(filename)
        if temp_mesh_name.suffix in ('.ply', '.PLY', '.dae', '.DAE'):
            bodyData['mesh'] = col_filename
        else:
            bodyData['mesh'] = filename
        bodyData['collision mesh'] = col_filename

        body.visual_offset = self.toKDLFrame(visualUrdf.find('origin'))
        body.collision_offset = self.toKDLFrame(collisionUrdf.find('origin'))
        body.inertial_offset = self.toKDLFrame(inertialUrdf.find('origin'))

        # inertial_off_pos = bodyData['inertial offset']['position']
        # inertial_off_rot = bodyData['inertial offset']['orientation']
        # inertial_off_pos['x'] = round(body.inertial_offset.p[0], 3)
        # inertial_off_pos['y'] = round(body.inertial_offset.p[1], 3)
        # inertial_off_pos['z'] = round(body.inertial_offset.p[2], 3)
        #
        # inertial_off_rot['r'] = round(body.inertial_offset.M.GetRPY()[0], 3)
        # inertial_off_rot['p'] = round(body.inertial_offset.M.GetRPY()[1], 3)
        # inertial_off_rot['y'] = round(body.inertial_offset.M.GetRPY()[2], 3)

        bodyData['mass'] = round(float(inertialUrdf.find('mass').attrib['value']), 3)
        #if urdfLink.inertial is not None:
        #    bodyData['inertial'] = {'ix': 0, 'iy': 0, 'iz': 0}
        #    bodyData['inertial']['ix'] = urdfLink.inertial.inertia.ixx
        #    bodyData['inertial']['iy'] = urdfLink.inertial.inertia.iyy
        #    bodyData['inertial']['iz'] = urdfLink.inertial.inertia.izz
        afYAML[self.bodyPrefix + urdfLink.attrib['name']] = bodyData
        self.bodiesMap[urdfLink.attrib['name']] = body
        #print(bodyData)

    def loadJointData(self, afYAML, urdfJoint):
        if urdfJoint.attrib['type'] == 'fixed':
            if urdfJoint.find('parent').attrib['link'] == 'world':
                afYAML[self.bodyPrefix + urdfJoint.find('child').attrib['link']]['mass'] = 0.0
                print 'Setting Mass of ', urdfJoint.find('child').attrib['link'], ' to 0.0'

        if urdfJoint.attrib['type'] == 'revolute':
            joint = JointTemplate()
            jointData = joint.data
            parentBody = self.bodiesMap[urdfJoint.find('parent').attrib['link']]
            childBody = self.bodiesMap[urdfJoint.find('child').attrib['link']]
            jointData['name'] = urdfJoint.attrib['name']
            jointData['parent'] = self.bodyPrefix + urdfJoint.find('parent').attrib['link']
            jointData['child'] = self.bodyPrefix + urdfJoint.find('child').attrib['link']
            joint.origin = self.toKDLFrame(urdfJoint.find('origin'))
            joint.axis = self.toKDLVec(urdfJoint.find('axis'))
            # Get the axis in parent's frame
            parent_temp_frame = parentBody.visual_offset.Inverse() * joint.origin
            parent_pivot = parent_temp_frame.p
            parent_axis = parent_temp_frame.M * joint.axis
            parentPivotData = jointData["parent pivot"]
            parentAxisData = jointData["parent axis"]
            parentPivotData['x'] = round(parent_pivot.x(), 3)
            parentPivotData['y'] = round(parent_pivot.y(), 3)
            parentPivotData['z'] = round(parent_pivot.z(), 3)
            parentAxisData['x'] = round(parent_axis.x(), 3)
            parentAxisData['y'] = round(parent_axis.y(), 3)
            parentAxisData['z'] = round(parent_axis.z(), 3)
            childPivotData = jointData["child pivot"]
            childAxisData = jointData["child axis"]
            child_temp_frame = childBody.visual_offset
            inv_child_temp_frame = child_temp_frame.Inverse()
            child_pivot = inv_child_temp_frame.p
            child_axis = inv_child_temp_frame.M * joint.axis
            childPivotData['x'] = round(child_pivot.x(), 3)
            childPivotData['y'] = round(child_pivot.y(), 3)
            childPivotData['z'] = round(child_pivot.z(), 3)
            childAxisData['x'] = round(child_axis.x(), 3)
            childAxisData['y'] = round(child_axis.y(), 3)
            childAxisData['z'] = round(child_axis.z(), 3)
            jointLimitData = jointData["joint limits"]
            jointLimitData['low'] = round(float(urdfJoint.find('limit').attrib['lower']), 3)
            jointLimitData['high'] = round(float(urdfJoint.find('limit').attrib['upper']), 3)
            afYAML[self.jointPrefix + urdfJoint.attrib['name']] = jointData
            #print(jointData)

    def computeParentPivotAndAxis(self, parentBody, joint, urdfJoint):
        if parentBody.visual_offset != childBody.collision_offset:
            print('%s WARNING: VISUAL AND COLLISION ORIGINS DONT MATCH' % childBody.data['name'])
            print('Visual Mesh Name: %s' % childBody.data['mesh'])
            print('Collision Mesh Name: %s' % childBody.data['collision mesh'])
            print('VISUAL FRAME: %s' %childBody.visual_offset.p)
            print('COLLISION FRAME: %s' % childBody.collision_offset.p)
        invJointFrame = parentBody.visual_offset.Inverse() * parentBody.origin.Inverse()
        parentPivot = invJointFrame.p
        parentAxis = invJointFrame.M * joint.axis
        return parentPivot, parentAxis

    # Since the URDF uses 4 frames for child links, 2 for our use, we have to account
    # for that while computing child axis
    def computeChildPivotAndAxis(self, childBody, joint, urdfJoint):
        if childBody.visual_offset != childBody.collision_offset:
            print('%s WARNING: VISUAL AND COLLISION ORIGINS DONT MATCH' % childBody.data['name'])
            print('Visual Mesh Name: %s' % childBody.data['mesh'])
            print('Collision Mesh Name: %s' % childBody.data['collision mesh'])
            print('VISUAL FRAME: %s' %childBody.visual_offset.p)
            print('COLLISION FRAME: %s' % childBody.collision_offset.p)
        invJointFrame = childBody.origin.Inverse()
        childPivot = invJointFrame.p
        childAxis = invJointFrame.M * joint.axis
        return childPivot, childAxis

    def saveAFYAML(self, urdfRobot):
        urdfLinks = urdfRobot.findall('link')
        urdfJoints = urdfRobot.findall('joint')

        for urdfLink in urdfLinks:
            self.bodiesNameList.append(self.bodyPrefix + urdfLink.attrib['name'])

        for urdfJoint in urdfJoints:
            if urdfJoint.attrib['name'] != 'fixed':
                self.jointNamesList.append(self.jointPrefix + urdfJoint.attrib['name'])

        self._save_as = '/home/adnan/chai3d/modules/BULLET/bin/resources/config/puzzles/urdf-mtm2.yaml'
        #save_to = '/home/adnan/chai3d/modules/BULLET/bin/resources/config/puzzles/urdf-kuka.yaml'
        fileName = os.path.basename(self._save_as)
        savePath = os.path.dirname(self._save_as)
        if not fileName:
            fileName = 'default.yaml'
        outputFileName = os.path.join(savePath, fileName)
        outputFile = open(outputFileName, 'w')
        print('Output filename is: ', outputFileName)
        afYAML = OrderedDict()

        afYAML['bodies'] = self.bodiesNameList
        afYAML['joints'] = self.jointNamesList

        afYAML['high resolution path'] = ""
        afYAML['low resolution path'] = ""
        for urdfLink in urdfLinks:
            self.loadBodyData(afYAML, urdfLink)

        for urdfJoint in urdfJoints:
           self.loadJointData(afYAML, urdfJoint)

        #print(afYAML)
        yaml.dump(afYAML, outputFile)


def main():
    global urdf_path
    setup_yaml()
    urdf_path = '/home/adnan/dvrk_ws/src/dvrk-ros/dvrk_model/model/mtm.urdf'
    #urdf_path = '/home/adnan/bullet3/data/kuka_lwr/kuka.urdf'
    root = ET.parse(urdf_path)
    robot = root.getroot()
    afMultiBodyConfig = CreateAFYAML()
    afMultiBodyConfig.saveAFYAML(robot)


if __name__ == "__main__":
    main()
