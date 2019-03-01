"""This script creates URDF files.
All units assumed to be m and kg."""

import numpy as np
from numpy import pi
import os

DEBUG = False


class URDFWrapper:
    def __init__(self,
                 branch_type,
                 file_name='robot.urdf',
                 branch_title='NEW BRANCH',
                 branch_name='NEW BRANCH',
                 size=None,
                 shape='box',
                 radius=None,
                 parent=None,
                 child=None,
                 axis=None,
                 origin=None,
                 color=None,
                 joint_type=None
                 ):
        # Reflexive Fields
        if not os.path.isdir('urdf'):
            os.mkdir('urdf')
        self.parent = parent
        self.radius = radius
        self.branch_type = branch_type
        self.branch_title = branch_title
        self.file_name = 'urdf/' + file_name
        self.size = size
        self.shape = shape
        self.child = child
        self.axis = axis
        self.branch_name = branch_name
        self.origin = origin
        self.color = color
        self.joint_type = joint_type
        # None Fields
        self.mass = None
        self.inertia = None
        # Writing the branch
        self.f = open(self.file_name, "a+")
        if branch_type == 'link':
            self.write_print('\n<!--{0} -->>'.format(branch_title))
            self.link()
        elif branch_type == 'joint':
            self.write_print('\n<!--{0} -->>'.format(branch_title))
            self.write_joint()
        elif branch_type == 'materials':
            self.write_print('\n<!--{0} -->>'.format(branch_title))
            self.write_material_definitions()
        elif branch_type == 'start':
            self.f.close()
            self.f = open(self.file_name, "w+")
            self.write_print('<?xml version="1.0"?>')
            self.write_print('<robot name="{0}">'.format(branch_name))
        elif branch_type == 'end':
            self.write_print('</robot>')
        self.f.close()

    def write_print(self, string):
        """Writes to file and prints to terminal"""
        self.f.write(string + '\n')
        if DEBUG:
            print(string)
        return

    def link(self):
        size = self.size
        # Physical definitions
        if self.shape == 'box':
            volume = size[0] * size[1] * size[2]
            assert isinstance(volume, float), type(volume)
            mass = volume * 1000
            m, h, d, w = mass / 12, size[0] ** 2, size[1] ** 2, size[2] ** 2
            self.inertia = np.array([m * (h + d), m * (h + w), m * (d + w)])
        elif self.shape == 'sphere':
            r = self.radius
            volume = 4 * pi * r ** 3 / 3
            assert volume is float
            mass = volume * 1000
            value = 2 * pi * r ** 2 / 5
            self.inertia = np.array([value, value, value])
        else:
            raise Exception('Invalid Shape')
        # Write link
        self.write_print('<{0} name="{1}">'.format(self.branch_type, self.branch_name))
        self.write_visual()
        self.write_collision()
        self.write_inertial(mass)
        self.write_contact()
        self.write_print('</{0}>'.format(self.branch_type))

    def write_material_definitions(self):
        materials = {'blue': '0 0 0.8 1',
                     'white': '1 1 1 1'}
        for material in materials:
            self.write_print('<material name="{0}">'.format(material))
            self.write_print('<color rgba="{}"/>'.format(materials[material]))
            self.write_print('</material>')

    def write_joint(self):
        # Write joint
        self.write_print('<{0} name="{1}" type="{2}">'.format(self.branch_type, self.branch_name, self.joint_type))
        self.write_print('<parent link="{0}"/>'.format(self.parent))
        self.write_print('<child link="{0}"/>'.format(self.child))
        self.write_print('<axis xyz="{0}"/>'.format(self.axis))
        o0, o1, o2 = self.origin[0], self.origin[1], self.origin[2]
        self.write_print('<origin xyz="{0} {1} {2}"/>'.format(o0, o1, o2))
        self.write_print('<limit effort="1000.0" velocity="10.0" lower="-3.14" upper="3.14" />')
        self.write_print('</{0}>'.format(self.branch_type))

    def write_visual(self):
        branch = 'visual'
        self.write_print('<{0}>'.format(branch))
        self.write_geometry()
        self.write_origin()
        self.write_material()
        self.write_print('</{0}>'.format(branch))
        return

    def write_collision(self):
        branch = 'collision'
        self.write_print('<{0}>'.format(branch))
        self.write_geometry()
        self.write_origin()
        self.write_print('</{0}>'.format(branch))
        return

    def write_contact(self):
        self.write_print('<contact>')
        self.write_print('<rolling_friction value="5"/>')
        self.write_print('<spinning_friction value="5"/>')
        self.write_print('<sliding_friction value="5"/>')
        self.write_print('<lateral_friction value="5"/>')
        self.write_print('</contact>')

    def write_inertial(self, mass):
        assert isinstance(mass, float), mass
        branch = 'inertial'
        self.write_print('<{0}>'.format(branch))
        self.write_mass(mass)
        self.write_inertia()
        self.write_print('</{0}>'.format(branch))
        return

    def write_geometry(self):
        branch = 'geometry'
        self.write_print('<{0}>'.format(branch))
        if self.shape == 'box':
            self.write_print('<box size="{0:.3f} {1:.3f} {2:.3f}"/>'.format(self.size[0], self.size[1], self.size[2]))
        elif self.shape == 'sphere':
            self.write_print('<sphere radius="{0:.3f}"/>'.format(self.radius))
        self.write_print('</{0}>'.format(branch))
        return

    def write_origin(self):
        self.write_print('<origin rpy="0 0 0" xyz="{0:.3f} {1:.3f} {2:.3f}"/>'.format(self.origin[0],
                                                                                      self.origin[1],
                                                                                      self.origin[2]))
        return

    def write_material(self):
        self.write_print('<material name="{0}"/>'.format(self.color))
        return

    def write_mass(self, mass):
        assert isinstance(mass, float), mass
        self.write_print('<mass value="{0:.3f}"/>'.format(mass))
        return

    def write_inertia(self):
        self.write_print('<inertia')
        self.write_print('ixx="{}" ixy="0.0"'.format(self.inertia[0]))
        self.write_print('ixz="0.0" iyy="{}"'.format(self.inertia[1]))
        self.write_print('iyz="0.0" izz="{}"/>'.format(self.inertia[2]))
        return
