"""
This class generates a dog8 URDF file

Abbreviations:
    b = base
    l = leg
    f = foot
    h = hip
    k = knee
    dim = dimensions
    orig = origins
"""

import numpy as np
from urdf_wrapper.main import URDFWrapper as Branch


class QbotGenerator:
    """model: 'dog', 'mantis' """

    def __init__(self, b_dim, l_dim, f_dim, model):
        for arg in [b_dim, l_dim, f_dim]:
            assert isinstance(arg, tuple), "Arg: {},Type: {}".format(arg, type(arg))
            assert len(arg) is 3, "Arg: {}, Length: {}".format(arg, len(arg))
        if model == 'mantis':
            leg_axis = '0 0 1'
            l_dim = list(l_dim)
            l_dim.reverse()
            l_dim = tuple(l_dim)
        elif model == 'dog':
            leg_axis = '1 0 0'
        else:
            raise Exception("Invalid model: {}".format(model))
        file_name = model + '.urdf'

        def num_to_pos(i):
            pos = {}
            if i % 2:
                pos['x_des'] = 'right'
                pos['x'] = 1
            else:
                pos['x_des'] = 'left'
                pos['x'] = -1
            if i > 1:
                pos['y_des'] = 'front'
                pos['y'] = 1
            else:
                pos['y_des'] = 'back'
                pos['y'] = -1
            return pos
        
        # Base origins
        b_orig = np.zeros(3)

        # Hip origins
        h_orig = []
        for i in range(4):
            pos = num_to_pos(i)
            a, b, c = b_orig[0], b_orig[1], b_orig[2]
            a += (b_dim[0]*pos['x'] + l_dim[0]*pos['x'])/2
            b += b_dim[1]*pos['y']/2
            h_orig.append(np.array([a, b, c]))

        # Leg origins
        l_orig = []
        for i in range(4):
            a, b, c = 0, 0, 0 - l_dim[2] / 2
            l_orig.append(np.array([a, b, c]))

        # Knee origins
        k_orig = []
        for i in range(4):
            pos = num_to_pos(i)
            if model == 'dog':
                x_offset = 0
            elif model == 'mantis':
                x_offset = l_dim[0]*pos['x']/2
            k_orig.append(np.array([x_offset, 0, -l_dim[2]]))

        # Foot origins
        f_orig = []
        for i in range(4):
            a, b, c = 0, 0, -f_dim[2] / 2
            f_orig.append(np.array([a, b, c]))

        # Write file
        Branch(file_name=file_name,
               branch_name=model,
               branch_type='start')

        Branch(file_name=file_name,
               branch_title='MATERIALS',
               branch_type='materials')

        Branch(file_name=file_name,
               branch_name='base_link',
               origin=b_orig,
               size=b_dim,
               color='blue',
               branch_title='BASE LINK',
               branch_type='link')

        link_index = 0
        joint_index = 0
        joint_map = {}
        link_map = {}
        for i in range(4):
            pos = num_to_pos(i)
            pos_des = '_' + pos['x_des'] + '_' + pos['y_des']

            branch_name = 'leg' + pos_des
            Branch(file_name=file_name,
                   branch_name=branch_name,
                   origin=l_orig[i],
                   size=l_dim,
                   color='white',
                   branch_title='LEG LINK ' + pos_des,
                   branch_type='link')
            link_map[str(link_index)] = branch_name
            link_index += 1

            branch_name = 'hip' + pos_des
            Branch(file_name=file_name,
                   branch_name=branch_name,
                   origin=h_orig[i],
                   joint_type='revolute',
                   parent='base_link',
                   child='leg' + pos_des,
                   axis=leg_axis,
                   branch_title='HIP JOINT ' + pos_des,
                   branch_type='joint')
            joint_map[str(joint_index)] = branch_name
            joint_index += 1

            branch_name = 'foot' + pos_des
            Branch(file_name=file_name,
                   branch_name=branch_name,
                   origin=f_orig[i],
                   size=f_dim,
                   color='white',
                   branch_title='FOOT LINK ' + pos_des,
                   branch_type='link')
            link_map[str(link_index)] = branch_name
            link_index += 1

            branch_name = 'knee' + pos_des
            Branch(file_name=file_name,
                   branch_name=branch_name,
                   origin=k_orig[i],
                   joint_type='revolute',
                   parent='leg' + pos_des,
                   child='foot' + pos_des,
                   axis='1 0 0',
                   branch_title='KNEE JOINT ' + pos_des,
                   branch_type='joint')
            joint_map[str(joint_index)] = branch_name
            joint_index += 1

        Branch(file_name=file_name,
               branch_name=model,
               branch_type='end')

        self.params = {'b_dim': b_dim,
                       'l_dim': l_dim,
                       'f_dim': f_dim,
                       'model': model,
                       'b_orig': b_orig,
                       'l_orig': l_orig,
                       'f_orig': f_orig,
                       'h_orig': h_orig,
                       'k_orig': k_orig,
                       'link_map': link_map,
                       'joint_map': joint_map,
                       'file_name': file_name}
