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
            leg_axis = '0 1 0'
        else:
            model = 'dog'
            leg_axis = '1 0 0'
        file_name = model + '.urdf'
        
        # Base origins
        b_orig = np.zeros(3)

        # Hip origins
        h_orig = []
        for i in range(4):
            bi = format(i, '02b')
            x = int(bi[0])
            y = int(bi[1])
            a, b, c = b_orig[0], b_orig[1], b_orig[2]
            a += 0.5 * b_dim[0] * (-1) ** x + 0.5 * l_dim[0] * (-1) ** x
            b += 0.5 * b_dim[1] * (-1) ** y
            c += 0
            h_orig.append(np.array([a, b, c]))

        # Leg origins
        l_orig = []
        for i in range(4):
            a, b, c = 0, 0, 0 - l_dim[2] / 2
            l_orig.append(np.array([a, b, c]))

        # Knee origins
        k_orig = []
        for i in range(4):
            if not f_dim:
                continue
            a, b, c = 0, 0, -l_dim[2]
            k_orig.append(np.array([a, b, c]))

        # Foot origins
        f_orig = []
        for i in range(4):
            if not f_dim:
                continue
            a, b, c = 0, 0, -f_dim[2] / 2
            f_orig.append(np.array([a, b, c]))

        self.b_orig = b_orig
        self.h_orig = h_orig
        self.l_orig = l_orig
        self.k_orig = k_orig
        self.f_orig = f_orig

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

        leg_index = []
        foot_index = []
        link_index = 0
        for i in range(4):
            bi = format(i, '02b')
            Branch(file_name=file_name,
                   branch_name='leg' + bi,
                   origin=l_orig[i],
                   size=l_dim,
                   color='white',
                   branch_title='LEG LINK ' + str(i),
                   branch_type='link')
            Branch(file_name=file_name,
                   branch_name='hip' + bi,
                   origin=h_orig[i],
                   joint_type='revolute',
                   parent='base_link',
                   child='leg' + bi,
                   axis=leg_axis,
                   branch_title='HIP JOINT ' + str(i),
                   branch_type='joint')
            leg_index.append(link_index)
            link_index += 1
            Branch(file_name=file_name,
                   branch_name='foot' + bi,
                   origin=f_orig[i],
                   size=f_dim,
                   color='white',
                   branch_title='FOOT LINK ' + str(i),
                   branch_type='link')
            Branch(file_name=file_name,
                   branch_name='knee' + bi,
                   origin=k_orig[i],
                   joint_type='revolute',
                   parent='leg' + bi,
                   child='foot' + bi,
                   axis='1 0 0',
                   branch_title='KNEE JOINT ' + str(i),
                   branch_type='joint')
            foot_index.append(link_index)
            link_index += 1

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
                       'leg_index': leg_index,
                       'foot_index': foot_index,
                       'file_name': file_name}
