Generates URDF files compatible with ROS and PyBullet for Qbot

To use:

from urdf_wrapper import QbotGenerator

b_dim = [0.3, 0.6, 0.1]
l_dim = [0.05, 0.05, 0.1]
f_dim = [0.05, 0.05, 0.1]
model = 'dog' #or mantis
qbot = QbotGenerator(b_dim, l_dim, f_dim, model)
print(qbot.params)