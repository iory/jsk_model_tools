#!/usr/bin/env python

from copy import deepcopy

import rospy
import trimesh
from skrobot.model import RobotModel
import sensor_msgs.msg
import std_msgs.msg


class MeshSaver(object):

    def __init__(self):
       self.robot_model = RobotModel()
       self.robot_model.load_urdf_from_robot_description()
       self.save_meshpath = rospy.get_param('~save_meshpath')
       self.sub = rospy.Subscriber('/joint_states',
                                   sensor_msgs.msg.JointState,
                                   queue_size=1,
                                   callback=self.callback)
       self.sub_trigger = rospy.Subscriber(
           '/save_mesh',
           std_msgs.msg.Empty,
           queue_size=1,
           callback=self.save_mesh,
       )

    def callback(self, msg):
        for name, angle in zip(msg.name, msg.position):
            joint = getattr(self.robot_model, name, None)
            if joint:
                joint.joint_angle(angle)

    def save_mesh(self, msg):
        meshes = []
        link_list = [self.robot_model.root_link]
        while link_list:
            link = link_list[0]
            link_list = link_list[1:]
            tmp_meshes = []
            for v in link.visual_mesh:
                v = deepcopy(v)
                v.apply_transform(link.copy_worldcoords().T())
                tmp_meshes.append(v)
            meshes.extend(tmp_meshes)
            link_list.extend(link.child_links)
        concat_mesh = trimesh.util.concatenate(meshes)
        concat_mesh.export(self.save_meshpath)
        rospy.loginfo('Mesh saved to {}'.format(self.save_meshpath))


if __name__ == '__main__':
    rospy.init_node('mesh_saver')
    act = MeshSaver()
    rospy.spin()
