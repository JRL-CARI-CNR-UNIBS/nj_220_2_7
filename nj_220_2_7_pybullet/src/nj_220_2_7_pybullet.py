#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from threading import Thread
from rosgraph_msgs.msg import Clock

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def jointStatePublisher(robot_id, js_publishers, joint_states, joint_state_publish_rate, joint_name_to_index):
    name = []
    position = []
    velocity = []
    effort = []
    rate = rospy.Rate(joint_state_publish_rate)
    while not rospy.is_shutdown():
        for robot_name in js_publishers.keys():
            joint_states[robot_name] = p.getJointStates(robot_id[robot_name], range(p.getNumJoints(robot_id[robot_name])))
            name.clear()
            position.clear()
            velocity.clear()
            effort.clear()
            for joint_name in joint_name_to_index[robot_name].keys():
                name.append(joint_name)
                position.append(joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][0])
                velocity.append(joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][1])
                effort.append(joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][3])
            js_msg = JointState()
            js_msg.header = Header()
            js_msg.header.stamp = rospy.Time.now() # ???
            js_msg.name = name
            js_msg.position = position
            js_msg.velocity = velocity
            js_msg.effort = effort
            js_publishers[robot_name].publish(js_msg)
        rate.sleep()


def jointTargetSubscriber(data, args):
    robot_id = args[0]
    joint_name_to_index = args[1]
    for joint_id in range(len(data.name)):
        p.setJointMotorControl2(robot_id, joint_name_to_index[data.name[joint_id]], p.POSITION_CONTROL, data.position[joint_id])

def main():
    rospy.init_node('pybullet_simulation')

    robot_names = []
    print(bcolors.OKGREEN + 'Wait for robot names' + bcolors.ENDC)
    ready = False
    init_rate = rospy.Rate(100)
    init_time = 0.0
    while not ready:
        if (init_time >= 15.0):
            print(bcolors.FAIL + 'No /robots param' + bcolors.ENDC)
            raise SystemExit
        if rospy.has_param('/robots'):
            robot_names = rospy.get_param('/robots')
            print(bcolors.OKGREEN + 'Robot:' + bcolors.ENDC)
            for robot_name in robot_names:
                print(bcolors.OKGREEN + ' - ' + robot_name + bcolors.ENDC)
            ready = True
        init_time += 1/100
        init_rate.sleep()

    joint_target = {}
    js_publishers = {}

    print(bcolors.OKGREEN + ' Topic generated:' + bcolors.ENDC)
    for robot_name in robot_names:
        js_topic = '/'+robot_name+'/joint_states'
        js_publishers[robot_name] = rospy.Publisher('/'+robot_name+'/joint_states', JointState, queue_size=1)
        print(bcolors.OKGREEN + ' - ' + js_topic + bcolors.ENDC)

    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  
    planeId = p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)

    controlled_joint_name = {}
    robot_id = {}
    link_name_to_index = {}
    joint_name_to_index = {}
    joint_state_publish_rate = None
    simulation_step_time = None

    if rospy.has_param('/simulation_step_time'):
        simulation_step_time = rospy.get_param('/simulation_step_time')
        print(bcolors.OKGREEN + 'simulation_step_time: ' + str(simulation_step_time) + bcolors.ENDC)
    else:
        print(bcolors.FAIL + 'No param /simulation_step_time' + bcolors.ENDC)
        raise SystemExit

    if rospy.has_param('/joint_state_publish_rate'):
        joint_state_publish_rate = rospy.get_param('/joint_state_publish_rate')
        print(bcolors.OKGREEN + 'joint_state_publish_rate: ' + str(joint_state_publish_rate) + bcolors.ENDC)
    else:
        print(bcolors.FAIL + 'No param /joint_state_publish_rate' + bcolors.ENDC)
        raise SystemExit

    for robot_name in robot_names:
        print(bcolors.OKGREEN + 'For robot ' + robot_name + ':' + bcolors.ENDC)
        if rospy.has_param('/'+robot_name):
            robot_info = rospy.get_param('/'+robot_name)
            if 'foulder_path' in robot_info:
                p.setAdditionalSearchPath(robot_info['foulder_path'])  
                print(bcolors.OKGREEN + '  foulder_path: ' + robot_info['foulder_path'] + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /'+robot_name+'/foulder_path' + bcolors.ENDC)
                raise SystemExit
            if 'file_name' in robot_info:
                name = robot_info['file_name']
                print(bcolors.OKGREEN + '  file_name: ' + robot_info['file_name'] + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /'+robot_name+'/file_name' + bcolors.ENDC)
                raise SystemExit
            if 'start_position' in robot_info:
                startPos = robot_info['start_position'] 
                print(bcolors.OKGREEN + '  startPos: ['
                                      + str(robot_info['start_position'][0]) + ','
                                      + str(robot_info['start_position'][1]) + ','
                                      + str(robot_info['start_position'][2]) + ']'
                                      + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /'+robot_name+'/start_position' + bcolors.ENDC)
                raise SystemExit
            if 'start_orientation' in robot_info:
                startOrientation = p.getQuaternionFromEuler(robot_info['start_orientation'])
                print(bcolors.OKGREEN + '  start_orientation: ['
                                      + str(robot_info['start_orientation'][0]) + ','
                                      + str(robot_info['start_orientation'][1]) + ','
                                      + str(robot_info['start_orientation'][2]) + ']'
                                      + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /'+robot_name+'/start_orientation' + bcolors.ENDC)
                raise SystemExit
            if 'controlled_joint_name' in robot_info:
                controlled_joint_name[robot_name] = robot_info['controlled_joint_name']
                array_str = bcolors.OKGREEN + '  controlled_joint_name: ['
                for joint_controlled_name in robot_info['controlled_joint_name']:
                    array_str += joint_controlled_name + ' '
                array_str += ']' + bcolors.ENDC
                print(array_str)
            else:
                print(bcolors.FAIL + 'No param /'+robot_name+'/controller_joint_name' + bcolors.ENDC)
                raise SystemExit
            robot_id[robot_name] = p.loadURDF(name, startPos, startOrientation, 0, 1)
            print(bcolors.OKGREEN + '  robot_id: ' + str(robot_id[robot_name]) + bcolors.ENDC)

            link_name_to_index[robot_name] = {p.getBodyInfo(robot_id[robot_name])[0].decode('UTF-8'): -1, }
            joint_name_to_index[robot_name] = {}
            for joint_id in range(p.getNumJoints(robot_id[robot_name])):
                link_name = p.getJointInfo(robot_id[robot_name], joint_id)[12].decode('UTF-8')
                link_name_to_index[robot_name][link_name] = joint_id
                joint_name = p.getJointInfo(robot_id[robot_name], joint_id)[1].decode('UTF-8')
                joint_name_to_index[robot_name][joint_name] = joint_id

            print(bcolors.OKGREEN + '  link_name & link_id: ' + bcolors.ENDC)
            array_str = bcolors.OKGREEN
            for link_name in link_name_to_index[robot_name].keys():
                print(bcolors.OKGREEN + '    - ' + link_name + ': ' + str(link_name_to_index[robot_name][link_name]) + bcolors.ENDC)

            print(bcolors.OKGREEN + '  joint_name & joint_id: ')
            for joint_name in joint_name_to_index[robot_name].keys():
                print(bcolors.OKGREEN + '    - ' + joint_name + ': ' + str(joint_name_to_index[robot_name][joint_name]) + bcolors.ENDC)


            if 'constraints' in robot_info:
                constraints = robot_info['constraints']
                print(bcolors.OKGREEN + '  constraints: ' + bcolors.ENDC)
                for constraint in constraints:                    
                    if 'parent_body' in constraint:
                        parent_body = constraint['parent_body']
                        print(bcolors.OKGREEN + '    - parent_body: ' + parent_body + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /'+robot_name+'/constraint/parent_body' + bcolors.ENDC)
                        raise SystemExit
                    if 'parent_link' in constraint:
                        parent_link = constraint['parent_link']
                        print(bcolors.OKGREEN + '      parent_link: ' + parent_link + bcolors.ENDC)
                    else:
                        print('No param /'+robot_name+'/constraint/parent_link')
                        print(bcolors.FAIL + 'No /robots param' + bcolors.ENDC)
                        raise SystemExit
                    if 'child_body' in constraint:
                        child_body = constraint['child_body']
                        print(bcolors.OKGREEN + '      child_body: ' + child_body + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /'+robot_name+'/constraint/child_body' + bcolors.ENDC)
                        raise SystemExit
                    if 'child_link' in constraint:
                        child_link = constraint['child_link']
                        print(bcolors.OKGREEN + '      child_link: ' + child_link + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /'+robot_name+'/constraint/child_link' + bcolors.ENDC)
                        raise SystemExit
                    if 'type' in constraint:
                        type = constraint['type']
                        print(bcolors.OKGREEN + '      type: ' + type + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /'+robot_name+'/constraint/type' + bcolors.ENDC)
                        raise SystemExit
                    if 'axis' in constraint:
                        axis = constraint['axis']
                        print(bcolors.OKGREEN + '      axis: ['
                                              + str(axis[0]) + ','
                                              + str(axis[1]) + ','
                                              + str(axis[2]) + ']'
                                              + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /'+robot_name+'/constraint/axis' + bcolors.ENDC)
                        raise SystemExit
                    if 'parent_frame_position' in constraint:
                        parent_frame_position = constraint['parent_frame_position']
                        print(bcolors.OKGREEN + '      parent_frame_position: ['
                                              + str(parent_frame_position[0]) + ','
                                              + str(parent_frame_position[1]) + ','
                                              + str(parent_frame_position[2]) + ']'
                                              + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /'+robot_name+'/constraint/parent_frame_position' + bcolors.ENDC)
                        raise SystemExit
                    if 'child_frame_position' in constraint:
                        child_frame_position = constraint['child_frame_position']
                        print(bcolors.OKGREEN + '      child_frame_position: ['
                                              + str(child_frame_position[0]) + ','
                                              + str(child_frame_position[1]) + ','
                                              + str(child_frame_position[2]) + ']'
                                              + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /'+robot_name+'/constraint/child_frame_position' + bcolors.ENDC)
                        raise SystemExit
                    if ( type == 'prismatic'):
                        p.createConstraint(robot_id[parent_body],
                                           link_name_to_index[robot_name][parent_link],
                                           robot_id[child_body],
                                           link_name_to_index[robot_name][child_link],
                                           p.JOINT_PRISMATIC,
                                           axis,
                                           parent_frame_position,
                                           child_frame_position)
                    elif ( type == 'fixed'):
                        p.createConstraint(robot_id[parent_body],
                                           link_name_to_index[robot_name][parent_link],
                                           robot_id[child_body],
                                           link_name_to_index[robot_name][child_link],
                                           p.JOINT_FIXED,
                                           axis,
                                           parent_frame_position,
                                           child_frame_position)
                    elif ( type == 'point2point'):
                        p.createConstraint(robot_id[parent_body],
                                           link_name_to_index[robot_name][parent_link],
                                           robot_id[child_body],
                                           link_name_to_index[robot_name][child_link],
                                           p.JOINT_POINT2POINT,
                                           axis,
                                           parent_frame_position,
                                           child_frame_position)
                    elif ( type == 'gear'):
                        p.createConstraint(robot_id[parent_body],
                                           link_name_to_index[robot_name][parent_link],
                                           robot_id[child_body],
                                           link_name_to_index[robot_name][child_link],
                                           p.JOINT_GEAR,
                                           axis,
                                           parent_frame_position,
                                           child_frame_position)
                    else:
                        print(bcolors.FAIL + 'Constraint type not foud' + bcolors.ENDC)
                        raise SystemExit
            else:
                print(bcolors.WARNING + 'No param /'+robot_name+'/constraints' + bcolors.ENDC)

            jt_topic = '/'+robot_name+'/joint_target'
            rospy.Subscriber(jt_topic, JointState, jointTargetSubscriber, (robot_id[robot_name], joint_name_to_index[robot_name]), queue_size=1)
            #        rospy.Subscriber('/'+robot_name+'/joint_target', JointState, jointTargetSubscriber, robot_name)
        else:
            print(bcolors.FAIL + '/'+robot_name+' param not found' + bcolors.ENDC)
            raise SystemExit

    joint_states = {}
    for robot_name in robot_names:
        joint_states[robot_name] = p.getJointStates(robot_id[robot_name], range(p.getNumJoints(robot_id[robot_name])))
        for joint_name in joint_name_to_index[robot_name].keys():
            if joint_name in controlled_joint_name[robot_name]:
                p.setJointMotorControl2(robot_id[robot_name],
                                        joint_name_to_index[robot_name][joint_name],
                                        p.POSITION_CONTROL,
                                        0.0)
            else:
                p.setJointMotorControl2(robot_id[robot_name],
                                        joint_name_to_index[robot_name][joint_name],
                                        p.VELOCITY_CONTROL,
                                        0.0,
                                        0.0)

    js_pub_thread = Thread(target=jointStatePublisher, args=(robot_id, js_publishers, joint_states, joint_state_publish_rate, joint_name_to_index))
    js_pub_thread.start()

    rate = rospy.Rate(1/simulation_step_time)
    simulation_time = 0.0
    time_pub = rospy.Publisher('/clock', Clock, queue_size=10)
    while not rospy.is_shutdown():
        p.stepSimulation()
        simulation_time += simulation_step_time
        simulation_time_msg = rospy.Time(simulation_time)
        time_pub.publish(simulation_time_msg)
        rate.sleep()
    p.disconnect()

    print(bcolors.OKGREEN + 'Waiting for the thread...' + bcolors.ENDC)
    js_pub_thread.join()



if __name__ == '__main__':
    main()


