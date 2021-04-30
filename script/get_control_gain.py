#!/usr/bin/env/ python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
from scipy import linalg
import yaml
import datetime

#倒立振子モデル 参照 "倒立２輪ロボットの安定化制御" 佐藤光 木澤悟

if __name__ =='__main__':
    g = 9.80665
    with open('robot_param.yaml','r') as file:
        robot_param = yaml.load(file, Loader=yaml.SafeLoader)

        # body param
        body_mass = robot_param["body_mass"]
        body_to_wheel_dist = robot_param["body_to_wheel_dist"]
        body_inertia = robot_param["body_inertia"]
        body_vis_fric = robot_param["body_vis_fric"]

        # wheel param
        wheel_mass = robot_param["wheel_mass"]
        wheel_radius = robot_param["wheel_radius"]
        wheel_inertia = robot_param["wheel_inertia"]
        wheel_vis_fric = robot_param["wheel_vis_fric"]

        # motor param
        motor_inertia = robot_param["motor_inertia"]
        reduction_ratio = robot_param["reduction_ratio"]
        motor_resist = robot_param["motor_resist"]
        torque_const = robot_param["torque_const"]
        induced_volt_const = robot_param["induced_volt_const"]

        #lqr param
        Q = robot_param["Q"]
        R = robot_param["R"]

    # Calc control model matrix
    a = (body_mass + wheel_mass)*wheel_radius**2 + wheel_inertia
    b = wheel_mass * wheel_radius * body_to_wheel_dist
    c = wheel_mass * body_to_wheel_dist**2 + body_inertia
    d = wheel_mass * g * body_to_wheel_dist
    E = a + motor_inertia * reduction_ratio**2
    F = a + b
    G = body_vis_fric + (torque_const * induced_volt_const * reduction_ratio**2)/motor_resist
    H = a + 2*b + c

    a21 = -(E * d)/(F**2-E*H)
    a22 = (E*body_vis_fric)/(F**2-E*H)
    a23 = -(F*G)/(F**2-E*H)
    a31 = (F*d)/(F**2-E*H)
    a32 = (F*body_vis_fric)/(F**2-E*H)
    a33 = (G*H)/(F**2-E*H)
    b2 = (E*torque_const*reduction_ratio)/(motor_resist*(F**2-E*H))
    b3 = -(H*torque_const*reduction_ratio)/(motor_resist*(F**2-E*H))

    A = np.array([ 
            [0, 1, 0],
            [a21, a22, a23],
            [a31, a32, a33]
        ])
    B = np.array([
            [0],
            [b2],
            [b3]
        ])
    
    C = np.array([
            [0, 0, 1]
        ])

    AB = np.append(A, B, axis=1)
    Ce = np.append(C, np.zeros((1,1)), axis=1)
    Ae = np.append(AB,np.zeros((1,4)),axis=0)
    Be = np.array([
        [0],
        [0],
        [0],
        [1]
    ])

    # Solve lqr
    Q = np.array(Q)
    Pe = linalg.solve_continuous_are(Ae, Be, Q, R)
    Ke = R * Be.T.dot(Pe)
    AB = Ae[0:3,0:4]
    ABC0 = np.append(AB, Ce, axis=0)
    K = Ke.dot(linalg.inv(ABC0))

    # Save gain
    config = {
        "control_gain":[
            float(K[0][0]),
            float(K[0][1]),
            float(K[0][2]),
            float(K[0][3])
        ]
    }

    now = datetime.datetime.now()
    title = 'control_param_' + now.strftime('%Y%m%d_%H%M%S') + '.yaml'
    with open(title,'w') as file:
        yaml.dump(config, file)

    print "Saved control_gain to", title