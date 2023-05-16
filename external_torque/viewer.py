#!/usr/bin/env python3
from time import sleep

import mujoco
import numpy as np
import rerun as rr
from mujoco import viewer


def main():
    np.set_printoptions(precision=3, suppress=True)
    rr.init('sim_viewer')
    rr.spawn()
    model = mujoco.MjModel.from_xml_path('test_ext.xml')
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while True:
            mujoco.mj_step(model, data, nstep=10)
            sleep(0.01)
            torque1_rpy = data.sensor('torque1').data
            torque2_rpy = data.sensor('torque2').data
            torque1 = model.joint("joint1").axis @ torque1_rpy
            torque2 = model.joint("joint2").axis @ torque2_rpy
            rr.log_scalar('torque/1', torque1, color=[255, 0, 0])
            rr.log_scalar('torque/2', torque2, color=[0, 255, 0])
            rr.log_scalar('qfrc_bias/1', data.qfrc_bias[0], color=[255, 0, 0])
            rr.log_scalar('qfrc_bias/2', data.qfrc_bias[1], color=[0, 255, 0])
            rr.log_scalar('actuator_force/1', data.actuator_force[0], color=[255, 0, 0])
            rr.log_scalar('actuator_force/2', data.actuator_force[1], color=[0, 255, 0])
            external_torque1 = torque1 - data.qfrc_bias[0]
            external_torque2 = torque2 - data.qfrc_bias[1]
            rr.log_scalar('external_torque/1', external_torque1, color=[255, 0, 0])
            rr.log_scalar('external_torque/2', external_torque2, color=[0, 255, 0])
            viewer.sync()


if __name__ == '__main__':
    main()
