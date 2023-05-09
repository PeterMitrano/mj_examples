#!/usr/bin/env python3
from time import sleep

import numpy as np
import rerun as rr
import mujoco
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
            torque1 = data.sensor('torque1').data
            torque2 = data.sensor('torque2').data
            arm1_origin = data.body('arm1').xpos
            arm2_origin = data.body('arm2').xpos
            torque1_dir = np.sign(np.dot(model.joint("joint1").axis, torque1))
            torque2_dir = np.sign(np.dot(model.joint("joint2").axis, torque2))
            torque1_mag_signed = torque1_dir * np.linalg.norm(torque1)
            torque2_mag_signed = torque2_dir * np.linalg.norm(torque2)
            external_torque1 = torque1_mag_signed - data.qfrc_bias[0]
            external_torque2 = torque2_mag_signed - data.qfrc_bias[1]
            arm_points = np.stack([np.zeros(3), arm1_origin, arm2_origin], axis=0)
            rr.log_line_strip('arm', arm_points)
            rr.log_arrow('torque/1', origin=arm1_origin, vector=torque1 / 5)
            rr.log_arrow('torque/2', origin=arm2_origin, vector=torque2 / 5)
            rr.log_scalar('scalars/torque/1', torque1_mag_signed)
            rr.log_scalar('scalars/torque/2', torque2_mag_signed)
            rr.log_scalar('scalars/qfrc_bias/1', data.qfrc_bias[0])
            rr.log_scalar('scalars/qfrc_bias/2', data.qfrc_bias[1])
            rr.log_scalar('scalars/external_torque/1', external_torque1)
            rr.log_scalar('scalars/external_torque/2', external_torque2)
            viewer.sync()


if __name__ == '__main__':
    main()
