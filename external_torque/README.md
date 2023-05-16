# Measuring External Torque

This contains an example model with torque sensors and code that lets you visualize and interact with the model to see how the external torque is measured.

## Dependencies

    # make sure pip is up to date in order to install ReRun
    pip install --upgrade pip
    pip install mujoco rerun-sdk

## Run it

python ./viewer.py


Two windows will open, the MuJoCo viewer and ReRun. In ReRun you can see the sensor torques and external torques.
You can double click on one of the two links on the robot and use Ctrl+Right Click Drag to apply a force. You can also move the arm until it collides with the box using the control sliders.

