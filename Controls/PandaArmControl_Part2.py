import mujoco as mj
from mujoco import viewer
import numpy as np
import math
import quaternion


# Set the XML filepath
xml_filepath = "../franka_emika_panda/panda_nohand_torque_sine_board.xml"

################################# Control Callback Definitions #############################

def get_board_control(model, data):

    # Instantite a handle to the desired body on the robot
    omega = 0.4
    data.ctrl[7] = 0.15*np.sin(omega*data.time) + 0.15

# Control callback for gravity compensation
def gravity_comp(model, data):

    # data.ctrl exposes the member that sets the actuator control inputs that participate in the
    # physics, data.qfrc_bias exposes the gravity forces expressed in generalized coordinates, i.e.
    # as torques about the joints

    data.ctrl[:7] = data.qfrc_bias[:7]

# Force control callback
def force_control(model, data):

    # Copy the code that you used to implement the controller from Part1 here

    
    # DO NOT CHANGE ANY THING BELOW THIS IN THIS FUNCTION
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    bodyid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'hand')
    mj.mj_jacBody(model, data, jacp, jacr, bodyid)
    jacobian = np.concatenate((jacp, jacr))[:, :7]
    desired_force = np.array([15, 0, 0, 0, 0, 0])
    control = np.array(jacobian.T @ desired_force).squeeze()
    
    data.ctrl[:7] = data.qfrc_bias[:7] + control

    
    global i
    print(i)
    i = i + 1
    
    if i < 8000:
        force[:] = np.roll(force, -1)[:]
        force[-1] = data.sensordata[2]

    # Update control inputs to the whiteboard
    get_board_control(model, data)
    
# Control callback for an impedance controller
def impedance_control(model, data):

    # Copy the code that you used to implement the controller from Part1 here


    # DO NOT CHANGE ANY THING BELOW THIS IN THIS FUNCTION
    # Implement an impedance control callback here that generates a force of 15 N along the global x-axis,
    # i.e. the x-axis of the robot arm base. You can use the comments as prompts or use your own flow
    # of code. The comments are simply meant to be a reference.

    # Instantite a handle to the desired body on the robot


    # Set the desired position



    # Set the desired velocities


    # Set the desired orientation (Use numpy quaternion manipulation functions)


    # Get the current orientation


    # Get orientation error


    # Get the position error
    body = data.body("hand")
    
    Kd = 10
    Kp = 100
    pdes = np.array([15/Kp + body.xpos[0], body.xpos[1], body.xpos[2], 0, 0, 0])
    perr = pdes - np.concatenate((body.xpos, [0], [0], [0]))

    # Set the desired joint angle positions
    # Set the desired joint velocities
    desired_joint_velocities = np.array([0,0,0,0, 0,0])
    
    # Desired gain on position error (K_p)
    
    # Desired gain on velocity error (K_d)

    # Set the actuator control torques
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    bodyid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'hand')
    mj.mj_jacBody(model, data, jacp, jacr, bodyid)
    jacobian = np.concatenate((jacp, jacr))[:, :7]
    handVel = np.zeros((6, ))
    mj.mj_objectVelocity(model, data, mj.mjtObj.mjOBJ_BODY, bodyid, handVel[:6], True)
    handVel[3:] = 0
    
    data.ctrl[:7] = data.qfrc_bias[:7] + jacobian.T @ (Kp*(perr) + Kd * (desired_joint_velocities.flatten() - handVel.flatten()))

    # Update force sensor readings

    # Update force sensor readings
    global i
    print(i)
    i = i + 1
    
    if i < 8000:
        force[:] = np.roll(force, -1)[:]
        force[-1] = data.sensordata[2]
    
    # Update control inputs to the whiteboard
    get_board_control(model, data)

def position_control(model, data):

    # Instantite a handle to the desired body on the robot
    body = data.body("hand")

    # Set the desired joint angle positions
    desired_joint_positions = np.array([0,0,0,-1.57079,0,1.57079,-0.7853])

    # Set the desired joint velocities
    desired_joint_velocities = np.array([0,0,0,0,0,0,0])

    # Desired gain on position error (K_p)
    Kp = 1000

    # Desired gain on velocity error (K_d)
    Kd = 1000

    # Set the actuator control torques
    data.ctrl[:7] = data.qfrc_bias[:7] + Kp*(desired_joint_positions-data.qpos[:7]) + Kd*(np.array([0,0,0,0,0,0,0])-data.qvel[:7])







####################################### MAIN #####################################


if __name__ == "__main__":
    
    # Load the xml file here
    model = mj.MjModel.from_xml_path(xml_filepath)
    data = mj.MjData(model)

    # Set the simulation scene to the home configuration
    mj.mj_resetDataKeyframe(model, data, 0)

    ################################# Swap Callback Below This Line #################################
    # This is where you can set the control callback. Take a look at the Mujoco documentation for more
    # details. Very briefly, at every timestep, a user-defined callback function can be provided to
    # mujoco that sets the control inputs to the actuator elements in the model. The gravity
    # compensation callback has been implemented for you. Run the file and play with the model as
    # explained in the PDF

    mj.set_mjcb_control(force_control) #TODO:

    ################################# Swap Callback Above This Line #################################
    i = 0
    # Initialize variables to store force and time data points
    force_sensor_max_time = 16
    force = np.zeros(int(force_sensor_max_time/model.opt.timestep))
    time = np.linspace(0, force_sensor_max_time, int(force_sensor_max_time/model.opt.timestep))

    # Launch the simulate viewer
    viewer.launch(model, data)   

    # Save recorded force and time points as a csv file
    force = np.reshape(force, (8000, 1))
    time = np.reshape(time, (8000, 1))
    plot = np.concatenate((time, force), axis=1)
    # np.savetxt('force_vs_time_imp_force.csv', plot, delimiter=',')