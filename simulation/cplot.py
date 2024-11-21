import matplotlib.pyplot as plt
import numpy as np

def plot_main(result):
    t = result.t
    fig1 = plt.figure(figsize=(10, 8))
    ax = fig1.add_subplot(111, projection='3d')
    ax.plot(result.outputs[0], result.outputs[1], result.outputs[2])
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('Quadcopter 3D Trajectory')
    
    # Position and Velocity Plots
    fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    ax1.plot(t, result.outputs[0], label='X')
    ax1.plot(t, result.outputs[1], label='Y')
    ax1.plot(t, result.outputs[2], label='Z')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (m)')
    ax1.set_title('Position vs Time')
    ax1.legend()
    
    ax2.plot(t, result.outputs[3], label='Vx')
    ax2.plot(t, result.outputs[4], label='Vy')
    ax2.plot(t, result.outputs[5], label='Vz')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Velocity vs Time')
    ax2.legend()
    
    # Attitude Plots
    fig3, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    ax1.plot(t, np.rad2deg(result.outputs[6]), label='Roll')
    ax1.plot(t, np.rad2deg(result.outputs[7]), label='Pitch')
    ax1.plot(t, np.rad2deg(result.outputs[8]), label='Yaw')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (deg)')
    ax1.set_title('Euler Angles vs Time')
    ax1.legend()
    
    ax2.plot(t, np.rad2deg(result.outputs[9]), label='p')
    ax2.plot(t, np.rad2deg(result.outputs[10]), label='q')
    ax2.plot(t, np.rad2deg(result.outputs[11]), label='r')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angular Rate (deg/s)')
    ax2.set_title('Angular Rates vs Time')
    ax2.legend()
    
    # Control Inputs and Forces/Torques
    fig4, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    ax1.plot(t, result.outputs[12], label='r1')
    ax1.plot(t, result.outputs[13], label='r2')
    ax1.plot(t, result.outputs[14], label='r3')
    ax1.plot(t, result.outputs[15], label='r4')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Rotor Input')
    ax1.set_title('Rotor Inputs vs Time')
    ax1.legend()
    
    ax2.plot(t, result.outputs[16], label='Thrust')
    ax2.plot(t, result.outputs[17], label='τx')
    ax2.plot(t, result.outputs[18], label='τy')
    ax2.plot(t, result.outputs[19], label='τz')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Force/Torque')
    ax2.set_title('Forces and Torques vs Time')
    ax2.legend()
    
    plt.show()