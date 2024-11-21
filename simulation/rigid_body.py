import control as ct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from rotors import rotor_map, rotor_map_inv
from utilities import body_to_inertial, inertial_to_body, euler_rates

def dynamics(t, x, u, params):
    # Unpack states
    x_e, y_e, z_e = x[0], x[1], x[2] # inertial position
    v_x, v_y, v_z = x[3], x[4], x[5] # inertial velocity
    phi, theta, psi = x[6], x[7], x[8] # orientation
    p, q, r = x[9], x[10], x[11] # angular velocity

    velocity = np.array([v_x, v_y, v_z])
    omega = np.array([p, q, r])

    r1, r2, r3, r4 = u[0], u[1], u[2], u[3] # rotor inputs

    # Unpack parameters
    mass = params['mass']
    gravity = params['gravity']
    arm_length = params['arm_length']
    cd = params['cd']
    density = params['density']
    area = params['area']
    inertia = params['inertia']

    v_body = inertial_to_body(phi, theta, psi) @ np.array([v_x, v_y, v_z])
    drag_body = 0.5 * density * v_body**2 * area * cd # assume same area and drag coefficient for all sides
    drag_inertial = body_to_inertial(phi, theta, psi) @ drag_body
    gravity_inertial = np.array([0, 0, -gravity]) * mass

    T = rotor_map @ np.array([r1, r2, r3, r4])**2
    thrust = T[0]
    torque_x = T[1]
    torque_y = T[2]
    torque_z = T[3]

    force_inertial = np.array([0, 0, thrust]) + drag_inertial + gravity_inertial
    moment_body = np.array([torque_x, torque_y, torque_z])

    acceleration = force_inertial / mass
    angular_acceleration = np.linalg.inv(inertia) @ (moment_body + np.cross(omega, inertia @ omega))
    euler_dot = euler_rates(phi, theta, psi) @ omega

    return [velocity, acceleration, euler_dot, angular_acceleration]

def outputs(t, x, u, params):
    # Unpack states
    x_e, y_e, z_e = x[0], x[1], x[2]

    r1, r2, r3, r4 = u[0], u[1], u[2], u[3]

    # add thrust and torques as an output
    T = rotor_map @ np.array([r1, r2, r3, r4])**2
    thrust = T[0]
    torque_x = T[1]
    torque_y = T[2]
    torque_z = T[3]

    # Return outputs
    return np.hstack([x, u, [thrust, torque_x, torque_y, torque_z]])

quadcopter_nonlinear = ct.nlsys(updfcn=dynamics, outfcn=outputs, states=12, inputs=4, outputs=20, name='quadcopter_nonlinear')

quadcopter_nonlinear.set_states(['pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z', 'phi', 'theta', 'psi', 'p', 'q', 'r'])
quadcopter_nonlinear.set_inputs(['r1', 'r2', 'r3', 'r4'])
quadcopter_nonlinear.set_outputs(['pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z', 'phi', 'theta', 'psi', 'p', 'q', 'r', 'r1', 'r2', 'r3', 'r4', 'thrust', 'torque_x', 'torque_y', 'torque_z'])


if __name__ == '__main__':
	# Time vector
	t = np.arange(0.0, 5.0, 0.01)
	
	# Parameters
	quadcopter_nonlinear.params = {
	    'mass': 2.0,
	    'gravity': 9.81,
	    'arm_length': 0.25,
        'density': 1.225, # kg/m^3
	    'cd': 1.5, # drag coefficient
        'area': 0.02, # m^2
        'inertia': np.diag([0.0023, 0.0023, 0.004]) # kg m^2
	}
	
	# Initial conditions
	x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
	
	# Define input functions
	u0 = lambda t: 1
	u1 = lambda t: 1
	u2 = lambda t: 1
	u3 = lambda t: 1
	
	# Combine input functions
	U = np.vstack([
	    np.array([u0(ti) for ti in t]).T,
	    np.array([u1(ti) for ti in t]).T,
	    np.array([u2(ti) for ti in t]).T,
	    np.array([u3(ti) for ti in t]).T
	])
	
	# Run input-output response simulation
	result = ct.input_output_response(quadcopter_nonlinear, T=t, U=U, X0=x0)

	from cplot import plot_main
	plot_main(result)