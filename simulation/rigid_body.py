import control as ct
import numpy as np

from rotors import rotor_map, rotor_map_inv

def dynamics(t, x, u, params):
    # Unpack states
    x, y, z = x[0], x[1], x[2]
    v_x, v_y, v_z = x[3], x[4], x[5]
    phi, theta, psi = x[6], x[7], x[8]
    p, q, r = x[9], x[10], x[11]

    # Unpack inputs
    thrust, torque_x, torque_y, torque_z = u[0], u[1], u[2], u[3]

    # Unpack parameters
    mass = params['mass']
    gravity = params['gravity']
    arm_length = params['arm_length']
    drag_coefficient = params['drag_coefficient']

    # Dynamics equations
    # Translational dynamics
    acceleration_x = (thrust / mass) * (sin(orientation_theta) * cos(orientation_psi)) - drag_coefficient * velocity_x
    acceleration_y = (thrust / mass) * (sin(orientation_theta) * sin(orientation_psi)) - drag_coefficient * velocity_y
    acceleration_z = (thrust / mass) * cos(orientation_theta) - gravity - drag_coefficient * velocity_z

    # Rotational dynamics
    angular_acceleration_x = torque_x / (mass * arm_length)
    angular_acceleration_y = torque_y / (mass * arm_length)
    angular_acceleration_z = torque_z / (mass * arm_length)

    # Return state derivatives
    return [acceleration_x, acceleration_y, acceleration_z,
            angular_velocity_x, angular_velocity_y, angular_velocity_z,
            angular_acceleration_x, angular_acceleration_y, angular_acceleration_z]

def outputs(t, x, u, params):
    # Unpack states
    position_x, position_y, position_z = x[0], x[1], x[2]

    # Return outputs
    return [position_x, position_y, position_z]

quadcopter_nonlinear = ct.nlsys(updfcn=dynamics, outfcn=outputs, states=9, inputs=4, outputs=3, name='quadcopter_nonlinear')

quadcopter_nonlinear.set_states(['velocity_x', 'velocity_y', 'velocity_z', 'orientation_phi', 'orientation_theta', 'orientation_psi', 'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z'])
quadcopter_nonlinear.set_inputs(['thrust', 'torque_x', 'torque_y', 'torque_z'])
quadcopter_nonlinear.set_outputs(['position_x', 'position_y', 'position_z'])


if __name__ == '__main__':
	# Time vector
	t = np.arange(0.0, 10.0, 0.01)
	
	# Parameters
	quadcopter_nonlinear.params = {
	    'mass': 1.5,
	    'gravity': 9.81,
	    'arm_length': 0.25,
	    'drag_coefficient': 0.1
	}
	
	# Initial conditions
	x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
	
	# Define input functions
	u0 = lambda t: 1.5 * 9.81
	u1 = lambda t: 0
	u2 = lambda t: 0
	u3 = lambda t: 0
	
	# Combine input functions
	U = np.vstack([
	    np.array([u0(ti) for ti in t]).T,
	    np.array([u1(ti) for ti in t]).T,
	    np.array([u2(ti) for ti in t]).T,
	    np.array([u3(ti) for ti in t]).T
	])
	
	# Run input-output response simulation
	result = ct.input_output_response(quadcopter_nonlinear, T=t, U=U, X0=x0)
	
	# Plot results
	p = result.plot()
	p.figure.show()
	