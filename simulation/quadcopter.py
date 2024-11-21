import control as ct
import numpy as np
from quadcopter_rotor_conversion import quadcopter_rotor_conversion
from quadcopter_nonlinear import quadcopter_nonlinear


quadcopter_inputs = ['quadcopter_rotor_conversion.omega']
quadcopter_input_labels = ['omega']
quadcopter_outputs = ['quadcopter_nonlinear.position_x', 'quadcopter_nonlinear.position_y', 'quadcopter_nonlinear.position_z']
quadcopter_output_labels = ['position_x', 'position_y', 'position_z']

quadcopter = ct.interconnect(
	(quadcopter_rotor_conversion, quadcopter_nonlinear),
	name = 'quadcopter',
	connections = (
		['quadcopter_nonlinear.thrust', 'quadcopter_rotor_conversion.thrust'],
		['quadcopter_nonlinear.torque_x', 'quadcopter_rotor_conversion.torque'],
		['quadcopter_nonlinear.torque_y', 'quadcopter_rotor_conversion.torque'],
		['quadcopter_nonlinear.torque_z', 'quadcopter_rotor_conversion.torque']
	),
	inplist = quadcopter_inputs,
	inputs = quadcopter_input_labels,
	outlist = quadcopter_outputs,
	outputs = quadcopter_output_labels
)

if __name__ == '__main__':
	# Time vector
	t = np.arange(0.0, 10.0, 0.01)
	
	# Parameters
	quadcopter_rotor_conversion.params = {
	    'kf': 1e-06,
	    'kd': 1e-08,
	    'l': 0.15,
	    'w_max': 500.0
	}
	
	quadcopter_nonlinear.params = {
	    'mass': 1.5,
	    'gravity': 9.81,
	    'arm_length': 0.25,
	    'drag_coefficient': 0.1
	}
	
	# Initial conditions
	x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
	
	# Define input functions
	u0 = lambda t: 0
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
	result = ct.input_output_response(quadcopter, T=t, U=U, X0=x0)
	
	# Plot results
	p = result.plot()
	p.figure.show()
	