import control as ct
import numpy as np
from thrust1 import thrust1
from thrust2 import thrust2
from thrust3 import thrust3
from thrust4 import thrust4
from gravity_force import gravity_force
from rigid_body import rigid_body

quadcopter_system_inputs = ['thrust1.control_signal', 'thrust2.control_signal', 'thrust3.control_signal', 'thrust4.control_signal']
quadcopter_system_input_labels = ['control_signal1', 'control_signal2', 'control_signal3', 'control_signal4']
quadcopter_system_outputs = ['rigid_body.x', 'rigid_body.y', 'rigid_body.z', 'rigid_body.phi', 'rigid_body.theta', 'rigid_body.psi']
quadcopter_system_output_labels = ['x', 'y', 'z', 'phi', 'theta', 'psi']

quadcopter_system = ct.interconnect(
	(thrust1, thrust2, thrust3, thrust4, gravity_force, rigid_body),
	name = 'quadcopter_system',
	connections = (
		['gravity_force.phi', 'rigid_body.phi'],
		['gravity_force.theta', 'rigid_body.theta'],
		['gravity_force.psi', 'rigid_body.psi'],
		['rigid_body.Fx', 'thrust1.thrust', 'thrust2.thrust', 'thrust3.thrust', 'thrust4.thrust', 'gravity_force.Fx'],
		['rigid_body.Fy', 'gravity_force.Fy'],
		['rigid_body.Fz', 'gravity_force.Fz'],
		['rigid_body.Mx', 'thrust1.moment', 'thrust2.moment', 'thrust3.moment', 'thrust4.moment'],
		['rigid_body.My', 'thrust1.moment', 'thrust2.moment', 'thrust3.moment', 'thrust4.moment'],
		['rigid_body.Mz', 'thrust1.moment', 'thrust2.moment', 'thrust3.moment', 'thrust4.moment']
	),
	inplist = quadcopter_system_inputs,
	inputs = quadcopter_system_input_labels,
	outlist = quadcopter_system_outputs,
	outputs = quadcopter_system_output_labels
)

if __name__ == '__main__':
	# Time vector
	t = np.arange(0.0, 10.0, 0.01)
	
	# Parameters
	rigid_body.params = {
	    'mass': 60.0,
	    'Ixx': 10.0,
	    'Iyy': 10.0,
	    'Izz': 30.0
	}
	
	# Initial conditions
	x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
	
	# Define input functions
	u0 = lambda t: 15
	u1 = lambda t: 15
	u2 = lambda t: 15
	u3 = lambda t: 15
	
	# Combine input functions
	U = np.vstack([
	    np.array([u0(ti) for ti in t]).T,
	    np.array([u1(ti) for ti in t]).T,
	    np.array([u2(ti) for ti in t]).T,
	    np.array([u3(ti) for ti in t]).T
	])
	
	# Run input-output response simulation
	result = ct.input_output_response(quadcopter_system, T=t, U=U, X0=x0)
	
	# Plot results
	p = result.plot()
	p.figure.show()
	