import control as ct
import numpy as np


def outputs(t, x, u, params):
    phi, theta, psi = u
    # Gravity vector in navigation frame (NED)
    g_ned = np.array([0, 0, 9.81])
    
    # Rotation matrix from navigation frame to body frame
    R = np.array([
        [np.cos(theta)*np.cos(psi), np.cos(theta)*np.sin(psi), -np.sin(theta)],
        [np.sin(phi)*np.sin(theta)*np.cos(psi) - np.cos(phi)*np.sin(psi), np.sin(phi)*np.sin(theta)*np.sin(psi) + np.cos(phi)*np.cos(psi), np.sin(phi)*np.cos(theta)],
        [np.cos(phi)*np.sin(theta)*np.cos(psi) + np.sin(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.sin(psi) - np.sin(phi)*np.cos(psi), np.cos(phi)*np.cos(theta)]
    ])
    
    # Gravity vector in body frame
    g_body = R @ g_ned
    
    return g_body.tolist()

gravity_force_model = ct.nlsys(updfcn=None, outfcn=outputs, inputs=3, outputs=3, name='gravity_force_model')

gravity_force_model.set_inputs(['phi', 'theta', 'psi'])
gravity_force_model.set_outputs(['Fx', 'Fy', 'Fz'])


if __name__ == '__main__':
	# Time vector
	t = np.arange(0.0, 10.0, 0.01)
	
	# Parameters
	# Initial conditions
	# Define input functions
	u0 = lambda t: 0
	u1 = lambda t: 0.1745
	u2 = lambda t: 0
	
	# Combine input functions
	U = np.vstack([
	    np.array([u0(ti) for ti in t]).T,
	    np.array([u1(ti) for ti in t]).T,
	    np.array([u2(ti) for ti in t]).T
	])
	
	# Run input-output response simulation
	result = ct.input_output_response(gravity_force_model, T=t, U=U)
	
	# Plot results
	p = result.plot()
	p.figure.show()
	