import control as ct
import numpy as np

def dynamics(t, x, u, params):
    # Unpack states
    pos = x[:3]  # Position [x, y, z]
    vel = x[3:6]  # Velocity [u, v, w]
    euler = x[6:9]  # Euler angles [phi, theta, psi]
    omega = x[9:]  # Angular rates [p, q, r]

    # Unpack inputs
    force = u[:3]  # Forces [Fx, Fy, Fz]
    moments = u[3:]  # Moments [Mx, My, Mz]

    # Unpack parameters
    mass = params['mass']
    Ixx = params['Ixx']
    Iyy = params['Iyy']
    Izz = params['Izz']

    # Rotation matrix from body to inertial frame
    phi, theta, psi = euler
    R = np.array([
        [np.cos(theta)*np.cos(psi), np.cos(theta)*np.sin(psi), -np.sin(theta)],
        [np.sin(phi)*np.sin(theta)*np.cos(psi) - np.cos(phi)*np.sin(psi), np.sin(phi)*np.sin(theta)*np.sin(psi) + np.cos(phi)*np.cos(psi), np.sin(phi)*np.cos(theta)],
        [np.cos(phi)*np.sin(theta)*np.cos(psi) + np.sin(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.sin(psi) - np.sin(phi)*np.cos(psi), np.cos(phi)*np.cos(theta)]
    ])

    # Kinematic equations
    ddt_pos = R @ vel
    ddt_vel = force / mass - np.cross(omega, vel)

    # Euler angle rates
    ddt_euler = np.array([
        [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
    ]) @ omega

    # Angular rate dynamics
    ddt_omega = np.array([
        (moments[0] - (Iyy - Izz) * omega[1] * omega[2]) / Ixx,
        (moments[1] - (Izz - Ixx) * omega[2] * omega[0]) / Iyy,
        (moments[2] - (Ixx - Iyy) * omega[0] * omega[1]) / Izz
    ])

    return np.hstack([ddt_pos, ddt_vel, ddt_euler, ddt_omega])

def outputs(t, x, u, params):
    return x  # Outputs are the states themselves

rigid_body = ct.nlsys(updfcn=dynamics, outfcn=outputs, states=12, inputs=6, outputs=12, name='rigid_body')

rigid_body.set_states(['x', 'y', 'z', 'u', 'v', 'w', 'phi', 'theta', 'psi', 'p', 'q', 'r'])
rigid_body.set_inputs(['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])
rigid_body.set_outputs(['x', 'y', 'z', 'u', 'v', 'w', 'phi', 'theta', 'psi', 'p', 'q', 'r'])


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
	u0 = lambda t: 0
	u1 = lambda t: 0
	u2 = lambda t: 0
	u3 = lambda t: 0
	u4 = lambda t: 0
	u5 = lambda t: 0
	
	# Combine input functions
	U = np.vstack([
	    np.array([u0(ti) for ti in t]).T,
	    np.array([u1(ti) for ti in t]).T,
	    np.array([u2(ti) for ti in t]).T,
	    np.array([u3(ti) for ti in t]).T,
	    np.array([u4(ti) for ti in t]).T,
	    np.array([u5(ti) for ti in t]).T
	])
	
	# Run input-output response simulation
	result = ct.input_output_response(rigid_body, T=t, U=U, X0=x0)
	
	# Plot results
	p = result.plot()
	p.figure.show()
	