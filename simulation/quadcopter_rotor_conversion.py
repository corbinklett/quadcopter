import control as ct
import numpy as np


def outputs(t, x, u, params):
    # Extract parameters
    kf = params['kf']
    kd = params['kd']
    l = params['l']
    w_max = params['w_max']
    
    # Extract input
    omega = u[0]
    
    # Calculate omega squared
    omega_squared = np.clip(omega, 0, w_max)**2
    
    # Calculate thrust and torque
    thrust = kf * omega_squared
    torque = kd * omega_squared * l
    
    return [thrust, torque]

quadcopter_rotor_conversion = ct.nlsys(updfcn=None, outfcn=outputs, inputs=1, outputs=2, name='quadcopter_rotor_conversion')

quadcopter_rotor_conversion.set_inputs(1, prefix='omega')
quadcopter_rotor_conversion.set_outputs(['thrust', 'torque'])


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
	
	# Initial conditions
	# Define input functions
	u0 = lambda t: 400 if t >= 0 else 0
	
	# Combine input functions
	U = np.vstack([
	    np.array([u0(ti) for ti in t]).T
	])
	
	# Run input-output response simulation
	result = ct.input_output_response(quadcopter_rotor_conversion, T=t, U=U)
	
	# Plot results
	p = result.plot()
	p.figure.show()
	