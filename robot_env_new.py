import matlab.engine
import numpy as np

mlab = matlab.engine.start_matlab();
class Robot(object):
	done = 0.0;
	ret = 0.0;
	#S = mlab.zeros(1,61);
	#mlab.rng(12345);
	
	def __init__(self, mode):
		self.mode = mode;
		self.robot = mlab.model_new('hard', 'r', 1, 30);
		self.action_bound = mlab.getfield(self.robot,'action_bound')[0]
		self.action_dim = mlab.getfield(self.robot,'action_dim')
		self.state_dim = mlab.getfield(self.robot,'state_dim')
		self.S = mlab.zeros(1,self.state_dim);
		self.precision = mlab.getfield(self.robot,'precision')
		self.stable = mlab.getfield(self.robot,'stable')

	def reset(self):
		# print mlab.getfield(self.robot,'mode')
		# A = mlab.zeros(1,self.action_dim)
		# a_bound = matlab.double(self.action_bound)

		self.S = mlab.reset(self.robot,nargout = 1)
		self.stable = mlab.getfield(self.robot,'stable')

		return np.hstack(self.S)


	def step(self, action):
		action = np.clip(action, *self.action_bound)
		action = matlab.double(action.tolist())
		a_bound = matlab.double(self.action_bound)

		self.S, self.ret, self.done, dist = mlab.step(self.robot, action, nargout = 4)
		self.stable = mlab.getfield(self.robot,'stable')

		return np.hstack(self.S), np.hstack(self.ret), np.hstack(self.done), np.hstack(dist)
