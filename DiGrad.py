import tensorflow as tf
from robot_env_new import Robot
import numpy as np
import os
import shutil
import time

s_path = './Model_ijcai_pari_non_ddpg_hu'

if os.path.isdir(s_path): shutil.rmtree(s_path)
os.mkdir(s_path)
f= open(s_path + "/log.txt","w+")
f.close()

#np.random.seed(1234)
#tf.set_random_seed(1234)

MAX_EPISODES = 10000
MAX_EP_STEPS = 180
LR_A = 1e-4   # learning rate for actor
LR_C = 1e-4   # learning rate for critic
GAMMA = 0.999   # reward discount
BETA = 0.007
REPLACE_ITER_A = 1000
REPLACE_ITER_C = 1100
MEMORY_CAPACITY = 45000
TAU = 0.001
BATCH_SIZE = 64
VAR_MIN = 0.06
RENDER = True
LOAD = False
MODE = ['easy', 'hard']
n_model = 1
env = Robot('hard');
STATE_DIM = int(env.state_dim)
ACTION_DIM = int(env.action_dim)
ACTION_BOUND = env.action_bound

r_avg = np.random.uniform(0,0,2)

# all placeholder for tf
with tf.name_scope('S'):
    S = tf.placeholder(tf.float32, shape=[None, STATE_DIM], name='s')

with tf.name_scope('A'):
    A = tf.placeholder(tf.float32, shape=[None, ACTION_DIM], name='act')
with tf.name_scope('A_'):
    A_ = tf.placeholder(tf.float32, shape=[None, ACTION_DIM], name='act_')
with tf.name_scope('R'):
    R = tf.placeholder(tf.float32, [None, 2], name='r')
# with tf.name_scope('R2'):
#     R2 = tf.placeholder(tf.float32, [None, 1], name='r2')
with tf.name_scope('S_'):
    S_ = tf.placeholder(tf.float32, shape=[None, STATE_DIM], name='s_')
# with tf.name_scope('S1_'):
#     S1_ = tf.placeholder(tf.float32, shape=[None, ACTION_DIM], name='s1_')    


class Actor2(object):
    def __init__(self, sess, action_dim, action_bound, learning_rate, t_replace_iter, tau):
        self.sess = sess
        self.a_dim = action_dim
        self.action_bound = action_bound
        self.lr = learning_rate
        self.t_replace_iter = t_replace_iter
        self.t_replace_counter = 0
        self.tau = tau

        with tf.variable_scope('Actor2'):
            # input s, output a
            self.a = self._build_net(S, scope='eval_net2', trainable=True)

            # input s_, output a, get a_ for critic
            self.a_ = self._build_net(S_, scope='target_net2', trainable=False)

        self.e_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor2/eval_net2')
        self.t_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor2/target_net2')

        self.update_target_params = \
                        [self.t_params[i].assign(tf.multiply(self.e_params[i],self.tau) + tf.multiply(self.t_params[i],1-self.tau))
                                                for i in range(len(self.t_params))]


    def _build_net(self, s, scope, trainable):
        with tf.variable_scope(scope):
            #init_w = tf.random_normal_initializer(mean=0.0,stddev=1.0)
            init_w = tf.contrib.layers.xavier_initializer()
            init_b = tf.constant_initializer(0.001)
            net = tf.layers.dense(s, 600, kernel_initializer=init_w, bias_initializer=init_b, name='l1', trainable=trainable)
            
            net = tf.layers.batch_normalization(net)
            net = tf.nn.crelu(net)
            net = tf.nn.dropout(net,0.9)

            net = tf.layers.dense(net, 500, kernel_initializer=init_w, bias_initializer=init_b, name='l2', trainable=trainable)
            net = tf.layers.batch_normalization(net)
            net = tf.nn.crelu(net)
            net = tf.nn.dropout(net,0.9)

       
            with tf.variable_scope('a'):
                #w_init = tf.random_uniform_initializer(minval  =-0.003, maxval = 0.003)
                actions = tf.layers.dense(net, self.a_dim, activation=tf.nn.tanh, kernel_initializer=init_w,
                                          name='a', trainable=trainable)
                scaled_a = tf.multiply(actions, self.action_bound, name='scaled_a')  # Scale output to -action_bound to action_bound
        return scaled_a

    def learn(self, s, a):   # batch update
        self.sess.run(self.train_op, feed_dict={S: s, A_:a, A1_:a[:,0:5], A2_:a[:,5:9], A3_:a[:,9:13]})
        # self.sess.run(self.update_target_params)

        if self.t_replace_counter % self.t_replace_iter == 0:
           self.sess.run([tf.assign(t, e) for t, e in zip(self.t_params, self.e_params)])
        self.t_replace_counter += 1

    def choose_action(self, s):
        s = s[np.newaxis, :]    # single state
        return self.sess.run(self.a, feed_dict={S: s})[0]  # single action
    
    def choose_actions(self, s):
        # s = s[np.newaxis, :]    # single state
        return self.sess.run(self.a, feed_dict={S: s})   
    
    def add_grad_to_graph(self, a_grads):
        with tf.variable_scope('policy_grads2'):
            self.policy_grads = tf.gradients(ys=self.a, xs=self.e_params, grad_ys=a_grads)

        with tf.variable_scope('A2_train'):
            opt = tf.train.RMSPropOptimizer(-self.lr)  # (- learning rate) for ascent policy
            self.train_op = opt.apply_gradients(zip(self.policy_grads, self.e_params))



class Actor(object):
    def __init__(self, sess, action_dim, action_bound, learning_rate, t_replace_iter, tau):
        self.sess = sess
        self.a_dim = action_dim
        self.action_bound = action_bound
        self.lr = learning_rate
        self.t_replace_iter = t_replace_iter
        self.t_replace_counter = 0
        self.tau = tau

        with tf.variable_scope('Actor'):
            # input s, output a
            self.a = self._build_net(S, scope='eval_net', trainable=True)

            # input s_, output a, get a_ for critic
            self.a_ = self._build_net(S_, scope='target_net', trainable=False)

        self.e_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor/eval_net')
        self.t_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor/target_net')

        self.update_target_params = \
                        [self.t_params[i].assign(tf.multiply(self.e_params[i],self.tau) + tf.multiply(self.t_params[i],1-self.tau))
                                                for i in range(len(self.t_params))]

        # with tf.variable_scope('aa_grad'):
            # self.aa_grads = tf.gradients(ys=self.a_dep, xs=self.a, grad_ys=a_grad)[0]   # tensor of gradients of each sample (None, a_dim)

    def _build_net(self, s, scope, trainable):
        with tf.variable_scope(scope):
            #init_w = tf.random_normal_initializer(mean=0.0,stddev=1.0)
            init_w = tf.contrib.layers.xavier_initializer()
            init_b = tf.constant_initializer(0.001)
            net = tf.layers.dense(s, 700, kernel_initializer=init_w, bias_initializer=init_b, name='l1', trainable=trainable)
            
            net = tf.layers.batch_normalization(net)
            net = tf.nn.crelu(net)
            net = tf.nn.dropout(net,0.8)

            net = tf.layers.dense(net, 400, kernel_initializer=init_w, bias_initializer=init_b, name='l2', trainable=trainable)
            net = tf.layers.batch_normalization(net)
            net = tf.nn.crelu(net)
            net = tf.nn.dropout(net,0.8)

            with tf.variable_scope('a'):
                #w_init = tf.random_uniform_initializer(minval  =-0.003, maxval = 0.003)
                actions = tf.layers.dense(net, self.a_dim, activation=tf.nn.tanh, kernel_initializer=init_w,
                                          name='a', trainable=trainable)
                scaled_a = tf.multiply(actions, self.action_bound, name='scaled_a')  # Scale output to -action_bound to action_bound
        return scaled_a

    def learn(self, s, a):   # batch update
        #self.sess.run(self.train_op, feed_dict={S: s, A_:a})
        self.sess.run(self.train_op, feed_dict={S: s, A_:a})
        # self.sess.run(self.update_target_params)

        if self.t_replace_counter % self.t_replace_iter == 0:
           self.sess.run([tf.assign(t, e) for t, e in zip(self.t_params, self.e_params)])
        self.t_replace_counter += 1

    def choose_action(self, s):
        s = s[np.newaxis, :]    # single state
        return self.sess.run(self.a, feed_dict={S: s})[0]  # single action

    def choose_actions(self, s):
        # s = s[np.newaxis, :]    # single state
        return self.sess.run(self.a, feed_dict={S: s})   

    def add_grad_to_graph(self, a_grads):
        with tf.variable_scope('policy_grads'):
            self.policy_grads = tf.gradients(ys=self.a, xs=self.e_params, grad_ys=a_grads)

        with tf.variable_scope('A_train'):
            opt = tf.train.RMSPropOptimizer(-self.lr)  # (- learning rate) for ascent policy
            self.train_op = opt.apply_gradients(zip(self.policy_grads, self.e_params))


class Critic(object):
    def __init__(self, sess, state_dim, action_dim, learning_rate, gamma, t_replace_iter, tau):
        self.sess = sess
        self.s_dim = state_dim
        self.a_dim = action_dim
        self.lr = learning_rate
        self.gamma = gamma
        self.t_replace_iter = t_replace_iter
        self.t_replace_counter = 0
        self.tau = tau

        with tf.variable_scope('Critic'):
            print 'Critic'
            # Input (s, a), output q
            # self.a = a
            self.q = self._build_net(S, A_, 'eval_net', trainable=True)

            # Input (s_, a_), output q_ for q_target
            self.q_ = self._build_net(S_, A, 'target_net', trainable=False)    # target_q is based on a_ from Actor's target_net
            # tmp1 = self._build_net_tar(S_, a2_, 'target_net2', trainable=False)    # target_q is based on a_ from Actor's target_net


            self.e_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Critic/eval_net')
            self.t_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Critic/target_net')

            self.update_target_params = \
                        [self.t_params[i].assign(tf.multiply(self.e_params[i],self.tau) + tf.multiply(self.t_params[i],1-self.tau))
                                                for i in range(len(self.t_params))]

        with tf.variable_scope('target_q'):
            self.target_q = (R - r_avg) + self.gamma * self.q_
            self.delta = self.target_q - self.q

        with tf.variable_scope('TD_error'):
            self.loss = (tf.reduce_mean(tf.squared_difference(self.target_q, self.q)))

        with tf.variable_scope('C_train'):
            self.train_op = tf.train.RMSPropOptimizer(self.lr).minimize(self.loss)

        with tf.variable_scope('a_grad'):


	    X = tf.gradients(self.q, A_)[0]
	    Y = tf.gradients(self.q[:,0], A_)[0]
	    Z = tf.gradients(self.q[:,1], A_)[0]
	    #print X
	    #print tf.concat([tf.gradients(self.q, A1_), tf.gradients(self.q, A2_)],1)
            #self.a_grads = tf.gradients(self.q, A1_)[0] +  tf.gradients(self.q, A2_)[0] +  tf.gradients(self.q, A3_)[0]   # tensor of gradients of each sample (None, a_dim)
            self.a_grads = tf.concat([tf.concat([0.5*X[:,0:5],Z[:,5:9]],1),Y[:,9:13]],1)   # tensor of gradients of each sample (None, a_dim)
            #self.a_grads = 0.75*X   # tensor of gradients of each sample (None, a_dim)
            #self.a_grads = tf.concat(1, [grads,Y[:,5:9]])   # tensor of gradients of each sample (None, a_dim)
            #self.a_grads = tf.gradients(self.q, A_)[0]   # tensor of gradients of each sample (None, a_dim)

    def _build_net(self, s, a, scope, trainable):
        with tf.variable_scope(scope):
            #init_w = tf.random_normal_initializer(mean=0.0,stddev=1.0)
            init_w = tf.contrib.layers.xavier_initializer()
            init_b = tf.constant_initializer(0.01)
	    regulate = tf.contrib.layers.l2_regularizer(0.01)

            with tf.variable_scope('l1'):
                n_l1 = 700
                w1_s = tf.get_variable('w1_s', [self.s_dim, n_l1], initializer=init_w, trainable=trainable, regularizer = regulate)
                w1_a = tf.get_variable('w1_a', [self.a_dim, n_l1], initializer=init_w, trainable=trainable, regularizer = regulate)
                b1 = tf.get_variable('b1', [1, n_l1], initializer=init_b, trainable=trainable, regularizer = regulate)

                net = tf.layers.batch_normalization(tf.matmul(s, w1_s) + tf.matmul(a, w1_a) + b1)
            net = tf.nn.crelu(net)
            net = tf.nn.dropout(net,0.7)
                
            net = tf.layers.dense(net,400 , activation=tf.nn.crelu,
                                  kernel_initializer=init_w, bias_initializer=init_b, name='l2',
                                  trainable=trainable, kernel_regularizer = regulate)

            net = tf.nn.dropout(net,0.8)

            with tf.variable_scope('q'):
		'''
                w4a = tf.get_variable('w4a', [1800, 300], initializer=init_w, trainable=trainable)
                b4a = tf.get_variable('b4a', [1, 300], initializer=init_b, trainable=trainable)
                
                w4b = tf.get_variable('w4b', [1800, 400], initializer=init_w, trainable=trainable)
                b4b = tf.get_variable('b4b', [1, 400], initializer=init_b, trainable=trainable)

                w5a = tf.get_variable('w5a', [300, 1], initializer=init_w, trainable=trainable)
                b5a = tf.get_variable('b5a', [1, 1], initializer=init_b, trainable=trainable)
            
                w5b = tf.get_variable('w5b', [400, 2], initializer=init_w, trainable=trainable)
                b5b = tf.get_variable('b5b', [1, 2], initializer=init_b, trainable=trainable)
		
		net_4a = tf.layers.dense(net,300 , activation=tf.nn.crelu,
                                  kernel_initializer=init_w, bias_initializer=init_b, name='net_4a',
                                  trainable=trainable, kernel_regularizer = regulate)

		net_4b = tf.layers.dense(net, 400 , activation=tf.nn.crelu,
                                  kernel_initializer=init_w, bias_initializer=init_b, name='net_4b',
                                  trainable=trainable, kernel_regularizer = regulate)


                #net_4a = tf.nn.crelu(tf.matmul(net, w4a) + b4a)
                #net_4b = tf.nn.crelu(tf.matmul(net, w4b) + b4b)


		value_5a = tf.layers.dense(net_4a,1 ,
                                  kernel_initializer=init_w, bias_initializer=init_b, name='value_5a',
                                  trainable=trainable, kernel_regularizer = regulate)

                adv_5b = tf.layers.dense(net_4b, 2,
                                  kernel_initializer=init_w, bias_initializer=init_b, name='adv_5b',
                                  trainable=trainable, kernel_regularizer = regulate)


		

                #value_5a = tf.identity(tf.matmul(net_4a, w5a) + b5a)
                #advantage_5b = tf.identity(tf.matmul(net_4b, w5b) + b5b)  
                # tmp = tf.stack([(advantage_5b - tf.reduce_mean(advantage_5b, axis=1, keep_dims=True)), (advantage_5b - tf.reduce_mean(advantage_5b, axis=1, keep_dims=True))], axis = 0)
                
                # print tmp

                q = value_5a +  adv_5b #tf.concat([(advantage_5b - tf.reduce_mean(advantage_5b, axis=1, keep_dims=True)), (advantage_5b - tf.reduce_mean(advantage_5b, axis=1, keep_dims=True))], axis = 0)
                '''
                # q = tf.argmax(tmp,1)
                #w_init = tf.random_uniform_initializer(minval  =-0.003, maxval = 0.003)
                q = tf.layers.dense(net, 2, kernel_initializer=init_w, bias_initializer=init_b, trainable=trainable)   # Q(s,a)
        return q


    def learn(self, s, a, r, s_, act):
        target_q, delta, op = self.sess.run([self.q,self.delta, self.train_op], feed_dict={S: s, A_: a, R: r, S_: s_, A: act})
        # self.sess.run(self.train_op, feed_dict={S: s, self.a: a, R: r, S_: s_})
        # self.sess.run(self.update_target_params)
        
        if self.t_replace_counter % self.t_replace_iter == 0:
           self.sess.run([tf.assign(t, e) for t, e in zip(self.t_params, self.e_params)])
        self.t_replace_counter += 1
        return target_q, delta

    def ret_q(self, s, a):
    	return self.sess.run([self.q], feed_dict={S: s, A_: a})

class Memory(object):
    def __init__(self, capacity, dims):
        self.capacity = capacity
        self.data = np.zeros((capacity, dims))
        self.pointer = 0

    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, a, r, s_))
        index = self.pointer % self.capacity  # replace the old memory with new memory
        self.data[index, :] = transition
        self.pointer += 1

    def sample(self, n):
        assert self.pointer >= self.capacity, 'Memory has not been fulfilled'
        indices = np.random.choice(self.capacity, size=n)
        return self.data[indices, :]


#sess = tf.Session()

config = tf.ConfigProto()
config.gpu_options.allow_growth=True
sess = tf.Session(config=config)

# Create actor and critic.
actor = Actor(sess, ACTION_DIM, ACTION_BOUND[1], LR_A, REPLACE_ITER_A, TAU)

critic = Critic(sess, STATE_DIM, ACTION_DIM, LR_C, GAMMA, REPLACE_ITER_C, TAU)
actor.add_grad_to_graph(critic.a_grads)



M = Memory(MEMORY_CAPACITY, dims=2 * STATE_DIM +  ACTION_DIM + 2)

saver = tf.train.Saver(max_to_keep=0)
# path = './'+'dpl'
path  = s_path
#path = './relu'
if LOAD:
    saver.restore(sess, tf.train.latest_checkpoint(path))
else:
    sess.run(tf.global_variables_initializer())


def train():
    var = 15.  # control exploration
    
  
    f = open(s_path + "/log.txt","a+")
    localtime = time.asctime( time.localtime(time.time()) )
    f.write(localtime + '\n')

    for ep in range(MAX_EPISODES):
        s = env.reset()
        ep_reward = [0,0]
        qmax = 0
        qmax1 = 0
        stability = 0;
        flag = [0,0]
        for t in range(MAX_EP_STEPS):
        # while True:
            #if RENDER:
             #   env.render()

                # Added exploration noise
            a1 = actor.choose_action(s)
            #a1 = np.clip(a1, *ACTION_BOUND);
            #a2[5:9] = [0,0,0,0];
            a = a1
            a = np.clip(np.random.normal((a), var), *ACTION_BOUND)    # add randomness to action selection for exploration
	    s_, r, done, dist = env.step(a)
            done1 = done[0]
            done2 = done[1]


            q_val = critic.ret_q(np.reshape(s,(1,STATE_DIM)),np.reshape(a,(1,ACTION_DIM)))
            M.store_transition(s, a, r, s_)
            # print a2

            if M.pointer > MEMORY_CAPACITY:
                var = max([var*.999983, VAR_MIN])    # decay the action randomness
                b_M = M.sample(BATCH_SIZE)
                b_s = b_M[:, :STATE_DIM]
                b_a = b_M[:, STATE_DIM: STATE_DIM + ACTION_DIM]
                b_r = b_M[:, -STATE_DIM - 2: -STATE_DIM]
                b_s_ = b_M[:, -STATE_DIM:]

                q, delta = critic.learn(b_s, b_a, b_r, b_s_,actor.choose_actions(b_s_))

                actor.learn(b_s,actor.choose_actions(b_s))


                qmax += np.max(q[:,0])
                qmax1 += np.max(q[:,1])

                                


		'''	
                if(q_val[0][0][0] >= np.max(q[:,0])):
                	flag[0] = 1
                	r_avg[0] += BETA * delta[0]

                if(q_val[0][0][1] >= np.max(q[:,1])):
                	flag[1] = 1
                	r_avg[1] += BETA * delta[1]
                '''
            s = s_
            ep_reward[0] += r[0]
            ep_reward[1] += r[1]

            if t == MAX_EP_STEPS-1 or (done1 and done2) :
        # if done:
                if done1 and done2:
                    result = '| done' 
                elif done1:
                    result = '| done1'
                elif done2: 
                    result = '| done2' 
                else:
                    result =  '| ----'


            # if stability == 1:
            #     result = result +  '|unstable'
            # else : 
            #     result = result + '|  stable'


                output = 'Ep:'+ str(ep) + result +\
                            '| dist: ' + str(int(dist[0])) + ',' + str(int(dist[1])) + \
                            '| R: ' + str(int(ep_reward[0])) + ',' + str(int(ep_reward[1])) + \
                            '| Steps: ' + str(t) + \
                            '| Qmax: ' + str(qmax/(t+1))  + ',' + str(qmax1/(t+1))   + \
                            '| Explore:' + str(var) + '\n' 

                f.write(output)
                print('Ep:', ep,
                      result,
                      '| dist: %i,%i' % (int(dist[0]), int(dist[1])),
                      '| R: %i,%i' % (int(ep_reward[0]) ,int(ep_reward[1])),
                      '| Steps: %i' % (t),
                      '| Qmax: %.2f, %.2f' % ((qmax/(t+1)), (qmax1/(t+1))),
                      '| Explore: %.2f' % var,
                      )
                break

        if(ep>=900 and (ep)%300==0 ):
            ckpt_path = os.path.join(path+'/'+'%i'%ep, 'DDPG.ckpt')
            save_path = saver.save(sess, ckpt_path, write_meta_graph=False)
            print("\nSave Model %s\n" % save_path)



def eval():
    count = 101
    while(count):
        i = 0
	count = count - 1
        #env.set_fps(30)
        s = env.reset()
        for i in range(1,150):
            i = i + 1
            #if RENDER:
             #   env.render()
            a = actor.choose_action(s)
            s_, r, done, dist = env.step(a)
	    if done[0] and done[1]:

	        break;
            s = s_

	print done[0] and done[1], i, dist[0], dist[1]
if __name__ == '__main__':
    if LOAD:
        eval()
    else:
        train()
