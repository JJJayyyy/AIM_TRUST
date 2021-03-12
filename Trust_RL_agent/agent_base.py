import numpy as np
import py_trees
import torch
import torch.nn as nn
from torch.autograd import Variable

import utils.replay as replay
from utils import nnutil

PRE_ACTION_NAME = "prev_action"
EPSILON = 0.95  # percentage to choose action
GAMMA = 0.9  # discount factor
TARGET_REPLACE_ITER = 10
ACTION_SIZE = 17  # 17
CAR_NUM = 10


class DqnAgent:
    def __init__(self):
        self.eval_net = None
        self.target_net = None
        self.opt = None
        self.loss = None
        self.input_value = None
        self.loss_history = []
        self.cumulative_reward_history = []
        self.cumulative_reward = 0
        self.batch_size = 1
        self.replay_buffer = None
        self.device = 'cpu'
        self.s_t0 = None  # update s_t0 is actually input_value
        self.s_t1 = None  # read from simulator
        self.crash_flag = None
        self.s0_action = None
        self.learn_step_counter = 0

        self.blackboard = py_trees.blackboard.Blackboard()

        self.input1 = []
        self.input2 = []
        self.output = []

    def initialize(self, opt):
        self.opt = opt
        self.gpu_ids = opt.gpu_ids
        self.isTrain = opt.isTrain
        self.Tensor = torch.cuda.FloatTensor if self.gpu_ids else torch.Tensor
        self.loss = Variable(torch.zeros(1))
        self.input_value = None

        try:
            self.eval_net = torch.load('eval_net.pkl')
            self.target_net = torch.load('target_net.pkl')
            print("load network")
        except:
            self.eval_net = nnutil.define_net(self.opt.input_nc, ACTION_SIZE)  # this always need to be defined first
            self.target_net = nnutil.define_net(self.opt.input_nc, ACTION_SIZE)
            print("No exist network, create a new network")

        # add load network later
        if not self.isTrain:
            nnutil.load_network(self.eval_net, 'P', 'eval_net.pkl')
        self.optimizer = torch.optim.Adam(self.eval_net.parameters(), lr=opt.lr, betas=(opt.beta1, 0.999))
        self.replay_buffer = replay.Replay(self.batch_size)

    def print_network_weights(self):
        num_params = 0
        for param in self.eval_net.parameters():
            num_params += param.numel()
        print(self.eval_net)
        print('Total number of parameters: %d' % num_params)
        print("Network Printed")
        weight_list = nnutil.get_nn_weights(self.eval_net)
        print(weight_list)

    def save(self, network_label):
        nnutil.save_network(self.eval_net, 'P', network_label, 0, self.opt.gpu_ids, "./network_checkpoints")

    def make_tensor(self, var, require_grad):
        var_tensor = torch.FloatTensor([var]).clone()
        if self.opt.gpu_ids:
            if require_grad is False:
                var_variable = Variable(var_tensor.cuda(device=self.opt.gpu_id), requires_grad=False)
            else:
                var_variable = Variable(var_tensor.cuda(device=self.opt.gpu_id), requires_grad=True)
        else:
            if require_grad is False:
                var_variable = Variable(var_tensor, requires_grad=False)
            else:
                var_variable = Variable(var_tensor, requires_grad=True)
        return var_variable

    def set_state_0(self, state):
        self.s_t0 = self.make_tensor(state, True)

    def set_state_1(self, state):
        self.s_t1 = self.make_tensor(state, require_grad=False)

    def choose_action(self):
        if np.random.random() < EPSILON:
            x = self.eval_net.forward(self.s_t0).view(CAR_NUM, ACTION_SIZE)
            print(x)  # output Q value
            self.s0_action = torch.max(x, 1)[1].view(CAR_NUM, 1)
            # return the index of the maximum value.if [0] return value
            # print("greedy action", self.s0_action)
        else:
            self.s0_action = torch.from_numpy(np.random.randint(0, ACTION_SIZE - 1, (CAR_NUM, 1)))
            # print("random action", self.s0_action)
        return self.s0_action  # note this is index

    def learn(self):
        # update the parameter to target net
        if self.learn_step_counter % TARGET_REPLACE_ITER == 0:
            self.target_net.load_state_dict(self.eval_net.state_dict())
        self.learn_step_counter += 1

        batch = self.replay_buffer.get_batch()
        state_batch = torch.cat(batch.state)
        # print("s0 ", state_batch)
        action_batch = torch.cat(batch.action)
        reward_batch = torch.cat(batch.reward)
        next_state_batch = torch.cat(batch.next_state)
        # print("s1 ", next_state_batch)
        # print("action", action_batch)
        # print("state_action_values: ", self.eval_net(state_batch)[0][:])
        state_action_values = self.eval_net(state_batch)[0][:].gather(1, action_batch)
        next_state_values = torch.max(self.target_net(next_state_batch)[0][:].detach(), 1)[0].view(CAR_NUM, 1)
        expected_state_action_values = (next_state_values * GAMMA) + reward_batch.view(CAR_NUM, 1)
        ff = nn.MSELoss()
        loss = ff(input=state_action_values, target=expected_state_action_values)
        # loss = F.smooth_l1_loss(input=state_action_values, target=expected_state_action_values)
        # Optimize the model
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        self.loss_history.append(loss)

    def update(self, collision):  # collision is a list
        self.record_experience(collision)  # s1, s0 both get, reward caluclate insede
        self.learn()

    def record_experience(self, collision):
        reward = self.calculate_reward(collision, self.s0_action)
        reward = self.make_tensor(reward, require_grad=True)  # todo: double check grad
        reward = reward.view(CAR_NUM, 1)
        self.replay_buffer.push(self.s_t0, self.s0_action, self.s_t1, reward)

    def calculate_reward(self, collision, action):
        reward = []
        bth = [16] * CAR_NUM

        # with bth
        for i in range(CAR_NUM):
            if collision[i] == 1:
                reward.append([-40])
            elif collision[i] == -1:
                reward.append([0])
            else:
                if bth[i] > action[i]:
                    reward.append([1 + (bth[i] - action[i])*0.1])
                else:
                    reward.append([1 + (bth[i] - action[i])*0.2])
        # print("reward: ", reward)
        self.cumulative_reward += np.sum(reward)
        # print("cumulative reward: ", self.cumulative_reward)
        self.cumulative_reward_history.append(self.cumulative_reward)
        return reward

    def test(self):
        output = self.eval_net(self.input_value)
        self.output.append(output.data.tolist()[0][0])
        i1 = self.input_value.data.tolist()[0]
        self.input1.append(i1[0])
        self.input2.append(i1[1])
        print(output)

    def get_loss(self):
        return self.loss_history

    def get_cumulative_reward(self):
        return self.cumulative_reward_history

    def choose_action_test(self):
        x = self.eval_net.forward(self.s_t0).view(CAR_NUM, ACTION_SIZE)
        print(x)
        self.s0_action = torch.max(x, 1)[1].view(CAR_NUM, 1)
        # return the index of the maximum value.if [0] return value
        # print("greedy action", self.s0_action)
        return self.s0_action  # note this is index
