#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import numpy
import torch
import numpy as np
import pandas as pd
from agent_base import DqnAgent
from option.train_options import TrainOptions
import csv
import agent_base
import matplotlib.pyplot as plt

readfile = '../AIM_trust_simulator/GUITORL.csv'
writefile = '../AIM_trust_simulator/BUFFER_UPDATE.csv'

t_stamp, episode_flag = -1, -1
datastream = -1


def write_buffer_size(filename, row):
    with open(filename, 'w') as f:
        writer = csv.writer(f)
        writer.writerows(row)


def read_info_from_java(filename, old_time):
    try:
        with open(filename, 'r') as f:
            global datastream, t_stamp, episode_flag
            t_stamp = float(f.readline())
            episode_flag = int(f.readline())
    except:
        # print("file read exception or timstamp data read exception")
        # time.sleep(0.001)
        return False
    if t_stamp > old_time:
        try:
            datastream = pd.read_csv(filename, skiprows=2)
            if len(datastream["Collision"]) == agent_base.CAR_NUM:
                return True
            return False
            # return True
        except:
            # print("datastream read exception")
            # time.sleep(0.001)
            return False
    else:
        # print("time is lower than old timestamp")
        # time.sleep(0.001)
        return False


def content_test(filename, old_time):
    flag = False
    while not flag:
        flag = read_info_from_java(filename, old_time)
    vin = datastream["VIN"].values
    trust = datastream["Trust"].values
    velocity = datastream["Velocity"].values
    Spawnroad = datastream["Spawnroad"].values
    destination = datastream["Destination"].values  # todo: when java ready change here
    buffersize = datastream["Buffersize"].values
    collision = datastream["Collision"].values
    v = datastream["VIN"].values - 1000
    d = [t_stamp, episode_flag, vin, trust, velocity, Spawnroad, destination, buffersize, collision]
    datainput = np.stack((v, trust, Spawnroad, destination), axis=1).reshape(len(vin), 4)
    # datainput = np.stack((v, Spawnroad, destination), axis=1).reshape(len(vin), 3)
    return d, datainput, t_stamp, episode_flag, collision


def initailize_gui(car_max_id):
    info_rows = [[50]]  # for 50 cars need a larger time 50, 10 car for 10.
    vin = []
    buffersize = []
    for i in range(car_max_id):
        vin.append(1000 + i)
        buffersize.append(1)
    info_rows.append(vin)
    info_rows.append(buffersize)
    write_buffer_size(writefile, info_rows)
    info_rows = [[0], [0], ["VIN", "Trust", "Velocity", "Spawnroad", "Destination", "Buffersize", "Collision", "Step"]]
    for i in range(car_max_id):
        info = [1000 + i, -1, -1, -1, -1, -1, -1, -1]
        info_rows.append(info)
    write_buffer_size(readfile, info_rows)


stable_number = 0


def run():
    print("training")
    opt = TrainOptions().parse()
    trainer = DqnAgent()
    trainer.initialize(opt)  # initialize DQN agent
    EPI_NUM = 25  # 2car: 30, 4car: 39, 6car: 40, 8car: 34, 10 car: 38
    episode_count = 0
    time_stamp_1 = -1
    initailize_gui(car_max_id=agent_base.CAR_NUM)
    d, state_0, time_stamp_1, episode_flag, collision = content_test(filename=readfile, old_time=time_stamp_1)
    with open("cumulativereward.csv", 'r') as f:
        lines = f.readlines()
        if lines is not None:
            trainer.cumulative_reward = float(lines[-1])
            print(trainer.cumulative_reward)
    while episode_count < EPI_NUM:
        continue_train = True
        while continue_train:
            # get the state 0
            d, state_0, time_stamp_0, episode_flag, collision = content_test(filename=readfile, old_time=time_stamp_1)
            print("read1:", episode_flag)
            trainer.set_state_0(state_0)
            vin = d[2]
            # get the action from the env
            if episode_count >= stable_number:
                action = trainer.choose_action()
                action = action.squeeze().tolist()
            else:
                action = [1]*len(vin)
            round(time_stamp_0, 2)
            info_rows = [[time_stamp_0 + 80], list(vin), action]
            # apply the action the env
            write_buffer_size(writefile, info_rows)
            # get the state 1 from the env
            d, state_1, time_stamp_1, episode_flag, collision = content_test(filename=readfile, old_time=time_stamp_0)
            print("read2:", episode_flag)
            trainer.set_state_1(state_1)
            if episode_count >= stable_number:
                trainer.update(collision=collision)
            # when the car spawns, enter the next episode
            if len(trainer.cumulative_reward_history) > 0:
                numpy.savetxt('cumulativereward.csv', trainer.cumulative_reward_history, delimiter=',')
                numpy.savetxt('loss.csv', trainer.loss_history, delimiter=',')
            if episode_flag == 1:
                continue_train = False
            else:
                continue_train = True
        print(episode_count)
        torch.save(trainer.eval_net, 'eval_net.pkl')  # save the parameter in the network
        torch.save(trainer.target_net, 'target_net.pkl')  # save the parameter in the network
        episode_count += 1

    loss = trainer.get_loss()
    cumulative_reward = trainer.get_cumulative_reward()

    plt.subplot(2, 1, 1)
    plt.plot(loss)
    plt.ylabel('loss')
    plt.subplot(2, 1, 2)
    plt.plot(cumulative_reward)
    plt.xlabel('step')
    plt.ylabel('Cumulative reward')
    plt.show()


def test():
    print("test")
    opt = TrainOptions().parse()
    trainer = DqnAgent()
    trainer.initialize(opt)  # initialize DQN agent
    EPI_NUM = 8
    episode_count = 0
    time_stamp_1 = -1
    initailize_gui(car_max_id=agent_base.CAR_NUM)
    d, state_0, time_stamp_1, episode_flag, collision = content_test(filename=readfile, old_time=time_stamp_1)
    while episode_count < EPI_NUM:
        continue_train = True
        while continue_train:
            # get the state 0
            d, state_0, time_stamp_0, episode_flag, collision = content_test(filename=readfile, old_time=time_stamp_1)
            print("read1:", episode_flag)
            trainer.set_state_0(state_0)
            vin = d[2]
            # get the action from the env
            if episode_count >= stable_number:
                action = trainer.choose_action_test()
                action = action.squeeze().tolist()
            else:
                action = [1]*len(vin)
            round(time_stamp_0, 2)
            info_rows = [[time_stamp_0 + 80], list(vin), action]
            # apply the action the env
            write_buffer_size(writefile, info_rows)
            # get the state 1 from the env
            d, state_1, time_stamp_1, episode_flag, collision = content_test(filename=readfile, old_time=time_stamp_0)
            print("read2:", episode_flag)
            # when the car spawns, enter the next episode
            if episode_flag == 1:
                continue_train = False
            else:
                continue_train = True
        print(episode_count)
        episode_count += 1


if __name__ == '__main__':
    # run()
    test()
