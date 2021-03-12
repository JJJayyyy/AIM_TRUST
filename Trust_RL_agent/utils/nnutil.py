import os

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.nn import init
from torch.optim import lr_scheduler


def weights_init_normal(m):
    classname = m.__class__.__name__
    if classname.find('Conv') != -1:
        init.normal_(m.weight.data, 0.0, 0.02)
    elif classname.find('Linear') != -1:
        # todo weight changed here
        init.normal_(m.weight.data, 0.0, 0.02)
        # init.normal(m.weight.data, 0.0, 0.02)
    elif classname.find('BatchNorm2d') != -1:
        init.normal_(m.weight.data, 1.0, 0.02)
        init.constant(m.bias.data, 0.0)


def weights_init_xavier(m):
    classname = m.__class__.__name__
    # print(classname)
    if classname.find('Conv') != -1:
        init.xavier_normal(m.weight.data, gain=1)
    elif classname.find('Linear') != -1:
        init.xavier_normal(m.weight.data, gain=1)
    elif classname.find('BatchNorm2d') != -1:
        init.normal(m.weight.data, 1.0, 0.02)
        init.constant(m.bias.data, 0.0)


def weights_init_kaiming(m):
    classname = m.__class__.__name__
    # print(classname)
    if classname.find('Conv') != -1:
        init.kaiming_normal(m.weight.data, a=0, mode='fan_in')
    elif classname.find('Linear') != -1:
        init.kaiming_normal(m.weight.data, a=0, mode='fan_in')
    elif classname.find('BatchNorm2d') != -1:
        init.normal(m.weight.data, 1.0, 0.02)
        init.constant(m.bias.data, 0.0)


def weights_init_orthogonal(m):
    classname = m.__class__.__name__
    print(classname)
    if classname.find('Conv') != -1:
        init.orthogonal(m.weight.data, gain=1)
    elif classname.find('Linear') != -1:
        init.orthogonal(m.weight.data, gain=1)
    elif classname.find('BatchNorm2d') != -1:
        init.normal(m.weight.data, 1.0, 0.02)
        init.constant(m.bias.data, 0.0)


def init_weights(net, init_type='normal'):
    print('initialization method [%s]' % init_type)
    if init_type == 'normal':
        net.apply(weights_init_normal)
    elif init_type == 'xavier':
        net.apply(weights_init_xavier)
    elif init_type == 'kaiming':
        net.apply(weights_init_kaiming)
    elif init_type == 'orthogonal':
        net.apply(weights_init_orthogonal)
    else:
        raise NotImplementedError('initialization method [%s] is not implemented' % init_type)


def define_net(input_nc, output_nc, init_type='normal', gpu_ids=None):
    global net
    if gpu_ids is None:
        gpu_ids = []
    use_gpu = len(gpu_ids) > 0
    if use_gpu:
        assert (torch.cuda.is_available())
    net = neural_Network(input_nc, output_nc, gpu_ids=gpu_ids)
    if len(gpu_ids) > 0:
        net.cuda(gpu_ids[0])
    # init_weights(net, init_type=init_type)
    return net


class neural_Network(nn.Module):
    def __init__(self, input_nc, output_nc, gpu_ids):
        super(neural_Network, self).__init__()
        self.gpu_ids = gpu_ids
        self.fc1 = nn.Linear(input_nc, 32)
        # self.fc1.weight.data.normal_(0, 0.1)  # initialization
        self.fc2 = nn.Linear(32, 32)
        # self.fc1.weight.data.normal_(0, 0.1)  # initialization
        self.fc3 = nn.Linear(32, output_nc)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.softmax(self.fc3(x), dim=-1)
        return x


# helper saving function that can be used by subclasses
def save_network(network, network_label, epoch_label, tick_label, gpu_ids, save_dir):
    print("Saving network")
    save_filename = 'epo{}_tick{}_net_{}.pth'.format(epoch_label, tick_label, network_label)
    save_path = os.path.join(save_dir, save_filename)
    torch.save(network.cpu().state_dict(), save_path)
    if len(gpu_ids) and torch.cuda.is_available():
        network.cuda(gpu_ids[0])
    save_latest_network(network, network_label, gpu_ids, save_dir)


def save_network_at_success(network, network_label, epoch_label, tick_label, gpu_ids, save_dir):
    print("Saving success network")
    save_filename = 'epo{}_tick{}_net_{}_success.pth'.format(epoch_label, tick_label, network_label)
    save_path = os.path.join(save_dir, save_filename)
    torch.save(network.cpu().state_dict(), save_path)
    if len(gpu_ids) and torch.cuda.is_available():
        network.cuda(gpu_ids[0])
    save_latest_network(network, network_label, gpu_ids, save_dir)


def save_latest_network(network, network_label, gpu_ids, save_dir):
    save_filename = 'latest_net_{}.pth'.format(network_label)
    save_path = os.path.join(save_dir, save_filename)
    torch.save(network.cpu().state_dict(), save_path)
    if len(gpu_ids) and torch.cuda.is_available():
        network.cuda(gpu_ids[0])

    x = torch.randn(1, 2, requires_grad=True)

    export_to_onnx(x, network)


def export_to_onnx(x, torch_model):
    torch.onnx.export(torch_model,  # model being run
                      x,  # model input (or a tuple for multiple inputs)
                      "super_resolution.onnx",  # where to save the model (can be a file or file-like object)
                      export_params=True,  # store the trained parameter weights inside the model file
                      opset_version=10,  # the ONNX version to export the model to
                      do_constant_folding=True,  # whether to execute constant folding for optimization
                      input_names=['input'],  # the model's input names
                      output_names=['output'],  # the model's output names
                      dynamic_axes={'input': {0: 'batch_size'},  # variable lenght axes
                                    'output': {0: 'batch_size'}})


# helper loading function that can be used by subclasses
def load_network(network, network_label, save_dir):
    print("======= Loading Network ========")
    save_filename = 'latest_net_{}.pth'.format(network_label)
    save_path = os.path.join(save_dir, save_filename)
    network.load_state_dict(torch.load(save_path))


# update learning rate (called once every epoch)
def update_learning_rate(self):
    for scheduler in self.schedulers:
        scheduler.step()
    lr = self.optimizers[0].param_groups[0]['lr']
    print('learning rate = %.7f' % lr)


def get_scheduler(optimizer, opt):
    if opt.lr_policy == 'lambda':
        def lambda_rule(epoch):
            lr_l = 1.0 - max(0, epoch + 1 + opt.epoch_count - opt.niter) / float(opt.niter_decay + 1)
            return lr_l

        scheduler = lr_scheduler.LambdaLR(optimizer, lr_lambda=lambda_rule)
    elif opt.lr_policy == 'step':
        scheduler = lr_scheduler.StepLR(optimizer, step_size=opt.lr_decay_iters, gamma=0.1)
    elif opt.lr_policy == 'plateau':
        scheduler = lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.2, threshold=0.01, patience=5)
    else:
        return NotImplementedError('learning rate policy [%s] is not implemented', opt.lr_policy)
    return scheduler


def get_nn_weights(net):
    params = list(net.parameters())
    weight_list = []
    for i in range(len(params)):
        print(params[i].data.tolist())  # this works
        weight_list.append(params[i].clone())
    return weight_list
