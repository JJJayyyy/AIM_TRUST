import argparse
import os
import torch

from option.options import Options

class TrainOptions(Options):
    def initialize(self):
        Options.initialize(self)
        self.isTrain = True


