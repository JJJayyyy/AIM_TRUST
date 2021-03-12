import collections


Transition = collections.namedtuple('Transition', ('state', 'action', 'next_state', 'reward'))


class Replay:
    def __init__(self, batch_size):
        self.capacity = 100
        self.experience = []
        self.batch_size = batch_size
        self.position = 0
        self.pos = 0
        # 20 hz * 15 * 27 = 8100 : ==> worst need for rem all time per episode in all start state

    def push(self, *args):
        if len(self.experience) < self.capacity:
            # print(len(self.experience))
            self.experience.append(Transition(*args))
            # print(Transition(*args))
        else:
            self.experience[self.position] = Transition(*args)
            self.position = (self.position + 1) % self.capacity

    def get_batch(self):
        if len(self.experience) < self.batch_size:
            batch = Transition(*zip(*self.experience))
            # batch = None
        else:
            # random catch from memory
            # sample = random.sample(self.experience, self.batch_size)
            # # sample = self.experience[-1]
            # batch = Transition(*zip(*sample))
            # print(batch)

            # catch the last one
            batch = Transition(*zip(self.experience[self.pos]))
            # print(batch)
            self.pos += 1
            self.pos = self.pos % self.capacity
            # batch = self.experience[-1]
        return batch

    def __len__(self):
        return len(self.experience)

    def read_prev_buffer(self, prev_buffer):
        # concat two list need use addition
        self.experience = self.experience + prev_buffer.experience[-100:]

