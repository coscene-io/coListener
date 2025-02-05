import abc


class Action(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def run(self, item, scope):
        pass
