class Enum(object):
    """Custom Enum implementation for Python 2.7 compatibility."""
    def __init__(self, name, members):
        self._name = name
        self._members = {member: index for index, member in enumerate(members)}

    def __getattr__(self, name):
        if name in self._members:
            return self._members[name]
        raise AttributeError("Enum '{}' has no attribute '{}'".format(self._name, name))

    def __getitem__(self, key):
        if key in self._members:
            return self._members[key]
        raise KeyError("'{}' is not a valid key for Enum '{}'".format(key, self._name))

    def __iter__(self):
        return iter(self._members)

    def __repr__(self):
        return "<Enum {}: {}>".format(self._name, ", ".join(self._members.keys()))