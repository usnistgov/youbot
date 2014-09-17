
import yaml
import copy

class JointPoseDictionary(object):
    
    def __init__(self, path_to_yaml):
        f = open(path_to_yaml)
        self._d = copy.deepcopy(yaml.load(f))
        f.close()        
        
    def __getitem__(self, k):
        return self._d[k]

    def __str__(self):
        return str(self._d)    
    
    