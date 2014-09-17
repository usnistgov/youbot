
import yaml
import copy

class CommandSequence(object):
    
    def __init__(self, path_to_yaml):
        # config file
        f = open(path_to_yaml)
        self._d = copy.deepcopy(yaml.load(f))
        f.close()
        # iterable items     
        self._i = 0   
        
    def __getitem__(self, k):
        return self._d[k]
    
    def __str__(self):
        return str(self._d)
        
    def reset(self): 
        self._i = 0
    
    def next(self):
        v = self._d[self._i]
        next_i = self._i + 1
        self._i = next_i % len(self._d) 
        return v
    
    
        