import numpy as np

class FIFO(object):
    def __init__(self,Numb):
        self.Numb = Numb
        self.array = np.zeros(Numb)
        self.m = 0
        self.i = 0

    def push(self,new):
        self.mean(new,self.array[self.i])
        self.array[self.i] = new
        self.i += 1
        if (self.i==self.Numb):
            self.i=0
            
    def mean(self,new,last):
        self.m += new/self.Numb
        self.m -= last/self.Numb
        
            
    def get_mean(self):
        return self.m
    
