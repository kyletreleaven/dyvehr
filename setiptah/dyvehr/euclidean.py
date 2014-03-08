
import numpy as np

import networkx as nx


# "Planner" for Euclidean geometry
class EuclideanTraj :
    def __init__(self, orig, dest ) :
        self.orig = np.array( orig )
        self.needle = np.array( dest ) - self.orig
        self.needle /= np.linalg.norm( self.needle )
        
    def __call__(self, progress ) :
        return self.orig + progress * self.needle
    
def EuclideanPlanner( orig, dest ) :
    #print orig, dest
    trajLength = np.linalg.norm( np.array(dest) - np.array(orig) )
    traj = EuclideanTraj( orig, dest )
    return trajLength, traj


# better off in taxi?
#from taxi import TaxiScheduler

    



