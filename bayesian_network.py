""" Bayesian networks """

from probability import BayesNet, enumeration_ask, elimination_ask, rejection_sampling, likelihood_weighting, gibbs_ask
from aima_utils import print_table
import pickle

T, F = True, False

class DataPoint:
    def __init__(self, rs, t, o, c, w):
        self.rs = rs
        self.t = t
        self.o = o
        self.c = c
        self.w = w


class BayesianNetwork:
    def __init__(self):
        data = pickle.load(open("data/bn_data.p","rb"))
        # BEGIN_YOUR_CODE ######################################################
        raise NotImplementedError
        
        # END_YOUR_CODE ########################################################
