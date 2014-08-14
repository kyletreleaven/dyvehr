
import numpy as np

# dev
from setiptah.eventsim.signaling import Signal, Message
from setiptah.queuesim.misc import Dispatch

DEBUG = False

def locationOfTaxiRequest( request ) :
    p, d = request
    return p

DEFAULTKEY = locationOfTaxiRequest
DEFAULTNNSTRUCT = search.PointSet


class NNeighDispatch(Dispatch) :
    
    def __init__(self, keyFunc=DEFAULTKEY, NNstruct=DEFAULTNNSTRUCT ) :
        # geometry
        self.keyFunc = keyFunc
        
        self.locations = NNstruct()     # essentially, current batch info
        self.MAP = {}
        
        # queues
        self._batchQueue = []
        self._pending = []
        
        # interface
        self.emptied = Signal()
        
    def join_sim(self, sim ) :
        self.sim = sim
        
    # slot
    def queueBatch(self, batch ) :
        self._batchQueue.append( batch )
        self._try_dispatch()
        
    # slotoid implementation
    def input(self, child, childLoc  ) :
        """ inputs are requests for nearest outstanding task """
        self._pending.append( ( child, childLoc ) )
        self._try_dispatch()
        
    # slotoid
    def _try_dispatch(self) :
        # load up a batch if possible
        
        while len( self._pending ) > 0 and len( self.locations ) > 0 :
            child, childLoc = self._pending.pop(0)
            
            
            
    """ auto-slot, utility """
    def _try_dispatch(self) :
        if len( self.dispatches_pending ) > 0 : self.dispatches_pending.pop(0)
        
        while len( self.pending ) > 0 :
            if len( self.demands ) == 0 : break     # we ran out of demands
            
            # otherwise
            interface, location = self.pending.pop(0)       # get the next pending request
            
            loc = self.locations.find_nearest( location, self.roadnet, 'length' )
            self.locations.remove( loc )
            dem = self.demands.pop( loc )
            
            # signaling
            interface.demand_out( dem )            
            #msg = Message( interface.demand_out, dem )
            #self.sim.schedule( msg )
        
        if len( self.demands ) == 0 and len( self.batch_requests ) <= 0 :
            self.batch_requests.append( token() )   # seriously, i can be an idiot
            self.request_batch()
            #self.sim.schedule( self.request_batch )
                        
            
          
        
    def batch_arrived(self, batch ) :
        # pop a batch request from the queue
        self.batch_requests.pop(0)
        
        # insert the whole batch
        for dem in batch :
            p,_ = dem
            self.locations.insert( p )
            self.demands[ p ] = dem
            
        # schedule the dispatch loop
        if len( self.dispatches_pending ) <= 0 :
            self.dispatches_pending.append( token() )
            self.sim.schedule( self._try_dispatch )
        
    """ slot, collected from interfaces """
    def request_in(self, interface, location ) :
        self.pending.append( ( interface, location ) )
        
        # schedule the dispatch loop
        if len( self.dispatches_pending ) <= 0 :
            self.dispatches_pending.append( token() )
            self.sim.schedule( self._try_dispatch )
        
          
            


