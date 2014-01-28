
import numpy as np

# dev
from setiptah.eventsim.signaling import Signal, Message
from setiptah.queuesim.misc import Dispatch

DEBUG = False


class TaxiDemand :
    def __init__(self, origin, destination ) :
        self.origin = origin
        self.destination = destination


class TaxiScheduler :
    """ abstract class for algorithm to schedule taxi demands """
    
    def __call__(self, demands, agentLocations ) :
        """
        demands is an iterable of, e.g., TaxiDemands;
        agentLocations is a dict whose keys are taxi agents and whose values are their locations
        returns as dict whose keys are agents and whose values are lists of their assigned demands, in order
        """
        raise NotImplementedError('please implemented scheduling algorithm')


class RoundRobin(TaxiScheduler) :
    def __call__(self, demands, agentLocations ) :
        print 'GOT CALLED'
        agents = agentLocations.keys()
        agents = list( agents )
        
        schedule = {}
        for dem in demands :
            i = agents.pop(0)
            schedule.setdefault(i, [] ).append( dem )
            agents.append( i )
            
        return schedule
            
            
class TaxiDispatchGated(Dispatch) :
    def __init__(self) :
        # config
        self._children = set()
        
        # sim data
        self._demandQ = []
        self._waiting = set()
        self._locations = {}
        
        self.emptyFlag = False
        
        # signals
        self.emptied = Signal()
        
    """ config """
    def setScheduler(self, scheduler ) :
        assert isinstance( scheduler, TaxiScheduler )
        self.scheduler = scheduler
        
    def add(self, child ) :
        self._children.add( child )
        
    def remove(self, child ) :
        raise NotImplementedError('probably dangerous to implement this function')
        
    def join_sim(self, sim ) :
        self.sim = sim
        
    """ simulation interface """
    # slot
    def queueDemand(self, demand ) :
        self._demandQ.append( demand )
        self.emptyFlag = False
        self._try_schedule()
        
    # slot
    def input(self, child, location ) :
        # child finished!
        self._waiting.add( child )
        self._locations[child] = location
        self._try_schedule()
        
    """ utility """
    def _try_schedule(self) :
        if len( self._demandQ ) > 0 :
            # we have demands; is the gate ready?
            active = self._children.difference( self._waiting )
            if len( active ) > 0 :
                return      # nope!
            
            # compute a schedule
            schedule = self.scheduler( self._demandQ, self._locations )
            print schedule
            
            # schedule the assignment of each agent
            for child in schedule :
                msg = Message( child.output, schedule[child] )
                self.sim.schedule( msg )
                
            # all demands assigned and everybody is busy again
            self._waiting = self._children.difference( schedule )
            #self._waiting.clear()
            self._demandQ = []
            self.sim.schedule( self.emptied )
            self.emptyFlag = True








            
            
if __name__ == '__main__' :
    from setiptah.eventsim.simulation import Simulation
    from setiptah.queuesim.sources import PoissonClock
    
    from servers import Taxi
    
    
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
    
    """ setup """
    sim = Simulation()
    
    clock = PoissonClock( 5. )
    clock.join_sim( sim )
    
    # prepare geometry queries
    if True :
        ORIGIN = np.zeros(2)
        
        def samplepoint() :
            return np.random.rand(2)
            
        planner = EuclideanPlanner
        
    else :
        import setiptah.roadgeometry.probability as roadprob
        from setiptah.roadgeometry.roadmap_paths import RoadmapPlanner
        
        roadmap = roadprob.sampleroadnet()
        U = roadprob.UniformDist( roadmap )
        samplepoint = U.sample
        
        ORIGIN = samplepoint()
        
        planner = RoadmapPlanner( roadmap )
    
    # instantiate the gate
    gate = TaxiDispatchGated()
    gate.setScheduler( RoundRobin() )
    
    # instantiate the fleet
    TAXI = []
    for i in range(3) :
        taxi = Taxi()
        TAXI.append( taxi )
        
        taxi.setPlanner( planner )
        taxi.setLocation( ORIGIN )
        taxi.setSpeed( 1. )
        
        taxi.join_sim( sim )
        
        gateIF = gate.newInterface()
        gate.add( gateIF )
        gateIF.output.connect( taxi.appendDemands )
        taxi.signalWakeup.connect( gateIF.input )
        taxi.signalIdle.connect( gateIF.input )
        
    gate.join_sim( sim )
        
    def tick() :
        print 'tick, %g' % sim.get_time()
        
    def myarrival() :
        x = samplepoint()
        y = samplepoint()
        
        #print x, y
        time = sim.get_time()
        demand = Taxi.Demand( x, y, time )
        print 'demand generated ', x, y, time
        
        # send the demand to the gate, not any one taxi
        gate.queueDemand( demand )
        
    #def reportPick() : print 'pickup'
    #def reportDelv() : print 'delivery'
    
    #clock.source().connect( tick )
    clock.source().connect( myarrival )
    #veh._arrived.connect( say )
    #veh._pickup.connect( reportPick )
    #veh._deliver.connect( reportDelv )
    
    """ run """
    while sim.get_time() <= 50. :
        callback = sim.get_next_action()
        callback()
    
    
    
    
