
import numpy as np

# dev
from vehicle import Vehicle
from setiptah.eventsim.signaling import Signal, Message
from setiptah.queuesim.misc import Dispatch

DEBUG = False


""" Simple Taxi Agent """

class Taxi :
    """ vehicular agent whose tasks are pickup-and-delivery tasks """
    IDLE = 0
    EMPTY = 1
    FULL = 2
    
    class Demand :
        def __init__(self, orig, dest, arrivalTime=None ) :
            # task spec
            self.origin = orig
            self.destination = dest
            
            # statistics
            self.arrived = arrivalTime
            self.embarked = None
            self.delivered = None
            
            
    def __init__(self) :
        # messaging interface
        self.signalWakeup = Signal()
        self.signalAssigned = Signal()
        self.signalPickup = Signal()         # emited when a demand is picked up
        self.signalDeliver = Signal()        # emited when a demand is delivered
        self.signalIdle = Signal()
        
        # state variables
        self.vehicle = Vehicle()
        #self.vehiclePhase = None
        self._demandQ = []
        
        # vehicle signal connections
        self.vehicle.signalArrived.connect( self.vehicleArrived )
        
        # statistics
        self.demandLog = []
        self.currentDemand = None
        self.vehiclePhase = self.IDLE
        
        #self.odometer_full = 0.
        #self.odometer_empty = 0.
        
    def join_sim(self, sim ) :
        self.sim = sim
        
        # wake-up call
        msg = Message( self.signalWakeup, self.location() )
        self.sim.schedule( msg )
        
        self.vehicle.join_sim( sim )
        
    # slotoid
    def _tryschedule(self) :
        if len( self._demandQ ) > 0 :
            if self.currentDemand is None :
                dem = self._demandQ[0]
                self.vehicle.queueWaypoint( dem.origin )
                self.currentDemand = dem
                self.vehiclePhase = self.EMPTY
            else :
                # we're already working on the first demand
                pass
            
        else :
            # try to idle the taxi
            if not self.vehiclePhase == self.IDLE :
                msg = Message( self.signalIdle, self.location() )
                self.sim.schedule( msg )
                self.vehiclePhase = self.IDLE
                
    """ vehicle configuration pass-thrus """
    def setPlanner(self, planningFunction ) : self.vehicle.setPlanner(planningFunction)
    def setSpeed(self, speed ) : self.vehicle.setSpeed(speed)
    def setLocation(self, location ) : self.vehicle.setLocation(location)
    
    def location(self) : return self.vehicle.location()
    
    """ simulation messaging interface """
    # slot
    def queueDemand(self, demand ) :
        self.demandLog.append( demand )
        if DEBUG :
            time = self.sim.get_time()
            args = ( repr(self), time, repr(demand.origin), repr(demand.destination) )
            #print '%s, got demand at %f: (%s, %s)' % args
            
        self._demandQ.append( demand )
        self._tryschedule()
        
    # slot
    def appendDemands(self, seq ) :
        for dem in seq : self.queueDemand( dem )
        #self.demandLog.extend( seq )
        #self._demandQ.extend( seq )
        #self._tryschedule()
        
    # slot --- switch
    def vehicleArrived(self) :
        if self.vehiclePhase == self.EMPTY :
            self.arrivedOrigin()
        elif self.vehiclePhase == self.FULL :
            self.arrivedDestination()
        else :
            raise 'vehicle should not be moving'
        
    # slot-oid
    def arrivedOrigin(self) :
        demand = self._demandQ[0]
        time = self.sim.get_time()
        demand.embarked = time
        
        self.signalPickup()      # send signal, does this need to be scheduled?
        
        self.vehicle.queueWaypoint( demand.destination )
        self.vehiclePhase = self.FULL
    
    # slot-oid
    def arrivedDestination(self) :
        demand = self._demandQ.pop(0)
        time = self.sim.get_time()
        demand.delivered = time
        
        self.signalDeliver()     # send signal, does this need to be scheduled?
        
        self.currentDemand = None
        self._tryschedule()
        
        








""" Other Taxi Resources """

class TaxiScheduler :
    """ subclassable abstract class for algorithm to schedule taxi demands """
    
    def __call__(self, demands, agentLocations ) :
        """
        demands is an iterable of, e.g., TaxiDemands;
        agentLocations is a dict whose keys are taxi agents and whose values are their locations
        returns as dict whose keys are agents and whose values are lists of their assigned demands, in order
        """
        raise NotImplementedError('please implemented scheduling algorithm')


class RoundRobinScheduler(TaxiScheduler) :
    """ a quick, simple round robin scheduler, for the unit test """
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
            
            
class GatedTaxiDispatch(Dispatch) :
    """
    a gated dispatcher to place between a single demand input stream and a fleet
    of simple Taxi models;
    requires a scheduler
    """
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
        """
        this slot is called (by child, a gate instance) to indicate the other end is now waiting
        """ 
        self._waiting.add( child )
        self._locations[child] = location
        self._try_schedule()
        
    def _try_schedule(self) :
        """ scheduling routine common to several slots """
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
    gate = GatedTaxiDispatch()
    gate.setScheduler( RoundRobinScheduler() )
    
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
    
    
    
    
