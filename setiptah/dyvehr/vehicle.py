
import numpy as np
from setiptah.eventsim.signaling import Signal, Message

DEBUG = True

class VehicleTrajectory :
    """
    pattern (purely abstract, and JUST FOR SHOW)
    of the layout of a geometrical planner
    """
    def __call__( s ) :
        """
        given any s, 0 <= s <= length of the trajectory,
        __call__ should return the location at distance s along the trajectory
        """
        raise NotImplementedError('please implement vehicle trajectory __call__')
    
    
class VehiclePlanner :
    def __call__(self, startLoc, endLoc ) :
        """ return a trajectory (see above) from startLoc to endLoc """
        raise NotImplementedError('planner not implemented')




class Vehicle :
    """
    The vehicle is not itself a task-fulfilling agent,
    but an automata which will handle waypoint tracking, to facilitate
    multiple vehicular agent types
    """
    def __init__(self) :
        # parameters
        self._planner = None
        # sim objects
        self.sim = None
        
        # state variables        
        self._lastUpdate = None
        self._location = None
        self._trajectory = None
        self._trajProgress = None
        
        self._pendingEvent = None
        self._waypoints = []
        
        self._speed = None
        self.odometer = 0.
                
        # SIGNALS
        self.signalArrived = Signal()
        
        
    """ configuration API """
    def setPlanner(self, planningFunction ) :
        """
        to support very general workspaces:
        planningFunction should give a tuple ( S, x(t) ), of 
            a total distance to target, S, and
            a trajectory x(t) returning the location of the point s distance along x,
                for all s \in [0,S]
        """
        self._planner = planningFunction
        
    def setSpeed(self, speed ) :
        self._speed = speed
        
    def setLocation(self, location ) :
        self._location = location
        
    def location(self) :
        return self._location
        
        
    """ simulator interface """
    def join_sim(self, sim ) :
        self.sim = sim
        self._lastUpdate = self.sim.get_time()
    
    def _stateUpdate(self ) :
        prevTime = self._lastUpdate
        currTime = self.sim.get_time()
        self._lastUpdate = currTime
        
        # update location if we are moving
        if self._trajectory is not None :
            elapsed = currTime - prevTime
            progress = elapsed * self._speed
            self.odometer += progress
            self._trajProgress += progress
            #print 'progress: %f / %f' % ( self._trajProgress, self._trajLength )
            #print self._trajProgress - self._trajLength
            
            try :
                newloc = self._trajectory( self._trajProgress )
                self.setLocation( newloc )
            except :
                print self._trajectory, self._trajProgress
                raise
            
    def _tryschedule(self) :
        # utility
        if self._pendingEvent is None and len( self._waypoints ) > 0 :
            target = self._waypoints[0]
            trajLength, traj = self._planner( self.location(), target )
            #print trajLength, traj
            self._trajectory = traj
            self._trajProgress = 0.
            self._trajLength = trajLength
            
            self._pendingEvent = self.sim.schedule( self.arrived, trajLength / self._speed )
    
    """ messaging interface """
    # auto-slot
    def arrived(self) :
        """ have arrived at the waypoint """
        self._pendingEvent = None
        target = self._waypoints.pop(0)
        self.setLocation( target )
        self._trajectory = None
        
        self._stateUpdate()
        
        self.signalArrived()     # report arrival
        self._tryschedule() # schedule another waypoint if available
        
    # slot
    def queueWaypoint(self, waypt ) :
        # an update could be done, but isn't really necessary
        # false! an update is *totally* necessary
        self._stateUpdate()
        self._waypoints.append( waypt )
        self._tryschedule()
        
    # slot
    def cancelAll(self) :
        # need to update location
        self._stateUpdate()
        
        # cancel arrival
        next = self._pendingEvent
        next.deactivate()
        self._pendingEvent = None
        # stop moving
        self._trajectory = None
        # empty the queue
        self._waypoints = []
        
        
        
        
        
        
if __name__ == '__main__' :
    from setiptah.eventsim.simulation import Simulation
    from setiptah.queuesim.sources import PoissonClock
    
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
    
    clock = PoissonClock()
    clock.join_sim( sim )
    
    #veh = Vehicle()
    veh = Taxi()
    
    if False :
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
    
    veh.setPlanner( planner )
    veh.setLocation( ORIGIN )
    veh.setSpeed( 1. )
    veh.join_sim( sim )
    
    def tick() :
        print 'tick, %g' % sim.get_time()
        
    def myarrival() :
        x = samplepoint()
        y = samplepoint()
        
        #print x, y
        time = sim.get_time()
        demand = Taxi.Demand( x, y, time )
        print 'demand generated ', x, y, time
        
        veh.queueDemand( demand )
        
    def reportPick() : print 'pickup'
    def reportDelv() : print 'delivery'
    
    #clock.source().connect( tick )
    clock.source().connect( myarrival )
    #veh._arrived.connect( say )
    veh.signalPickup.connect( reportPick )
    veh.signalDeliver.connect( reportDelv )
    
    """ run """
    while sim.get_time() <= 50. :
        callback = sim.get_next_action()
        callback()
    
    





