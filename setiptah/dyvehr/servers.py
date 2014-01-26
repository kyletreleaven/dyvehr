
import numpy as np
from setiptah.eventsim.signaling import Signal, Message

DEBUG = False


class Vehicle :
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
        self._arrived = Signal()
        
        
    """ configuration API """
    def setPlanner(self, planningFunction ) :
        """
        to support very general workspaces:
        planningFunction should give a tuple ( S, x(t) ), of 
            a total distance S, and
            a trajectory x(t) returning location for all s \in [0,S]
        """
        self._planner = planningFunction
        
    def setSpeed(self, speed ) :
        self._speed = speed
        
    def setLocation(self, location ) :
        self._location = location
        
        
    """ simulator interface """
    def join_sim(self, sim ) :
        self.sim = sim
        self._lastUpdate = self.sim.get_time()
    
    def _stateUpdate(self ) :
        currTime = self.sim.get_time()
        prevTime = self._lastUpdate
        self._lastUpdate = currTime
        
        # update location if we are moving
        if self._trajectory is not None :
            elapsed = currTime - prevTime
            progress = elapsed * self._speed
            self.odometer += progress
            self._trajProgress += progress
            self._location = self._trajectory( self._trajProgress )
                
    def _tryschedule(self) :
        # utility
        if self._pendingEvent is None and len( self._waypoints ) > 0 :
            target = self._waypoints[0]
            trajLength, traj = self._planner( self._location, target )
            self._trajectory = traj
            self._trajProgress = 0.
            self._pendingEvent = self.sim.schedule( self.arrived, trajLength / self._speed )
    
    """ messaging interface """
    # auto-slot
    def arrived(self) :
        self._stateUpdate()
        
        """ have arrived at the waypoint """
        self._pendingEvent = None
        self._location = self._waypoints.pop(0)
        self._trajectory = None
        
        self._arrived()     # report arrival
        self._tryschedule() # schedule another waypoint if available
        
    # slot
    def queueWaypoint(self, waypt ) :
        # an update could be done, but isn't really necessary
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
        trajLength = np.linalg.norm( np.array(dest) - np.array(orig) )
        traj = EuclideanTraj( orig, dest )
        return trajLength, traj
    
    """ setup """
    sim = Simulation()
    
    clock = PoissonClock()
    clock.join_sim( sim )
    
    veh = Vehicle()
    veh.setPlanner( EuclideanPlanner )
    veh.setLocation( np.zeros(2) )
    veh.setSpeed( 1. )
    veh.join_sim( sim )
    
    def tick() :
        print 'tick, %g' % sim.get_time()
        
    def myarrival() :
        x = np.random.rand(2)
        veh.queueWaypoint( x )
        print 'point generated ', x
        
    def say() :
        print 'arrived'    
    
    clock.source().connect( tick )
    clock.source().connect( myarrival )
    veh._arrived.connect( say )
    
    
    """ run """
    while sim.get_time() <= 50. :
        callback = sim.get_next_action()
        callback()
    
    

if False :
    
    
    class Taxi :
        def __init__(self) :
            self.ready = Signal()
            self.odometer = 0.
            self.odometer_full = 0.
            self.odometer_empty = 0.
            
            self.mylog = []
            self.notches = 0
            
        def set_environment(self, f_dist ) :
            self.f_dist = f_dist
            
        def set_speed(self, speed ) :
            self.speed = speed
            
        def set_location(self, location ) :
            self.location = location
            
        def join_sim(self, sim ) :
            self.sim = sim
            self.sim.schedule( self.ready_at )
            
        def log_timefull( self, time ) :
            self.odometer_full += time
            self.odometer += time
            
        def log_timeempty( self, time ) :
            self.odometer_empty += time
            self.odometer += time
            
            
        """ signaloid """
        def ready_at(self) : self.ready( self.location )
        
        """ slot """
        def receive_demand(self, demand ) :
            self.mylog.append( demand )     # record the demand
            
            p, q = demand
            time = self.sim.get_time()
            if True : 
                print '%s, got demand at %f: (%s, %s)' % ( repr( self ), time, repr(p), repr(q) )
            
            self.pick = p ; self.delv = q
            
            dist_curr_to_pick = self.f_dist( self.location, p )
            time_curr_to_pick = dist_curr_to_pick / self.speed
            if DEBUG : print 'moving to pickup, there in %f' % time_curr_to_pick
            
            self.next_dist = dist_curr_to_pick
            self.sim.schedule( self.arrived_at_pickup, time_curr_to_pick )
            
        """ auto slot """
        def arrived_at_pickup(self) :
            time = self.sim.get_time()
            self.log_timeempty( self.next_dist )
            self.location = self.pick
            if DEBUG : print 'arrived to pickup at %f' % time
            
            q = self.delv
            dist_pick_to_delv = self.f_dist( self.location, q )
            time_pick_to_delv = dist_pick_to_delv / self.speed
            if DEBUG : print 'moving to delivery, there in %f' % time_pick_to_delv
            
            self.next_dist = dist_pick_to_delv
            self.sim.schedule( self.delivered, time_pick_to_delv )
            
        """ auto slot """
        def delivered(self) :
            time = self.sim.get_time()
            self.log_timefull( self.next_dist )
            self.location = self.delv
            time = self.sim.get_time()
            if DEBUG : print 'delivered at %f' % time
            
            self.notches = self.notches + 1
            
            self.ready_at()
    
    





