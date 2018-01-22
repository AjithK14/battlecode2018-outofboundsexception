import battlecode as bc
import random
import sys
from collections import deque
import traceback
import time
from heapq import heappush, heappop

import os

directions = [bc.Direction.North, bc.Direction.Northeast, bc.Direction.East,
              bc.Direction.Southeast, bc.Direction.South, bc.Direction.Southwest,
              bc.Direction.West, bc.Direction.Northwest]

earthMap = None
marsMap = None
start = None
enemyStart = None
roundsBack = 10
maxRobotMovementHeat = 10

class Node:
    '''mapLocation = None
    depth = None
    goal = None
    manhattan = 0
    children = None'''

    def __init__(self, p, ml, d, go):
        self.mapLocation = ml
        self.parent = p
        self.depth = d
        self.goal = go
        self.manHattan = 0
        self.children = deque()

    def toString(self):
        return str(self.mapLocation.x) + ", " + str(self.mapLocation.y) + ", " + str(self.mapLocation.planet)

    def __lt__(self, other):
        return self.manhattanDistance() < other.manhattanDistance() #not really manhattanDistance, just a heuristic I used

    def expand(self):
        for direction in directions:
            newMapLocation = self.mapLocation.add(direction);
            ######only append if it's on the map and the square is passable
            if self.mapLocation.planet == bc.Planet.Earth and earthMap.on_map(newMapLocation):
                if earthMap.is_passable_terrain_at(newMapLocation):
                    self.children.append(Node(self, newMapLocation, self.depth+1, self.goal))
            elif self.mapLocation.planet == bc.Planet.Mars and marsMap.on_map(newMapLocation):
                if marsMap.is_passable_terrain_at(newMapLocation):
                    self.children.append(Node(self, newMapLocation, self.depth+1, self.goal))
        return self.children

    def manhattanDistance(self):
        return self.mapLocation.distance_squared_to(self.goal)+self.depth

def invert(loc):
    newx = earthMap.width - loc.x
    newy = earthMap.height - loc.y
    return bc.MapLocation(bc.Planet.Earth, newx, newy)


def locToStr(loc):
    return '(' + str(loc.x) + ',' + str(loc.y) + ')'

# pathDict = dict() #set that stores a tuple containing the unit, current location, and the path to its destination


# dict where key is round and value is list of areas that shouldn't be walked on, going 2 rounds back
bannedSquares = dict()
pathDict = dict() #set that stores a tuple containing the unit, current location, and the path to its destination
tryRotate = [0, -1, 1, -2, 2, -4, 4, -3, -3, -5, 5]
deadEndTryRotate = [-3, 3, -5, 5, -4, 4]

def notBanned(mapLocation, round):
    newtup = (mapLocation.planet, mapLocation.x, mapLocation.y)
    start = round - roundsBack
    start = max(start, 1)
    for i in range(start, round):
        if i not in bannedSquares:
            continue
        elif newtup in bannedSquares[i]:
            return False
    return True

def astar(unit, dest):
    if not unit.movement_heat() < maxRobotMovementHeat:
        return
    currentLocation = unit.location.map_location()
    if (unit.id, str(dest)) in pathDict: #the program has saved where this thing has been trying to go
        path = pathDict[unit.id, str(dest)]
        prev = path.popleft().mapLocation
        if len(path) == 0:
            pathDict[unit.id, str(dest)] = None
        d = currentLocation.direction_to(prev)
        if gc.can_move(unit.id, d):
            print ("sice me")
            gc.move_robot(unit.id, d)
        else: #at this point, there is clearly an obstable, such as a factory in the way.  Calling bugnav here to try to continue a*
            newDest = path.popleft().mapLocation
            go_to(unit, newDest)
    else: #the first time this program is trying to make the unit get to this destination
        startState = Node(None, currentLocation, 0, dest)
        prev = set()
        fringe = []
        fringe.append(startState)
        while True:
            if len(fringe) == 0:
                return '-'
            node = heappop(fringe)
            # print (node.state)
            if node.mapLocation.distance_squared_to(node.goal) == 0:
                path = deque()
                while node != None:
                    path.append(node)
                    node = node.parent
                path.reverse() #because it's in reverse order
                pathDict[unit.id, str(dest)] = path
            else:
                children = node.expand()
                for i in range(len(children)):
                    if str(children[i].mapLocation) not in prev:
                        prev.add(str(children[i].mapLocation))
                        heappush(fringe, children[i])

def go_to(unit, dest):  # using bugnav
    # assuming dest is a MapLocation
    if not unit.movement_heat() < maxRobotMovementHeat:
        return
    currentLocation = unit.location.map_location()
    d = currentLocation.direction_to(dest)
    if d == bc.Direction.Center:
        return
    round = gc.round()
    if gc.can_move(unit.id, d) and notBanned(currentLocation.add(d), round): #might take out the notbanned thing
        gc.move_robot(unit.id, d)
        tup = (currentLocation.planet, currentLocation.x, currentLocation.y)
        if round not in bannedSquares:
            l = list()
            l.append(tup)
            bannedSquares[round] = l
        else:
            bannedSquares[round].append(tup)
    else:
        fuzzy_go_to(unit, dest)


def rotate(dir, amount):
    ind = directions.index(dir)
    return directions[(ind + amount) % 8]


def fuzzy_go_to(unit, dest):
    if not unit.movement_heat() < maxRobotMovementHeat:
        return
    currentLocation = unit.location.map_location()
    toward = currentLocation.direction_to(dest)
    round = gc.round()
    for tilt in tryRotate:
        d = rotate(toward, tilt)
        if gc.can_move(unit.id, d) and notBanned(currentLocation.add(d), round): #might take out the notbanned thing
            gc.move_robot(unit.id, d)
            tup = (currentLocation.planet, currentLocation.x, currentLocation.y)
            if round not in bannedSquares:
                l = list()
                l.append(tup)
                bannedSquares[round] = l
            else:
                bannedSquares[round].append(tup)
            return


def go_to_coordinates(unit, x, y, planet):
    dest = bc.MapLocation(planet, x, y)
    return go_to(unit, dest)


print("pystarting")

# A GameController is the main type that you talk to the game with.
# Its constructor will connect to a running game.
gc = bc.GameController()


print("pystarted")


# It's a good idea to try to keep your bots deterministic, to make debugging easier.
# determinism isn't required, but it means that the same things will happen in every thing you run,
# aside from turns taking slightly different amounts of time due to noise.
random.seed(6137)

# let's start off with some research!
# we can queue as much as we want.

if gc.planet() == bc.Planet.Earth:  # initializing the map, and starting locations
    myStart = gc.my_units()[0].location.map_location()
    earthMap = gc.starting_map(bc.Planet.Earth)
    marsMap = gc.starting_map(bc.Planet.Mars)
    enemyStart = invert(myStart)

print(os.getcwd())
gc.queue_research(bc.UnitType.Rocket)
gc.queue_research(bc.UnitType.Worker)
gc.queue_research(bc.UnitType.Knight)

my_team = gc.team()

while True:
    # We only support Python 3, which means brackets around print()
    round = gc.round()

    if gc.planet() == bc.Planet.Earth:
        firstMan = gc.my_units()[0]
        print('pyround:', round, 'time left:', gc.get_time_left_ms(), 'ms',
      'heat:', firstMan.movement_heat(), 'loc: ', firstMan.location.map_location())
        randomLocation = bc.MapLocation(bc.Planet.Earth, 0, 0)

        astar(firstMan, randomLocation)
    # frequent try/catches are a good idea
    '''try:
       #clearing banned squares for pathfinding
       if(round >= 1 + roundsBack):
           for i in range(round-roundsBack, round):
               bannedSquares[i] = None
       # walk through our units:
       for unit in gc.my_units():

           # first, factory logic
           if unit.unit_type == bc.UnitType.Factory:
               garrison = unit.structure_garrison()
               if len(garrison) > 0:
                   d = random.choice(directions)
                   if gc.can_unload(unit.id, d):
                       print('unloaded a knight!')
                       gc.unload(unit.id, d)
                       continue
               elif gc.can_produce_robot(unit.id, bc.UnitType.Knight):
                   gc.produce_robot(unit.id, bc.UnitType.Knight)
                   print('produced a knight!')
                   continue

           # first, let's look for nearby blueprints to work on
           location = unit.location
           if location.is_on_map():
               nearby = gc.sense_nearby_units(location.map_location(), 2)
               for other in nearby:
                   if unit.unit_type == bc.UnitType.Worker and gc.can_build(unit.id, other.id):
                       gc.build(unit.id, other.id)
                       print('built a factory!')
                       # move onto the next unit
                       continue
                   if other.team != my_team and gc.is_attack_ready(unit.id) and gc.can_attack(unit.id, other.id):
                       print('attacked a thing!')
                       gc.attack(unit.id, other.id)
                       continue

           # okay, there weren't any dudes around
           # pick a random direction:
           d = random.choice(directions)

           # or, try to build a factory:
           if gc.karbonite() > bc.UnitType.Factory.blueprint_cost() and gc.can_blueprint(unit.id, bc.UnitType.Factory, d):
               gc.blueprint(unit.id, bc.UnitType.Factory, d)
           # and if that fails, try to move
           elif gc.is_move_ready(unit.id) and gc.can_move(unit.id, d):
               gc.move_robot(unit.id, d)

   except Exception as e:
       print('Error:', e)
       # use this to show where the error was
       traceback.print_exc()'''

    # send the actions we've performed, and wait for our next turn.
    gc.next_turn()

    # these lines are not strictly necessary, but it helps make the logs make more sense.
    # it forces everything we've written this turn to be written to the manager.
    sys.stdout.flush()
    sys.stderr.flush()
