import battlecode as bc
import random
import sys
from collections import deque
import math
import traceback
import time
from heapq import heappush, heappop

import os

directions = [bc.Direction.North, bc.Direction.Northeast, bc.Direction.East,
              bc.Direction.Southeast, bc.Direction.South, bc.Direction.Southwest,
              bc.Direction.West, bc.Direction.Northwest]
gc = bc.GameController()
robots = [bc.UnitType.Worker, bc.UnitType.Knight,
    bc.UnitType.Ranger, bc.UnitType.Mage, bc.UnitType.Healer]

earthMap = None
marsMap = None
start = None
enemyStart = None
roundsBack = 10
maxRobotMovementHeat = 10

workerHarvestAmount = 3


def factory_move(robot_id):  # move robot away from factory -- this method takes the robot id
  for i in direction:
    if gc.can_move(robot_id, i):
      gc.move_robot(robot_id, d)
      return


class Node:
    '''mapLocation = None
    depth = None
    goal = None
    manhattan = 0
    children = None'''

    def __init__(self, p, ml, d, go, u):
        self.mapLocation = ml
        self.parent = p
        self.depth = d
        self.goal = go
        self.manHattan = 0
        self.children = deque()
        self.unit = u

    def toString(self):
        return str(self.mapLocation.x) + ", " + str(self.mapLocation.y) + ", " + str(self.mapLocation.planet)

    def __lt__(self, other):
        # not really manhattanDistance, just a heuristic I used
        return self.heuristic() < other.heuristic()

    def expand(self):
        for direction in directions:
            newMapLocation = self.mapLocation.add(direction);
            # only append if it's on the map and the square is passable
            if self.mapLocation.planet == bc.Planet.Earth and earthMap.on_map(newMapLocation):
                if earthMap.is_passable_terrain_at(newMapLocation):
                    self.children.append(
                        Node(self, newMapLocation, self.depth + 1, self.goal, self.unit))
            elif self.mapLocation.planet == bc.Planet.Mars and marsMap.on_map(newMapLocation):
                if marsMap.is_passable_terrain_at(newMapLocation):
                  self.children.append(
                      Node(self, newMapLocation, self.depth + 1, self.goal, self.unit))
        return self.children

    def heuristic(self):
      if self.mapLocation.planet == bc.Planet.Earth:
        # change .1 to a function of current karbonite, and round numbers
        return math.sqrt(self.mapLocation.distance_squared_to(self.goal)) + self.depth - ((.1) * (earthMap.initial_karbonite_at(self.mapLocation)))
      else:
        # change .1 to a function of current karbonite, and round numbers
        return math.sqrt(self.mapLocation.distance_squared_to(self.goal)) + self.depth - ((.1) * (marsMap.initial_karbonite_at(self.mapLocation)))


def invert(loc):
    newx = earthMap.width - loc.x
    newy = earthMap.height - loc.y
    return bc.MapLocation(bc.Planet.Earth, newx, newy)


def locToStr(loc):
    return '(' + str(loc.x) + ',' + str(loc.y) + ')'

# pathDict = dict() #set that stores a tuple containing the unit, current location, and the path to its destination


# dict where key is round and value is list of areas that shouldn't be walked on, going 2 rounds back
bannedSquares = dict()
pathDict = dict()  # set that stores a tuple containing the unit, current location, and the path to its destination
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


def harvestKarbonite(unit, currentLocation):
  amount = gc.karbonite_at(currentLocation)
  max = 0
  maxDir = None
  if amount > workerHarvestAmount:
    if gc.can_harvest(unit.id, bc.Direction.Center):
      gc.harvest(unit.id, bc.Direction.Center)
  for direction in directions:
    newLoc = currentLocation.add(direction)
    if earthMap.on_map(newLoc):
      cur = gc.karbonite_at(newLoc)
      if cur > max:
        max = cur
        maxDir = direction

  if max > 0:
    if gc.can_harvest(unit.id, maxDir):
      gc.harvest(unit.id, maxDir)


def astar(unit, dest):
    if not unit.movement_heat() < maxRobotMovementHeat:
        return
    currentLocation = unit.location.map_location()
    # robots[0] is worker
    if unit.unit_type == robots[0] and gc.karbonite() < optimalKarboniteAmount:
      # print (gc.karbonite())
      harvestKarbonite(unit, currentLocation)
    if currentLocation.direction_to(dest) == bc.Direction.Center:
      pathDict.pop(unit.id, str(dest))
      return
    if (unit.id, str(dest)) in pathDict:  # the program has saved where this thing has been trying to go
        path = pathDict[unit.id, str(dest)]
        prev = path[0].mapLocation
        # had used bugnav recently and not completely finished
        if currentLocation.is_adjacent_to(prev) == False:
          print (str(currentLocation) + " p:" + str(prev))
          go_to(unit, prev)
          return
        prev = path.popleft().mapLocation
        if len(path) == 0:
            pathDict[unit.id, str(dest)] = None
        d = currentLocation.direction_to(prev)
        if gc.can_move(unit.id, d):
            print ("sice me")
            gc.move_robot(unit.id, d)
            # path.popleft()
        else:  # at this point, there is clearly an obstable, such as a factory in the way.  Calling bugnav
            newDest = path.popleft().mapLocation
            go_to(unit, newDest)
    else:  # the first time this program is trying to make the unit get to this destination
        startState = Node(None, currentLocation, 0, dest, unit)
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
                path.reverse()  # because it's in reverse order
                path.popleft()
                pathDict[unit.id, str(dest)] = path
                astar(unit, dest)
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
    # might take out the notbanned thing
    if gc.can_move(unit.id, d) and notBanned(currentLocation.add(d), round):
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
        # might take out the notbanned thing
        if gc.can_move(unit.id, d) and notBanned(currentLocation.add(d), round):
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
gc.queue_research(bc.UnitType.Rocket)  # necessary
gc.queue_research(bc.UnitType.Worker)
gc.queue_research(bc.UnitType.Knight)

my_team = gc.team()
STAYERS = 4  # number of robots who stay on earth
# controls whether our karbonite-harvesting group (khg) has reached mars yet.
touchedMars = False
KHGWORKERS = 2
KHGKNIGHTS = 2
KHGRANGERS = 1
KHGMAGES = 1
KHGHEALERS = 2

earthRocketLocations = list()
baseLocations = list()

KHGARRAY = [KHGWORKERS, KHGKNIGHTS, KHGRANGERS, KHGMAGES, KHGHEALERS]
INITIALKHGARRAY = [KHGWORKERS + STAYERS, KHGKNIGHTS + STAYERS,
    KHGRANGERS + STAYERS, KHGMAGES + (STAYERS * 0), KHGHEALERS + STAYERS]
factoryIndex = 0  # controls what the different factories do

earthWorkers = 0
earthKnights = 0
earthRangers = 0
earthMages = 0
earthHealers = 0
maxRocketGarrison = 8
# key is type of robot number [0 to 4], planet.  Value is maplocation, errorRadius, number
whereTo = dict()
first_rocket = False
firstRocketBuilt = False

# seeing if this is really the first rocket to mars (what word does "vrgn" remind you of?)
vrgn = True


def getRobotProportions(round):
  return KHGARRAY  # will change the proportions so that it is a fnction of round


while True:
    # We only support Python 3, which means brackets around print()
    round = gc.round()
    if round == 126:
      workerHarvestAmount += 1
    if gc.planet() == bc.Planet.Earth:
        firstMan = gc.my_units()[0]
        '''  print('pyround:', r ound, 'time left:', gc.get_time_left_ms(), 'ms',
        'heat:', firstMan.movement_heat(), 'loc: ', firstMan.location.map_location())'''
        print ('pyround:', round, 'karbonite', gc.karbonite())
        randomLocation = bc.MapLocation(bc.Planet.Earth, 0, 0)

        # astar(firstMan, randomLocation)

    # frequent try/catches are a good idea
    try:
        # possibly useless piece of code begins
        if(round >= 1 + roundsBack):
            for i in range(round - roundsBack, round):
                bannedSquares[i] = None
         # possibly useless piece of code ends

        for unit in gc.my_units():

          if unit.unit_type == bc.UnitType.Rocket:
            if gc.planet() == bc.Planet.Earth:
              garrison == unit.structure_garrison()
              countNeeded = 8
              if vrgn == False:
                countNeeded = 6
              if len(garrison) >= countNeeded and len(garrison) <= maxRocketGarrison:
                tempPlanetMap = gc.planet_map(bc.Planet.Mars)
                tempLoc = MapLocation(bc.Planet.Mars, (int)(
                    Mars, tempPlanetMap.width / 4), (int)(Mars, tempPlanetMap.height / 4))
                if gc.can_launch_rocket(unit.id, tempLoc):
                  gc.launch_rocket(unit.id, tempLoc)
                  vrgn = False
            else:
              if len(garrison) > 0:
                d = random.choice(directions)  # good for now, change later
                if gc.can_unload(unit.id, d):
                  print ("unloaded")
                  gc.unload(unit.id, d)
                  continue
                else:
                  for tilt in tryRotate:
                    newD = rotate(d, tilt)
                    while gc.can_unload(unit.id, d):
                      print ("unloaded")
                      gc.unload(unit.id, d)
                      factory_move(gc.unit(unit.id))

          if unit.unit_type == bc.UnitType.Factory:
              garrison = unit.structure_garrison()

              if len(garrison) > 0:
                d = random.choice(directions)  # good for now, change later
                if gc.can_unload(unit.id, d):
                  print ("unloaded")
                  gc.unload(unit.id, d)
                  continue
                else:
                  for tilt in tryRotate:
                    newD = rotate(d, tilt)
                    while gc.can_unload(unit.id, d):
                      print ("unloaded")
                      gc.unload(unit.id, d)
                      factory_move(gc.unit(unit.id))
                    break
              if touchedMars == False:
                currentRobotArray = [0, 0, 0, 0, 0]
                for unit in gc.my_units():
                    if unit.unit_type == gc.UnitType.Worker:
                      currentRobotArray[0] += 1
                    elif unit.unit_type == gc.UnitType.Knight:
                        currentRobotArray[1] += 1
                    elif unit.unit_type == gc.UnitType.Rangers:
                        currentRobotArray[2] += 1 
                    elif unit.unit_type == gc.UnitType.Mage:
                        currentRobotArray[3] += 1
                    elif unit.unit_type == gc.UnitType.Healer:
                        currentRobotArray[4] += 1

                    deficit = [INITIALKHGARRAY[0] - currentRobotArray[0],
                         INITIALKHGARRAY[1] - currentRobotArray[1],
                         INITIALKHGARRAY[2] - currentRobotArray[2],
                         INITIALKHGARRAY[3] - currentRobotArray[3],
                         INITIALKHGARRAY[4] - currentRobotArray[4]]
                if max(deficit) <= 1: #start calling the players to the first rocket location, modify this condition if necessary
                  if len(earthRocketLocations > 0):
                    for i in range(len(robots)):
                      whereTo[i, str(bc.Planet.Earth)] = earthRocketLocations[0], 1, KHGARRAY[i]
                      
                  '''else: #we're probably not building a base rn
                    for i in range(len(robots)):
                      whereTo[i, bc.Planet.Earth] = baseLocations[0].x, baseLocations[0].y, 2, KHGARRAY[i]'''
                for i in range(len(deficit)):
                    robotType = deficit.index(max(deficit))
                    if gc.can_produce_robot(unit.id, robotType):
                        gc.produce_robot(unit.id, robotType)
                        print('produced a robot!')
                        continue
              else: #touchedMars = true
                robotProportions = getRobotProportions(round)
                # build general robots here
                if gc.can_produce_robot(unit.id, bc.UnitType.Knight):
                    gc.produce_robot(unit.id, bc.UnitType.Knight)
                    print('produced a knight!')
                    continue

          objectNumID = -1
          for i in range(len(robots)):
            if unit.unit_type == robots[i]:
            objectNumID = i
            break
          if objectNumID != -1: #meaning this unit is a robot
            if objectNumID == 0: #meaning that this unit is a worker
              if not first_rocket:
                for q in directions:
                  if gc.karbonite() > bc.UnitType.Rocket.blueprint_cost() and gc.can_blueprint(unit.id,bc.UnitType.Rocket,q):
                    gc.blueprint(unit.id,bc.UnitType.Rocket,q)
                    print("ROCKET BLUEPRINTED YAH")
                    rocketLocation = gc.unit(unit.id).mapLocation().add(q)
                    earthRocketLocations.append(rocketLocation)
                    whereTo[0, str(gc.planet())] = rocketLocation, 1, 1 #calling workers here
                    first_rocket = True
                    break
            if objectNumID, str(gc.planet()) in whereTo:
              tup = whereTo[objectNumID, str(gc.planet())]
              dest = tup[0]
              errorRadius = tup[1]
              num = tup[2]
              astar(res, dest)
              if math.sqrt(unit.location.map_location().distance_squared_to(dest)) < errorRadius:
                num -= 1
                if num <= 0:
                  whereTo.pop((objectNumID, str(gc.planet()))
                else:
                  whereTo[objectNumID, str(gc.planet())] = dest, errorRadius, num

          if gc.karbonite() > 800: #want more rockets now
            first_rocket = False
            firstRocketBuilt = False

          location = unit.location
          if location.is_on_map():
              nearby = gc.sense_nearby_units(location.map_location(), 4)
              # ajith your strat starts here
              for other in nearby:
                  if not firstRocketBuilt and unit.unit_type == bc.UnitType.Worker and gc.can_build(unit.id, other.id):
                      gc.build(unit.id, other.id)
                      print('built a rocket!')
                      firstRocketBuilt = True
                      continue
                  if unit.unit_type == bc.UnitType.Rocket and gc.can_load(other.id,unit.id):
                      gc.load(other.id,unit.id)
                      print('loaded into the rocket!')
                  if other.team != my_team and gc.is_attack_ready(unit.id) and gc.can_attack(unit.id, other.id):
                      print('attacked a thing!')
                      gc.attack(unit.id, other.id)
                      continue
                  if unit.unit_type == bc.UnitType.Worker and gc.can_build(unit.id, other.id):
                      gc.build(unit.id, other.id)
                      print('built a factory!')
                      continue

              # ajith your strat ends here
          # okay, there weren't any dudes around
          # pick a random direction:
          d = random.choice(directions)

          # or, try to build a factory:
          if gc.karbonite() > bc.UnitType.Factory.blueprint_cost() and gc.can_blueprint(unit.id, bc.UnitType.Factory, d):
              gc.blueprint(unit.id, bc.UnitType.Factory, d)
          # and if that fails, try to move
          astar(unit, unit.location.map_location().add(d))

       
    except Exception as e:
        print('Error:', e)
        # use this to show where the error was
        traceback.print_exc()

    # send the actions we've performed, and wait for our next turn.
    gc.next_turn()

    # these lines are not strictly necessary, but it helps make the logs make more sense.
    # it forces everything we've written this turn to be written to the manager.
    sys.stdout.flush()
    sys.stderr.flush()
