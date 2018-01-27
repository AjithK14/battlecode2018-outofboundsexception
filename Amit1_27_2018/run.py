"""
MAJOR CHANGES TO BE TRANSFERRED:

A* METHOD
DIFFERENTIATING MAP (ALREADY DONE SO JUST PASTE IN THE METHOD)
--a* method assumes map has been created
------A* ONLY TO BE USED FOR ATTACKERS !!!!!!!
"""




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
allDirections = [bc.Direction.North, bc.Direction.Northeast, bc.Direction.East,
              bc.Direction.Southeast, bc.Direction.South, bc.Direction.Southwest,
              bc.Direction.West, bc.Direction.Northwest]
factoryCosts = [25, 20, 20, 20, 20]
gc = bc.GameController()

robots = [bc.UnitType.Worker, bc.UnitType.Knight, bc.UnitType.Ranger, bc.UnitType.Mage, bc.UnitType.Healer]

## IDs
MARSPLANETMAP = gc.starting_map(bc.Planet.Mars)
EARTHPLANETMAP = gc.starting_map(bc.Planet.Earth)
workerNum = 0
knightNum = 1
rangerNum = 2
mageNum = 3
healerNum = 4

earthMap = gc.starting_map(bc.Planet.Earth)
marsMap = None
start = None
enemyStart = None
roundsBack = 10
maxRobotMovementHeat = 10
maxRocketGarrison = 6

class mmap():
  def __init__(self,width,height):
    self.width=width
    self.height=height
    self.arr=[[0]*self.height for i in range(self.width)];
  def onMap(self,loc):
    if (loc.x<0) or (loc.y<0) or (loc.x>=self.width) or (loc.y>=self.height): return False
    return True
  def get(self,mapLocation):
    if not self.onMap(mapLocation):return -1
    return self.arr[mapLocation.x][mapLocation.y]
  def set(self,mapLocation,val):
    self.arr[mapLocation.x][mapLocation.y]=val
  def printout(self):
    print('printing map:')
    for y in range(self.height):
      buildstr=''
      for x in range(self.width):
        buildstr+=format(self.arr[x][self.height-1-y],'2d')
      print(buildstr)
  def addDisk(self,mapLocation,r2,val):
    locs = gc.all_locations_within(mapLocation,r2)
    for loc in locs:
      if self.onMap(loc):
        self.set(loc,self.get(loc)+val)
  def multiply(self,mmap2):
    for x in range(self.width):
      for y in range(self.height):
        ml = bc.MapLocation(bc.Planet.Earth,x,y);
        self.set(ml,self.get(ml)*mmap2.get(ml))
  def findBest(self,mapLocation,r2):
    locs = gc.all_locations_within(mapLocation,r2)
    bestAmt = 0
    bestLoc = None
    for loc in locs:
      amt = self.get(loc)
      if amt>bestAmt:
        bestAmt=amt
        bestLoc=loc
    return bestAmt, bestLoc
passableMap = mmap(earthMap.width,earthMap.height);
kMap = mmap(earthMap.width,earthMap.height);
for x in range(earthMap.width):
  for y in range(earthMap.height):
    ml = bc.MapLocation(bc.Planet.Earth,x,y);
    passableMap.set(ml,earthMap.is_passable_terrain_at(ml))
    kMap.set(ml,earthMap.initial_karbonite_at(ml))
workerHarvestAmount = 3
#generate an ordered list of karbonite locations, sorted by distance to start
tOrderStart=time.time()
kLocs = []
currentLocs = []
evalMap = mmap(earthMap.width,earthMap.height)
for unit in gc.my_units():
  currentLocs.append(unit.location.map_location())
while(len(currentLocs)>0):
  nextLocs = []
  for loc in currentLocs:
    for dir in directions:
      newPlace = loc.add(dir)
      if evalMap.get(newPlace)==0:
        evalMap.set(newPlace,1)
        nextLocs.append(newPlace)
        if kMap.get(newPlace)>0:
          kLocs.append(loc)
  currentLocs=nextLocs

def factory_move(unit): #move robot away from factory -- this method takes the robot itself
  for i in directions:
    if gc.can_move(robot_id,i):
      gc.move_robot(robot_id,i)
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
        return self.heuristic() < other.heuristic() #not really manhattanDistance, just a heuristic I used

    def expand(self):
        for direction in directions:
            newMapLocation = self.mapLocation.add(direction);
            ######only append if it's on the map and the square is passable
            if self.mapLocation.planet == bc.Planet.Earth and earthMap.on_map(newMapLocation):
                if earthMap.is_passable_terrain_at(newMapLocation):
                    self.children.append(Node(self, newMapLocation, self.depth+1, self.goal, self.unit))
            elif self.mapLocation.planet == bc.Planet.Mars and marsMap.on_map(newMapLocation):
                if marsMap.is_passable_terrain_at(newMapLocation):
                  self.children.append(Node(self, newMapLocation, self.depth+1, self.goal, self.unit))
        return self.children

    def heuristic(self):
      if self.mapLocation.planet == bc.Planet.Earth:
        return math.sqrt(self.mapLocation.distance_squared_to(self.goal))+self.depth-((.1)*(earthMap.initial_karbonite_at(self.mapLocation))) #change .1 to a function of current karbonite, and round numbers
      else:
        return math.sqrt(self.mapLocation.distance_squared_to(self.goal))+self.depth-((.1)*(marsMap.initial_karbonite_at(self.mapLocation))) #change .1 to a function of current karbonite, and round numbers
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
my_team = gc.team()
if my_team==bc.Team.Blue:enemy_team = bc.Team.Red
else: enemy_team = bc.Team.Blue

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

def harvestKarbonite(unit):
  mostK, bestDir = bestKarboniteDirection(unit.location.map_location())
  if mostK>0:#found some karbonite to harvest
    if gc.can_harvest(unit.id,bestDir):
      gc.harvest(unit.id,bestDir)
      return
  elif gc.is_move_ready(unit.id):#need to go looking for karbonite
    if len(kLocs)>0:
      dest=kLocs[0]
      if gc.can_sense_location(dest):
        kAmt = gc.karbonite_at(dest)
        if kAmt==0:
          kLocs.pop(0)
        else:
          fuzzygoto(unit,dest)

def closedIn(unit):
  counter = 0
  curLoc = unit.location.map_location()
  for d in directions:
    newLoc = curLoc.add(d)
    if earthMap.on_map(newLoc) == False and marsMap.on_map(newLoc) == False:
      continue
    if gc.is_occupiable(newLoc):
      counter += 1
  if counter > 1:
    return True
  else:
    return False

def onEarth(loc):
  return True

def checkK(loc):
  b1 = loc.planet == bc.Planet.Earth and 0<=loc.x<EARTHPLANETMAP.width and 0<=loc.y<EARTHPLANETMAP.height
  b2 = loc.planet == bc.Planet.Mars and 0<=loc.x<MARSPLANETMAP.width and 0<=loc.y<MARSPLANETMAP.height
  if b1 or b2:
    return gc.karbonite_at(loc)
  else:
    return 0
def EDH(x1,y1,x2,y2): # EDH stands for Euclidean Distance Heuristic
  return (int)(((abs(x2-x1)**2)+(abs(y2-y1)**2)))

def astar(unit, dest):
  closedSet = set()
  startingLoc=unit.location.map_location()
  start=(startingLoc.x,startingLoc.y)
  #print("MY VISION", unit.vision_range)
  #print("START NODE:", start[0], start[1])
  unitPlanetWidth = gc.starting_map(startingLoc.planet).width
  unitPlanetHeight = gc.starting_map(startingLoc.planet).height
  cameFrom = {}
  gScore = {} #default value is infinity
  gScore[start]=0
  fScore = {} #default value is infinity
  fScore[start] = EDH(start[0],start[1],dest.x,dest.y)
  openSet = {(startingLoc.x,startingLoc.y): fScore[start]}
  while len(openSet) >0:
    minKeyPair = min(openSet, key=openSet.get)
    minKey = (minKeyPair[0],minKeyPair[1])
    del openSet[minKey]
    #print("CURRENT NODE:", minKey[0], minKey[1])
    if unit.vision_range-3 <= EDH(start[0],start[1],minKey[0],minKey[1]) or (minKey[0]==dest.x and minKey[1]==dest.y):
      reconPath(cameFrom,minKey,start,unit)
      break;
    
    closedSet.add(minKey)

    for x in [[1,0],[-1,0],[0,1],[0,-1],[1,1],[-1,1],[1,-1],[-1,-1]]:
      neighbor = (minKey[0]+x[0],minKey[1]+x[1])
      if EDH(start[0],start[1],neighbor[0],neighbor[1]) > unit.vision_range:
        continue
      if (neighbor[0]<0 or neighbor[0]>=unitPlanetWidth or neighbor[1]<0 or neighbor[1]>=unitPlanetHeight):
        continue
      shouldExit = neighbor in closedSet or not gc.is_occupiable(
        (bc.MapLocation(startingLoc.planet,neighbor[0],neighbor[1])))
      shouldExit = shouldExit or not gc.starting_map(startingLoc.planet).is_passable_terrain_at(  
        (bc.MapLocation(unit.location.map_location().planet,neighbor[0],neighbor[1])))
      
      if shouldExit:
        continue
      

      if neighbor not in openSet:
        openSet[neighbor] = openSet[neighbor] if neighbor in openSet else math.inf

      currentG = gScore[minKey] if minKey in gScore else math.inf
      tentG = (currentG + EDH(minKey[0],minKey[1],neighbor[0],neighbor[1]))
      isDangerLoc = dmap.get((bc.MapLocation(startingLoc.planet,neighbor[0],neighbor[1])))==0
      if isDangerLoc: tentG = (int)(tentG/2)
      gScore[neighbor] = gScore[neighbor] if neighbor in gScore else math.inf
      if tentG >= gScore[neighbor]:
        continue

      cameFrom[neighbor] = minKey
      gScore[neighbor] = tentG
      fScore[neighbor] = gScore[neighbor] + EDH(neighbor[0],neighbor[1],dest.x,dest.y)
  return;

def reconPath(cameFrom,minKey,start,unit):
  #print(cameFrom)
  print("Start", start)
  #print(minKey)
  if unit.movement_heat() < 10:
    totalPath = [minKey]
    while minKey in cameFrom:
      minKey = cameFrom[minKey]
      totalPath.append(minKey)
      #print(totalPath)
    dy = totalPath[-2][1]-totalPath[-1][1]
    dx = totalPath[-2][0]-totalPath[-1][0]
    #print(dx, dy)
    if dy == 1:
      if dx == 0: gc.move_robot(unit.id,bc.Direction.North); return
      elif dx ==1: gc.move_robot(unit.id,bc.Direction.Northeast); return
      else: gc.move_robot(unit.id,bc.Direction.Northwest); return
    elif dy == 0:
      if dx == 1: gc.move_robot(unit.id,bc.Direction.East); return
      else: gc.move_robot(unit.id,bc.Direction.West) ; return
    else:
      if dx == 0: gc.move_robot(unit.id,bc.Direction.South); return
      elif dx ==1: gc.move_robot(unit.id,bc.Direction.Southeast); return
      else: print("MOVEMENT HEAT", unit.movement_heat()<10);gc.move_robot(unit.id,bc.Direction.Southwest); return
  return
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

def fuzzygoto(unit,dest):
  if unit.location.map_location()==dest or gc.is_move_ready(unit.id) == False or unit.movement_heat() >= maxRobotMovementHeat:
    return
  toward = unit.location.map_location().direction_to(dest)
  for tilt in tryRotate:
    d = rotate(toward,tilt)
    newLoc = unit.location.map_location().add(d)
    if dmap.get(newLoc)==0:
      if gc.can_move(unit.id, d):
        gc.move_robot(unit.id,d)
        break
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
def bestKarboniteDirection(loc):
  mostK = 0
  bestDir = None
  for dir in allDirections:
    newK = checkK(loc.add(dir))
    if newK>mostK:
      mostK=newK
      bestDir=dir
  return mostK, bestDir 


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
gc.queue_research(bc.UnitType.Rocket) #necessary
gc.queue_research(bc.UnitType.Worker)
gc.queue_research(bc.UnitType.Knight)

my_team = gc.team()
STAYERS = 4 #number of robots who stay on earth
touchedMars = False #controls whether our karbonite-harvesting group (khg) has reached mars yet.
KHGWORKERS = 2
KHGKNIGHTS = 1
KHGRANGERS = 3
KHGMAGES = 1
KHGHEALERS = 1

earthBlueprintLocations = list()
baseLocations = list()

KHGARRAY = [KHGWORKERS, KHGKNIGHTS, KHGRANGERS, KHGMAGES, KHGHEALERS]
INITIALKHGARRAY = [KHGWORKERS + STAYERS, KHGKNIGHTS + STAYERS, KHGRANGERS + STAYERS, KHGMAGES + (STAYERS*3), KHGHEALERS + STAYERS]
factoryIndex = 0 #controls what the different factories do

earthWorkers = 0
earthKnights = 0
earthRangers = 0
earthMages = 0
earthHealers = 0
vrgn=True
whereTo = dict() #key is type of robot number [0 to 4], planet.  Value is maplocation, errorRadius, number
first_rocket = False
firstRocketBuilt = False
firstRocketLaunched=False
blueprintLocation = None
blueprintWaiting = False



def getRobotProportions(round):
  return KHGARRAY #will change the proportions so that it is a fnction of round

def factoryProtocol(unit, first_rocket, earthBlueprintLocations, firstRocketLaunched):
  if unit.unit_type == bc.UnitType.Factory:
    if not unit.structure_is_built() or unit.health < .75*unit.max_health:
      ml = unit.location.map_location()
      earthBlueprintLocations.append(ml)
      whereTo[workerNum, str(gc.planet())] = ml, 1, 2


    garrison = unit.structure_garrison()

    for item in garrison:
      d = random.choice(directions)  # good for now, change later
      if gc.can_unload(unit.id, d):
        #print ("unloaded")
        gc.unload(unit.id, d)
      else:
        for tilt in tryRotate:
          newD = rotate(d, tilt)
          if gc.can_unload(unit.id, d):
            #print ("unloaded")
            gc.unload(unit.id, d)
          break

    if unit.structure_is_built() == False:
      return


    if firstRocketLaunched == True:
        currentRobotArray = [0, 0, 0, 0, 0]

        for unit in gc.my_units():
            if unit.unit_type == bc.UnitType.Worker:
              currentRobotArray[0] += 1
            elif unit.unit_type == bc.UnitType.Knight:
                currentRobotArray[1] += 1
            elif unit.unit_type == bc.UnitType.Ranger:
                currentRobotArray[2] += 1 
            elif unit.unit_type == bc.UnitType.Mage:
                currentRobotArray[3] += 1
            elif unit.unit_type == bc.UnitType.Healer:
                currentRobotArray[4] += 1

        deficit = [INITIALKHGARRAY[0] - currentRobotArray[0],
             INITIALKHGARRAY[1] - currentRobotArray[1],
             INITIALKHGARRAY[2] - currentRobotArray[2],
             INITIALKHGARRAY[3] - currentRobotArray[3],
             INITIALKHGARRAY[4] - currentRobotArray[4]]

        index = deficit.index(max(deficit))
        robotType = robots[index]
        if gc.can_produce_robot(unit.id, robotType):
            gc.produce_robot(unit.id, robotType)
            print('producing a robot!')

    else: #firstRocketLaunched = true
      robotProportions = getRobotProportions(round)
      # build general robots here
      if gc.can_produce_robot(unit.id, bc.UnitType.Ranger):#produce Rangers
        gc.produce_robot(unit.id, bc.UnitType.Ranger)
        

def rocketProtocol(unit, earthBlueprintLocations):

  global firstRocketLaunched
  global maxRocketGarrison
  global first_rocket
  if unit.unit_type == bc.UnitType.Rocket and unit.location.is_on_map():
    global vrgn #so I can access it whenever
    if unit.location.is_in_space():
      return
    if not unit.structure_is_built() or unit.health < .75*unit.max_health:
      ml = unit.location.map_location()
      earthBlueprintLocations.append(ml)
      blueprintWaiting = True
      whereTo[workerNum, str(gc.planet())] = ml, 1, 2

    if unit.location.is_in_space() or unit.location.is_in_garrison():
      print ("tf")

    if unit.location.is_on_planet(bc.Planet.Earth):
      if not unit.location.is_in_space() and not unit.location.is_in_garrison():
        unit.location.map_location()
        nearby = gc.sense_nearby_units(unit.location.map_location(), 2)
        #print("nearby units to the rocket", nearby)
        for other in nearby:
          if gc.can_load(unit.id,other.id):
            gc.load(unit.id,other.id)
            print('loaded into the rocket!')

        garrison = unit.structure_garrison()
        countNeeded = 5
        if vrgn == False:
          countNeeded = 5
        if len(garrison) >= countNeeded and len(garrison) <= maxRocketGarrison:
          tempPlanetMap = marsMap
          tempLoc = bc.MapLocation(bc.Planet.Mars, (int)(tempPlanetMap.width / 4), (int)(tempPlanetMap.height / 4)) #convert this to a weighted average b4hand
          if gc.can_launch_rocket(unit.id, tempLoc):
            gc.launch_rocket(unit.id, tempLoc)
            vrgn = False
            firstRocketLaunched = True
            print ("Rocket Launched!!!")
            touchedMars = False
            first_rocket = False
          else:
            print ("Rocket failed to launch")

    elif unit.location.is_on_planet(bc.Planet.Mars):
      garrison = unit.structure_garrison();
      print(garrison)
      print("LANDED AND UNLOADING")
      touchedMars = True
      unloadedUnits = 0; prevUnloadedUnits=-1
      while len(garrison) > 0 and unloadedUnits != prevUnloadedUnits:
        prevUnloadedUnits=unloadedUnits
        for d in directions: # good for now, change later
          if gc.can_unload(unit.id, d):
            print ("rocket unloaded")
            unloadedUnits+=1
            gc.unload(unit.id, d)
            continue

def coefficient():
  return 2

def workerProtocol(unit, earthBlueprintLocations, numWorkers):

  global maxRocketHealth
  global maxFactoryHealth
  global maxRobotMovementHeat
  global first_rocket
  global firstRocketLaunched
  if unit.unit_type == bc.UnitType.Worker:

    harvestKarbonite(unit) #cuz workers need to harvest karbonite before they do anything else
    replicated = False
    d = random.choice(directions)
    if numWorkers<10:
      replicated=False
      for d in directions:
        if gc.can_replicate(unit.id,d):
          gc.replicate(unit.id,d)
          replicated=True
          break
    else:
      #blueprint rocket
      if not first_rocket and unit.location.is_on_planet(bc.Planet.Earth):
        for q in directions:
          if not first_rocket and gc.karbonite() > bc.UnitType.Rocket.blueprint_cost() and gc.can_blueprint(unit.id,bc.UnitType.Rocket,q):
            gc.blueprint(unit.id,bc.UnitType.Rocket,q)
            print("ROCKET BLUEPRINTED YAH")
            earthBlueprintLocations.append(unit.location.map_location().add(q))
            first_rocket = True
            print ("set first rocket to True")
            break

      adjacentUnits = gc.sense_nearby_units(unit.location.map_location(), 1) 
      for adjacent in adjacentUnits:#once you build, you need to take it out of earthBlueprintLocations
        adjacentLocation = adjacent.location.map_location()
        if gc.can_build(unit.id,adjacent.id) and adjacent.health != adjacent.max_health:
          gc.build(unit.id,adjacent.id)
          if adjacent.unit_type == bc.UnitType.Rocket:
            print("ROCKET BEING BUILT!")
            if adjacent.health == adjacent.max_health and adjacentLocation in earthBlueprintLocations:
              earthBlueprintLocations.remove(adjacentLocation)

          elif adjacent.unit_type == bc.UnitType.Factory:
            #print ("FACTORY BEING BUILT!")
            if adjacent.health == adjacent.max_health and adjacentLocation in earthBlueprintLocations:
              earthBlueprintLocations.remove(adjacentLocation)

        elif gc.can_repair(unit.id,adjacent.id) and adjacent.health != adjacent.max_health:
          gc.repair(unit.id,adjacent.id)
          if adjacent.unit_type == bc.UnitType.Rocket:
            print("ROCKET BEING REPAIRED!")
            if adjacent.health == adjacent.max_health and adjacentLocation in earthBlueprintLocations:
              earthBlueprintLocations.remove(adjacentLocation)

          elif adjacent.unit_type == bc.UnitType.Factory:
            #print ("FACTORY BEING REPAIRED!")
            if adjacent.health == adjacent.max_health and adjacentLocation in earthBlueprintLocations:
              earthBlueprintLocations.remove(adjacentLocation)

      #attack enemies
      '''attackableEnemies = gc.sense_nearby_units_by_team(unit.location.map_location(),unit.attack_range(),enemy_team)
      if len(attackableEnemies)>0 and  gc.is_attack_ready(unit.id) and gc.can_attack(unit.id, attackableEnemies[0].id):
        if gc.is_attack_ready(unit.id):
          gc.attack(unit.id, attackableEnemies[0].id)'''

      #blueprint factories
      d = random.choice(directions)
      if gc.karbonite() > coefficient() * bc.UnitType.Factory.blueprint_cost():#blueprint
        if gc.can_blueprint(unit.id, bc.UnitType.Factory, d):
          gc.blueprint(unit.id, bc.UnitType.Factory, d)

      #head toward blueprint locations
      if unit.movement_heat() < maxRobotMovementHeat: 
        for blueprintLocation in earthBlueprintLocations: #this system handles multiple blueprints, going to the first one
          ml = unit.location.map_location()
          bdist = ml.distance_squared_to(blueprintLocation)
          if bdist>2:
            #print ("heading towards blueprint")
            astar(unit, blueprintLocation)
            break
            return #can't do anything else at this point

      if len(kLocs)>0 and unit.movement_heat() < maxRobotMovementHeat: #need to go looking for karbonite
        dest=kLocs[0]
        if gc.can_sense_location(dest):
          kAmt = gc.karbonite_at(dest)
          if kAmt==0:
            kLocs.pop(0)
          else:
            fuzzygoto(unit,dest)


def rangerProtocol(unit, first_rocket, earthBlueprintLocations, firstRocketLaunched):
  if unit.unit_type == bc.UnitType.Ranger:
    #print(unit.unit_type)
    if not unit.location.is_in_garrison():#can't move from inside a factory
      attackableEnemies = gc.sense_nearby_units_by_team(unit.location.map_location(),unit.attack_range(),enemy_team)
      if len(attackableEnemies)>0 and  gc.is_attack_ready(unit.id) and gc.can_attack(unit.id, attackableEnemies[0].id):
        if gc.is_attack_ready(unit.id):
          gc.attack(unit.id, attackableEnemies[0].id)
      elif gc.is_move_ready(unit.id):
        nearbyEnemies = gc.sense_nearby_units_by_team(unit.location.map_location(),unit.vision_range,enemy_team)
        if len(nearbyEnemies)>0:
          destination=nearbyEnemies[0].location.map_location()
        else:
          destination=enemyStart
        if destination is not None:
          astar(unit,destination)

def healerProtocol(unit):
    if unit.unit_type == bc.UnitType.Healer:
      if not unit.location.is_in_garrison(): 
        attackableFriends = gc.sense_nearby_units_by_team(unit.location.map_location(),unit.attack_range(),my_team)
        if len(attackableFriends)>0 and  gc.is_attack_ready(unit.id) and gc.can_attack(unit.id, attackableFriends[0].id):
          if gc.is_attack_ready(unit.id):
            gc.attack(unit.id, attackableFriends[0].id)
        elif gc.is_move_ready(unit.id):
          nearbyFriends = gc.sense_nearby_units_by_team(unit.location.map_location(),unit.vision_range,my_team)
          destination=nearbyFriends[0].location.map_location()
          if destination is not None:
            astar(unit,destination)

def clearRoom(unit):
  if unit.location.is_in_garrison() or unit.location.is_in_space():
    return
  currentLocation = unit.location.map_location()
  adjacentUnits = gc.sense_nearby_units(currentLocation, 1) #apparently this includes unit itself
  for adjacent in adjacentUnits:#sensing if there is a factory or rocket nearby
    adjLoc = adjacent.location.map_location()
    if adjacent.unit_type == bc.UnitType.Rocket and currentLocation.is_adjacent_to(adjLoc): #gtfo, you don't want to be near a rocket
      towardRocket = currentLocation.direction_to(adjLoc)
      awayFromRocket = rotate(towardRocket, 4) #4 means 180 degrees turn
      astar(unit, currentLocation.add(awayFromRocket)) #stay fuzzygoto (don't do astar)

  myTeamAdjacentUnits = gc.sense_nearby_units_by_team(currentLocation, 1, my_team) #apparently this includes unit itself
  for madjacent in myTeamAdjacentUnits:
    adjLoc = adjacent.location.map_location()
    if madjacent.unit_type == bc.UnitType.Factory and currentLocation.is_adjacent_to(adjLoc): #if it's one of my own factories, I don't want to be blocking it.
      if closedIn(madjacent) == True:
        towardFactory = currentLocation.direction_to(adjLoc)
        awayFromFactory = rotate(towardFactory, 4) #4 means 180 degrees turn
        astar(unit, currentLocation.add(awayFromFactory)) #stay fuzzygoto (don't do astar)


if gc.planet() == bc.Planet.Earth:
  w=earthMap.width
  h=earthMap.height
else:
  marsMap=gc.starting_map(bc.Planet.Mars)
  w=marsMap.width
  h=marsMap.height

while True:
    # We only support Python 3, which means brackets around print()
    round = gc.round()
    if round == 126:
      workerHarvestAmount += 1
    # frequent try/catches are a good idea
    try:
          
          dmap = mmap(w,h)
          for unit in gc.units():
            if not unit.location.is_in_garrison():
              if unit.team!=my_team:
                if unit.unit_type == bc.UnitType.Mage or unit.unit_type ==bc.UnitType.Ranger or unit.unit_type ==bc.UnitType.Knight:
                  dmap.addDisk(unit.location.map_location(),unit.attack_range(),1)
          
          numWorkers = 0

          for unit in gc.my_units():
            if unit.unit_type== bc.UnitType.Worker:
              numWorkers+=1

          for unit in gc.my_units():

            if unit.location.is_in_space():
              continue

            factoryProtocol(unit, first_rocket, earthBlueprintLocations, firstRocketLaunched)

            rocketProtocol(unit, first_rocket, earthBlueprintLocations)

            location = unit.location
            if location.is_on_map() == True and location.is_in_garrison() == False:

              clearRoom(unit)

              workerProtocol(unit, earthBlueprintLocations, numWorkers)

              rangerProtocol(unit, first_rocket, earthBlueprintLocations, firstRocketLaunched)

              healerProtocol(unit)



                


                
                 #possibly useless piece of code begins
                 #possibly useless piece of code ends
                 
                  
                  
                        #build general robots here
                
                
              

                  #ajith your strat ends here
              # okay, there weren't any dudes around
              # pick a random direction:

       #possibly useless piece of code begins
       
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
