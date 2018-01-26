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
gc = bc.GameController()
robots = [bc.UnitType.Mage, bc.UnitType.Knight, bc.UnitType.Ranger, bc.UnitType.Mage, bc.UnitType.Ranger]
earthMap = gc.starting_map(bc.Planet.Earth)
marsMap = None
start = None
enemyStart = None
roundsBack = 10
maxRobotMovementHeat = 10
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
def factory_move(robot_id): #move robot away from factory -- this method takes the robot id
  for i in direction:
    if gc.can_move(robot_id,i):
      gc.move_robot(robot_id,d)
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

def harvestKarbonite(unit, currentLocation):
  amount = gc.karbonite_at(currentLocation)
  max = 0
  maxDir = None
  if amount > workerHarvestAmount:
    if gc.can_harvest(unit.id, Direction.Center):
      gc.harvest(unit.id, Direction.Center)
  for direction in directions:
    newLoc = currentLocation.add(direction)
    if earthMap.on_map(newLoc):
      cur = gc.karbonite_at(newLoc)
      if cur > max:
        max = cur
        maxDir = direction

  if max > 0:
    gc.harvest(unit.id, maxDir)

def onEarth(loc):
  if (loc.x<0) or (loc.y<0) or (loc.x>=earthMap.width) or (loc.y>=earthMap.height): return False
  return True

def checkK(loc):
  if not onEarth(loc): return 0
  return gc.karbonite_at(loc)

def astar(unit, dest):
    if not unit.movement_heat() < maxRobotMovementHeat:
        return
    currentLocation = unit.location.map_location()
    if currentLocation.is_adjacent_to(prev) == True:
      return
    if unit.unit_type == robots[0] and gc.karbonite() < optimalKarboniteAmount: #robots[0] is worker
      #print (gc.karbonite())
      harvestKarbonite(unit, currentLocation)
    if currentLocation.direction_to(dest) == bc.Direction.Center:
      pathDict.pop(unit.id, str(dest))
      return
    if (unit.id, str(dest)) in pathDict: #the program has saved where this thing has been trying to go
        path = pathDict[unit.id, str(dest)]
        prev = path[0].mapLocation
        if currentLocation.is_adjacent_to(prev) == False: #had used bugnav recently and not completely finished
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
            #path.popleft()
        else: #at this point, there is clearly an obstable, such as a factory in the way.  Calling bugnav
            newDest = path[0].mapLocation
            go_to(unit, newDest)
    else: #the first time this program is trying to make the unit get to this destination
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
                path.reverse() #because it's in reverse order
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
  if unit.location.map_location()==dest:return
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
KHGKNIGHTS = 2
KHGRANGERS = 1
KHGMAGES = 1
KHGHEALERS = 2

earthRocketLocations = list()
baseLocations = list()

KHGARRAY = [KHGWORKERS, KHGKNIGHTS, KHGRANGERS, KHGMAGES, KHGHEALERS]
INITIALKHGARRAY = [KHGWORKERS + STAYERS, KHGKNIGHTS + STAYERS, KHGRANGERS + STAYERS, KHGMAGES + (STAYERS*0), KHGHEALERS + STAYERS]
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
firstRocketLanded = False

def getRobotProportions(round):
  return KHGARRAY #will change the proportions so that it is a fnction of round

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
    if gc.planet() == bc.Planet.Earth:
        #firstMan = gc.my_units()[0]
        '''  print('pyround:', r ound, 'time left:', gc.get_time_left_ms(), 'ms',
        'heat:', firstMan.movement_heat(), 'loc: ', firstMan.location.map_location())'''
        print ('pyround:', round, 'karbonite', gc.karbonite())
        randomLocation = bc.MapLocation(bc.Planet.Earth, 0, 0)

        #astar(firstMan, randomLocation)
    # frequent try/catches are a good idea
    try:
          
          dmap = mmap(w,h)
          numWorkers = 0
          blueprintLocation = None
          blueprintWaiting = False
          for unit in gc.units():
            if unit.unit_type== bc.UnitType.Factory:
              if not unit.structure_is_built():
                ml = unit.location.map_location()
                blueprintLocation = ml
                blueprintWaiting = True
            if unit.unit_type== bc.UnitType.Worker:
              numWorkers+=1
            if not unit.location.is_in_garrison():
              if unit.team!=my_team:
                dmap.addDisk(unit.location.map_location(),50,1)
          
          for unit in gc.my_units():
            
            location = unit.location
            if location.is_on_map():
                
                if unit.unit_type == bc.UnitType.Rocket:
                  #print("ROCKET")
                  if unit.location.is_on_planet(bc.Planet.Earth):
                    nearby = gc.sense_nearby_units(location.map_location(), 2)
                    #print("nearby units to the rocket", nearby)
                    for other in nearby:
                      if gc.can_load(unit.id,other.id):
                        gc.load(unit.id,other.id)
                        print('loaded into the rocket!')
                    garrison == unit.structure_garrison()
                    countNeeded = 1
                    if vrgn == False:
                      countNeeded = 1
                    if len(garrison) >= countNeeded and len(garrison) <= 8:
                      tempPlanetMap = gc.starting_map(bc.Planet.Mars)
                      tempLoc = bc.MapLocation(bc.Planet.Mars, (int)(
                          tempPlanetMap.width / 4), (int)(tempPlanetMap.height / 4))
                      if gc.can_launch_rocket(unit.id, tempLoc):
                        print("ROCKET LAUNCHED!!!!!!!!")
                        gc.launch_rocket(unit.id, tempLoc)
                        vrgn = False
                  else:
                      garrison = unit.structure_garrison();
                      if not firstRocketLanded:
                        print(garrison)
                        print("LANDED!")
                        firstRocketLanded=True
                      unloadedUnits = 0; prevUnloadedUnits=-1
                      while len(garrison) > 0 and unloadedUnits != prevUnloadedUnits:
                        prevUnloadedUnits=unloadedUnits
                        for d in directions: # good for now, change later
                          if gc.can_unload(unit.id, d):
                            print ("unloaded")
                            unloadedUnits+=1
                            gc.unload(unit.id, d)
                            continue
                      
                if unit.unit_type == bc.UnitType.Worker:
                  #print("work")
                  if not first_rocket and unit.location.is_on_planet(bc.Planet.Earth):
                    for q in directions:
                      if not first_rocket and gc.karbonite() > bc.UnitType.Rocket.blueprint_cost() and gc.can_blueprint(unit.id,bc.UnitType.Rocket,q):
                        gc.blueprint(unit.id,bc.UnitType.Rocket,q)
                        print("ROCKET BLUEPRINTED YAH")
                        rocketLocation = gc.unit(unit.id).location.map_location().add(q)
                        #whereTo[0, gc.planet()] = rocketLocation, 1, 1
                        first_rocket = True
                        break
                if unit.unit_type == bc.UnitType.Worker:
                  #print(unit.unit_type)
                  d = random.choice(directions)
                  if numWorkers<10:
                    if gc.can_replicate(unit.id,d) and numWorkers < 10:
                      gc.replicate(unit.id,d);
                      continue
                  if gc.karbonite() > bc.UnitType.Factory.blueprint_cost():#blueprint
                    if gc.can_blueprint(unit.id, bc.UnitType.Factory, d):
                      gc.blueprint(unit.id, bc.UnitType.Factory, d)

                      continue
                  adjacentUnits = gc.sense_nearby_units(unit.location.map_location(), 1)
                  for adjacent in adjacentUnits:#build
                    if gc.can_build(unit.id,adjacent.id):
                      gc.build(unit.id,adjacent.id)
                      if adjacent.id == bc.UnitType.Rocket:
                        print("ROCKET BUILT!")
                      continue
                  #head toward blueprint location
                  if gc.is_move_ready(unit.id):
                    if blueprintWaiting:
                      ml = unit.location.map_location()
                      bdist = ml.distance_squared_to(blueprintLocation)
                      if bdist>2:
                        fuzzygoto(unit,blueprintLocation)
                        continue
                  #harvest karbonite from nearby
                  mostK, bestDir = bestKarboniteDirection(unit.location.map_location())
                  if mostK>0:#found some karbonite to harvest
                    if gc.can_harvest(unit.id,bestDir):
                      gc.harvest(unit.id,bestDir)
                      continue
                  elif gc.is_move_ready(unit.id):#need to go looking for karbonite
                    if len(kLocs)>0:
                      dest=kLocs[0]
                      if gc.can_sense_location(dest):
                        kAmt = gc.karbonite_at(dest)
                        if kAmt==0:
                          kLocs.pop(0)
                        else:
                          fuzzygoto(unit,dest)
                
                if unit.unit_type == bc.UnitType.Factory:
                  #print(unit.unit_type)
                  garrison = unit.structure_garrison()
                  if len(garrison) > 0:#ungarrison
                    d = random.choice(directions)
                    if gc.can_unload(unit.id, d):
                      gc.unload(unit.id, d)
                      continue
                  elif gc.can_produce_robot(unit.id, bc.UnitType.Mage):#produce Mages
                    gc.produce_robot(unit.id, bc.UnitType.Mage)
                    continue
                
                if unit.unit_type == bc.UnitType.Mage:
                  #print(unit.unit_type)
                  if not unit.location.is_in_garrison():#can't move from inside a factory
                    attackableEnemies = gc.sense_nearby_units_by_team(unit.location.map_location(),unit.attack_range(),enemy_team)
                    shouldNOTAttackSomething = False
                    if len(attackableEnemies)>0 and  gc.is_attack_ready(unit.id) and gc.can_attack(unit.id, attackableEnemies[0].id):
                      if gc.is_attack_ready(unit.id):
                        gc.attack(unit.id, attackableEnemies[0].id)
                    elif gc.is_move_ready(unit.id):
                      nearbyEnemies = gc.sense_nearby_units_by_team(unit.location.map_location(),unit.vision_range,enemy_team)
                      if len(nearbyEnemies)>0:
                        destination=nearbyEnemies[0].location.map_location()
                      else:
                        if unit.location.is_on_planet(bc.Planet.Earth):
                          destination=enemyStart
                        else: shouldNOTAttackSomething=True

                      if not shouldNOTAttackSomething: fuzzygoto(unit,destination)
                      

                
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
