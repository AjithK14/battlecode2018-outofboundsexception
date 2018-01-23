import battlecode as bc
import random
import sys
from collections import deque
import traceback
import time
from heapq import heappush, heappop

import os

def factory_move(robot): #move robot away from factory -- this method takes the robot id
  for i in direction:
    if gc.can_move(robot,i):
      gc.move_robot(robot,d)
      return 

directions = [bc.Direction.North, bc.Direction.Northeast, bc.Direction.East,
              bc.Direction.Southeast, bc.Direction.South, bc.Direction.Southwest,
              bc.Direction.West, bc.Direction.Northwest]

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

print(os.getcwd())
gc.queue_research(bc.UnitType.Rocket)
gc.queue_research(bc.UnitType.Worker)
gc.queue_research(bc.UnitType.Mage)

my_team = gc.team()
first_rocket = False
firstRocketBuilt=False
roundsToDurations = {x: gc.orbit_pattern().duration(x) for x in range(1,200)}
optimalRoundForRocket = min(roundsToDurations, key=roundsToDurations.get)
print("OPTIM", optimalRoundForRocket)
while True:
    # We only support Python 3, which means brackets around print()
    round = gc.round()
    print("MYKARBONITE", gc.karbonite())
    try:
        # walk through our units:
        #print("units", gc.my_units())
        for unit in gc.my_units():

            # first, factory logic
            
            if not first_rocket:
              for q in directions:
                if gc.karbonite() > bc.UnitType.Rocket.blueprint_cost() and gc.can_blueprint(unit.id,bc.UnitType.Rocket,q):
                  gc.blueprint(unit.id,bc.UnitType.Rocket,q)
                  print("ROCKET BLUEPRINTED YAH")
                  first_rocket = True
            if round == optimalRoundForRocket and unit.unit_type == bc.UnitType.Rocket:
              tempPlanetMap = gc.planet_map(bc.Planet.Mars)
              tempLoc = MapLocation(bc.Planet.Mars,(int)(Mars,tempPlanetMap.width/2),(int)(Mars,tempPlanetMap.height/2))
              if gc.can_launch_rocket(unit.id,tempLoc):
                gc.launch_rocket(unit.id,tempLoc)
            if unit.unit_type == bc.UnitType.Factory:
                garrison = unit.structure_garrison()
                if len(garrison) > 0:
                    d = random.choice(directions)
                    if gc.can_unload(unit.id, d):
                        print('unloaded a knight!')
                        gc.unload(unit.id, d)
                        continue
                elif gc.can_produce_robot(unit.id, bc.UnitType.Mage):
                    gc.produce_robot(unit.id, bc.UnitType.Mage)
                    print('produced a mage!')
                    continue
            location = unit.location
            if location.is_on_map():
                nearby = gc.sense_nearby_units(location.map_location(), 4)
                for other in nearby:
                    if not firstRocketBuilt and unit.unit_type == bc.UnitType.Rocket and gc.can_build(unit.id, other.id):
                        gc.build(unit.id, other.id)
                        print('built a rocket!')
                        firstRocketBuilt = True
                        continue
                    if unit.unit_type == bc.UnitType.Rocket and gc.can_load(other.id,unit.id):
                        gc.load(other.id,unit.id)
                        print('loaded into the rocket!')
                    if unit.unit_type == bc.UnitType.Worker and gc.can_build(unit.id, other.id):
                        gc.build(unit.id, other.id)
                        print('built a factory!')
                        continue
                    if other.team != my_team and gc.is_attack_ready(unit.id) and gc.can_attack(unit.id, other.id):
                        print('attacked a thing!')
                        gc.attack(unit.id, other.id)
                        continue
            #if its an actual unit then just move in all directions
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
        traceback.print_exc()
    # send the actions we've performed, and wait for our next turn.
    gc.next_turn()

    # these lines are not strictly necessary, but it helps make the logs make more sense.
    # it forces everything we've written this turn to be written to the manager.
    sys.stdout.flush()
    sys.stderr.flush()
