# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from sensed_world import SensedWorld
from events import Event
import os.path
from colorama import Fore, Back
# Imports for code implementation
from queue import PriorityQueue
import random
import math

class TestCharacter(CharacterEntity):

    def do(self, wrld):
        # Your code here

        # Prints the current position of the character after the character moves
        # print(self.x, self.y)

        # Uncomment to learn
        # self.learn(wrld)

        w = self.getW()
        f = self.getF()
        a = self.getActions(wrld)
        Q = []

        bombs = self.getBomb(wrld)
        if len(bombs) > 0:
            if (0, 0) in a:
                a.remove((0, 0))
        for act in a:
            Q.append(self.calculateQ(wrld, w, f, act))

        maxQ = -999999
        maxA = a[0]
        for i in range(len(Q)):
            if Q[i] > maxQ:
                maxQ = Q[i]
                maxA = a[i]
        print(a)
        print(Q)
        print(maxA)
        if (maxA == (0,0)):
            self.place_bomb()
        else:
            self.move(maxA[0], maxA[1])

        pass

    # Gets the locations of the enemies location
    #
    # PARAM: [world] wrld: the current state of the world
    #
    # RETURNS: [list [entities]] enemies: a list of enemies
    #
    def getEnemy(self, wrld):
        enemies = []

        for y in range(wrld.height()):
            for x in range(wrld.width()):
                if(wrld.monsters_at(x, y)):
                    enemies.append((x, y))

        return enemies

    # Gets the locations of the bomb location
    #
    # PARAM: [world] wrld: the current state of the world
    #
    # RETURNS: [list [entities]] bombs: a list of bombs
    #
    def getBomb(self, wrld):
        bombs = []

        for y in range(wrld.height()):
            for x in range(wrld.width()):
                if (wrld.bomb_at(x, y)):
                    bombs.append((x, y))

        return bombs

    # Gets the locations of the explosion location
    #
    # PARAM: [world] wrld: the current state of the world
    #
    # RETURNS: [list [entities]] explosions: a list of explosions
    #
    def getExplosion(self, wrld):
        explosions = []

        for y in range(wrld.height()):
            for x in range(wrld.width()):
                if (wrld.explosion_at(x, y)):
                    explosions.append((x, y))

        return explosions

    # Gets the locations of the future explosions
    #
    # PARAM: [world] wrld: the current state of the world
    #
    # RETURNS: [list [entities]]: a list of future explosion spots
    #
    def getFutureExplosions(self, wrld):
        bombs = []

        for y in range(wrld.height()):
            for x in range(wrld.width()):
                if (wrld.bomb_at(x, y)):
                    temp = [(x, y), (x+1, y), (x+2, y), (x+3, y), (x+4, y), (x-1, y), (x-2, y), (x-3, y), (x-4, y),
                            (x, y+1), (x, y+2), (x, y+3), (x, y+4), (x, y-1), (x, y-2), (x, y-3), (x, y-4)]
                    bombs.extend(temp)

        return bombs


    # Finds the goal / exit to the world
    #
    # PARAM: [world] wrld: the current world configuration
    # RETURNS: [int, int] x, y: the x and y coordinates of the goal / exit
    #
    def findGoal(self, wrld):
        x = 0
        y = 0
        for y in range(wrld.height()):
            for x in range(wrld.width()):
                if(wrld.exit_at(x,y)):
                    return (x,y)

        # Return impossible exit coordinate to signal no exit found
        print('No Exit Found')
        return (-1,-1)

    # Returns a world with the A* path marked
    #
    # PARAM: [world, [int, int], [int, int]] wrld: the current world configuration
    #                                       [start.x, start.y]: the x and y coordinated the agent is located at
    #                                       [goal.x, goal.y]: the x and y coordinated of the goal / exit
    # RETURNS: [world] newWrld: a grid world marking the A* path to the goal
    #
    def a_star_search(self, wrld, start, goal):

        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            for next in self.getAllMoves(wrld, current):
                if (wrld.wall_at(next[0], next[1])):
                    new_cost = cost_so_far[current] + 20
                else: # if there is a wall there
                    new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.a_star_heuristic2(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

        return came_from, cost_so_far


    # Returns a heuristic value to influence agent movement
    #
    # PARAM: [goal.x, goal.y] goal: the x and y location of the goal
    #        [start.x, start.y] current: the x and y coordinated the agent is located at
    # RETURNS: [int] distance: manhattan distance between the current agent location and the goal
    #
    def a_star_heuristic(self, goal, next):
        return abs(goal[0] - next[0]) + abs(goal[1] - next[1])

    # Returns a heuristic value to influence agent movement
    #
    # PARAM: [goal.x, goal.y] goal: the x and y location of the goal
    #        [start.x, start.y] current: the x and y coordinated the agent is located at
    # RETURNS: [int] distance: distance between the current agent location and the goal
    #
    def a_star_heuristic2(self, goal, next):
        return math.sqrt(((goal[0] - next[0]) ** 2) + ((goal[1] - next[1]) ** 2))

    # Returns list of all the possible moves for agent (up, down, left, right, diagonal) including walls
    #
    # PARAM: [world, [int, int]] wrld: the current world configuration
    #        [start.x, start.y] currentLocation: the x and y coordinated the agent is located at
    # RETURNS: [list [int, int]] allMoves: a list of all the possible moves for agent
    #
    def getAllMoves(self, wrld, currentLocation):
        movesList = []

        # Look right
        # Avoid out of bound look ups
        if ((currentLocation[0] + 1) < wrld.width()):
            movesList.append((currentLocation[0] + 1, currentLocation[1]))
        # Look left
        # Avoid out of bound look ups
        if ((currentLocation[0] - 1) >= 0):
            movesList.append((currentLocation[0] - 1, currentLocation[1]))
        # Look down
        # Avoid out of bound look ups
        if ((currentLocation[1] - 1) >= 0):
            movesList.append((currentLocation[0], currentLocation[1] - 1))
        # Look up
        # Avoid out of bound look ups
        if ((currentLocation[1] + 1) < wrld.height()):
            movesList.append((currentLocation[0], currentLocation[1] + 1))
        # Look diagonal right, up
        # Avoid out of bound look ups
        if ((currentLocation[0] + 1) < wrld.width() and (currentLocation[1] + 1) < wrld.height()):
            movesList.append((currentLocation[0] + 1, currentLocation[1] + 1))
        # Look diagonal right, down
        # Avoid out of bound look ups
        if ((currentLocation[0] + 1) < wrld.width() and (currentLocation[1] - 1) >= 0):
            movesList.append((currentLocation[0] + 1, currentLocation[1] - 1))
        # Look diagonal left, up
        # Avoid out of bound look ups
        if ((currentLocation[0] - 1) >= 0 and (currentLocation[1] + 1) < wrld.height()):
            movesList.append((currentLocation[0] - 1, currentLocation[1] + 1))
        # Look diagonal left, down
        # Avoid out of bound look ups
        if ((currentLocation[0] - 1) >= 0 and (currentLocation[1] - 1) >= 0):
            movesList.append((currentLocation[0] - 1, currentLocation[1] - 1))

        return movesList


    # Finds the A* path to the goal
    #
    # PARAM: [start.x, start.y] currentLocation: the x and y coordinated the agent is located at
    #        [world] wrld: the current world configuration
    #        [world] a_star_grpah: a graph representing the a_star path fro the agent
    # RETURNS: [list [int, int]] path: a list of the path to the goal
    #
    def findPath(self, start, goal, wrld, a_star_graph):
        move = goal
        while (move != start):
            lastMove = move
            move = a_star_graph[0][lastMove]

        return lastMove


    # Returns the move recommended by a*
    #
    # PARAM: [world, [int, int], [int, int]] wrld: the current world configuration
    #                                       [start.x, start.y]: the x and y coordinated the agent is located at
    #                                       [goal.x, goal.y]: the x and y coordinated of the goal / exit
    #
    # RETURNS: [int, int]: a_star_move: the move recommended by a*
    #
    def get_a_star_move(self, wrld, start, goal):
        a_star_path = self.a_star_search(wrld, start, goal)

        nextMove = self.findPath(start, goal, wrld, a_star_path)
        a_star_move = (nextMove[0] - self.x, nextMove[1] - self.y)
        return a_star_move

    def learn(self, wrld):
        w = self.getW()
        f = self.getF()

        if len(w) != len(f):
            sys.exit(-1)

        # DISCOUNT FACTOR
        gamma = 0.9

        # INITIAL LEARNING RATE
        # will decrease by 0.1 every 2500 iterations
        alpha = 0.5
        alphaDecFreq = 1250
        alphaDecRate = 0.1


        # EXPLORATION VS EXPLOITATION FACTOR
        # will decrease by 0.1 every 1500
        epsilon = 0.8
        epsilonDecFreq = 750
        epsilonDecRate = 0.1

        # Learning will start from this world
        s = SensedWorld.from_world(wrld)

        done = False
        for i in range(5000):
            if i == 100 or i == 200 or i == 300 or i == 400 or i == 500 or i % 1000 == 0:
                print(i)
            if i % epsilonDecFreq == 0:
                epsilon -= epsilonDecRate
            if i % alphaDecFreq == 0:
                alpha -= alphaDecRate

            if done:
                s = SensedWorld.from_world(wrld)
                done = False

            me = s.me(self)

            # get actions and calculate Q(s, a)
            a = self.getActions(s)
            Q = []
            for i in range(len(a)):
                Q.append(0)
                Q[i] = self.calculateQ(s, w, f, a[i])

            # choose move
            temp = random.random()
            if temp < epsilon:
                act = random.choice(a)
                choice = a.index(act)
            else:
                maxQ = -99999
                act = a[0]
                choice = 0
                for i in range(len(a)):
                    if Q[i] > maxQ:
                        maxQ = Q[i]
                        act = a[i]
                        choice = i

            # do the moves, progress the world, and get reward
            for m2 in s.monsters.values():
                for m in m2:
                    m.do(s)
            if act == (0, 0):
                me.place_bomb()
            else:
                me.move(act[0], act[1])
            newS, events = s.next()
            r = self.getReward(newS, events)

            # if dead or finished, Q(s', a') == 0
            if r == 1000 or r < 0:
                nextQ = 0
                done = True
            else:
                na = self.getActions(newS)
                nQ = []
                for i in range(len(na)):
                    nQ.append(0)
                    nQ[i] = self.calculateQ(newS, w, f, na[i])
                nextQ = max(nQ)

            delta = (r + gamma*nextQ) - Q[choice]
            for i in range(len(w)):
                w[i] = w[i] + alpha * delta * f[i](s, act)

            s = newS

        with open('../S1V2weights.txt', 'w+') as out:
            for weight in w:
                out.write(str(weight))
                out.write("\n")
        sys.exit(1)

    # calculates (Max?) Q value
    def calculateQ(self, s, w, f, a):
        qVal = 0
        for j in range(len(w)):
            qVal += w[j] * f[j](s, a)
        return qVal

    # returns the reward for a state, and the events that happened
    def getReward(self, s, events):
        score =  0
        for e in events:
            if e.tpe == Event.CHARACTER_FOUND_EXIT:
                return 1000
            elif e.tpe == Event.CHARACTER_KILLED_BY_MONSTER or e.tpe == Event.BOMB_HIT_CHARACTER:
                return -1000
            elif e.tpe == Event.BOMB_HIT_MONSTER:
                score += 100
            elif e.tpe == Event.BOMB_HIT_WALL:
                score += 10
        me = s.me(self)
        temp = (s.width() + s.height()) - self.a_star_heuristic(self.findGoal(s), (me.x, me.y))
        score += temp*2

        return score

    # returns a list of all legal actions
    def getActions(self, s):
        me = s.me(self)
        a = []
        for x in range(-1, 2):
            # print(x)
            for y in range(-1, 2):
                if me.x+x >= 0 and me.x+x < s.width() and me.y+y >= 0 and me.y+y < s.height() and not s.wall_at(me.x+x, me.y+y):
                    a.append((x, y))
        return a

    # returns number of monsters
    def numMonsters(self, s, a):
        me = s.me(self)
        if a == (0, 0):
            me.place_bomb()
        else:
            me.move(a[0], a[1])
        newS, events = s.next()
        monsters = self.getEnemy(newS)
        return len(monsters)

    # Returns 1/(1 + distance to closest monster)
    # if there are no monsters, just makes the distance monstrous
    def distToCloseMonster(self, s, a):
        me = s.me(self)
        monsters = self.getEnemy(s)
        closestDist = 9999999
        for m in monsters:
            dist = self.a_star_heuristic2((me.x + a[0], me.y + a[1]), m)
            if dist <= closestDist:
                closestDist = dist
        return float(1) / (1 + closestDist)

    # returns 1 / (1 + dist to goal)
    def distToGoal(self, s, a):
        me = s.me(self)
        goal = self.findGoal(s)
        dist = self.a_star_heuristic2((me.x + a[0], me.y + a[1]), goal)
        return float(1) / (1 + dist)

    # returns 1 for True, 0 for False
    def charIsOnFutureExplosion(self, s, a):
        me = s.me(self)
        futExps = self.getFutureExplosions(s)
        for fe in futExps:
            if fe[0] == me.x+a[0] and fe[1] == me.y+a[1]:
                return 1
        return 0

    # Returns 1/(1 + distance to closest explosion)
    # if there are no explosions, just makes the distance monstrous
    def distToExplosion(self, s, a):
        me = s.me(self)
        explosions = self.getExplosion(s)
        closestDist = 9999999
        for e in explosions:
            dist = self.a_star_heuristic2((me.x + a[0], me.y + a[1]), e)
            if dist <= closestDist:
                closestDist = dist
        return float(1) / (1 + closestDist)

    # Returns 1/(1 + distance to bomb)
    # if there are no bombs, just makes the distance monstrous
    def distToBomb(self, s, a):
        me = s.me(self)
        bomb = self.getBomb(s)
        closestDist = 9999999
        for b in bomb:
            dist = self.a_star_heuristic2((me.x + a[0], me.y + a[1]), b)
            if dist <= closestDist:
                closestDist = dist
        return float(1) / (1 + closestDist)


    def getW(self):
        if not os.path.isfile('../S1V2weights.txt'):
            return [-776.8290575861843, 128.5085107820687, -63.732647008532176, -173.71497364470977,  -53.995981462616804]
        w = []
        with open('../S1V2weights.txt', 'r') as fd:
            for line in fd:
                w.append(float(line))

        f = self.getF()
        if len(w) != len(f):
            print("Using default weights")
            return [-776.8290575861843, 128.5085107820687, -63.732647008532176, -173.71497364470977, -53.995981462616804]
        else:
            return w

    def getF(self):
        return [self.distToCloseMonster, self.distToGoal,
             self.charIsOnFutureExplosion, self.distToExplosion, self.distToBomb]
