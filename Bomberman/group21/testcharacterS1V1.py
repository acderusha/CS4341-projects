# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from sensed_world import SensedWorld
from events import Event
from colorama import Fore, Back
# Imports for code implementation
from queue import PriorityQueue
import random
import math
import os.path

class TestCharacter(CharacterEntity):

    def do(self, wrld):
        # Your code here

        # Prints the current position of the character after the character moves
        print(self.x, self.y)
        # self.learn(wrld)
        w = self.getW()
        f = self.getF()
        a = self.getActions(wrld)
        Q = []

        bombs = self.getBomb(wrld)
        if len(bombs) > 0:
            print("Removing bomb")
            if (0, 0) in a:
                a.remove((0, 0))
            # for act in a:
            #     if wrld.wall_at(act[0], act[1]):
            #         a.remove(act)
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
        # print(1==self.rowBelowIsAllWalls(wrld, (0,0)))
        # if (1==self.rowBelowIsAllWalls(wrld, (0,0)) or maxA == (0,0)) and len(self.getBomb(wrld)) == 0:# or wrld.wall_at(maxA[0], maxA[1]):
        if (maxA == (0,0)):
            self.place_bomb()
        else:
            self.move(maxA[0], maxA[1])

        # # Find the start (current position) and goal
        # start = (self.x, self.y)
        # goal = self.findGoal(wrld)
        #
        # # Get all possible directions fro agent
        # allDirections = self.getAllDirections(wrld, start)
        # allSpaces = self.getAllSpaces(wrld, start)
        #
        # # Find the current best move for the agent
        # bestScoreMove = self.scoreMoves(wrld, start, goal, allDirections, allSpaces)
        # if(bestScoreMove == 'B'):
        #     self.place_bomb()
        # else:
        #     self.move(bestScoreMove[0], bestScoreMove[1])
        #
        # # Go to the goal state if the path leads to a space next to it. ie Terminal Test
        # self.goToGoal(start, goal)

        pass

    # Chooses the best direction to go in based on heuristics
    #
    # PARAM: [ world, [int, int], (int, int)]: wrld: the current state of the world
    #                                          [start.x, start.y]: the x and y coordinated the agent is located at
    #                                          [goal.x, goal.y]: the x and y coordinated of the goal / exit
    #                                          allDirections: a list of all the directions the agent can go
    #
    # RETRUNS: the best move for the agent
    #
    def scoreMoves(self, wrld, start, goal, allDirections, allSpaces):
        enemies = self.getEnemy(wrld)
        bombs = self.getBomb(wrld)
        explosions = self.getExplosion(wrld)
        futureExplosions = self.getFutureExplosions(wrld)

        # Gets the move recommended by A*
        a_star_move = self.get_a_star_move(wrld, start, goal)
        bestMove = (a_star_move[0], a_star_move[1])

        # print("******************\nA* Move\n******************")
        # print(start)
        # print(a_star_move)
        # print("******************")

        # if there are no bombs, don't worry about avoiding them
        if len(bombs) == 0:
            # Deterermine if agent can move that direction
            # if (a_star_move not in allDirections and len(bombs) != 1 and len(explosions) == 0):
            #     bestMove = 'B'
            #     return bestMove
            if (wrld.wall_at(start[0]+a_star_move[0], start[1]+a_star_move[1]) and len(explosions) == 0):
                bestMove = 'B'
                return bestMove

            highestScore = -1
            for space in allSpaces:
                # print("****************************")
                # print(space)
                livingScore = abs(wrld.time)

                enemyScore = 0
                for enemyLoc in enemies:
                    futureX = space[0]
                    futureY = space[1]

                    enemyDis = math.sqrt((enemyLoc[0] - futureX) ** 2 + (enemyLoc[1] - futureY) ** 2)
                    if (enemyDis < 4):
                        enemyScore = enemyScore - ((4 - enemyDis) * 6)

                a_star_score = 0
                # if(a_star_move == allDirections[i]):
                if (start[0]+a_star_move[0] == space[0]) and (start[1]+a_star_move[1] == space[1]):
                    a_star_score = 5

                totalScore = livingScore + a_star_score + enemyScore
                # print(space[0] - start[0], space[1]-start[1], totalScore)
                if(totalScore > highestScore):
                    highestScore = totalScore
                    bestMove = (space[0] - start[0], space[1]-start[1])

        # if there is a bomb, ignore a* and just stay alive. also don't put another bomb down
        else:

            highestScore = -1
            for space in allSpaces:
                # print("****************************")
                # print(space)
                livingScore = abs(wrld.time)

                # try not to walk onto a bomb
                bombScore = 0
                onBomb = False
                for bombLoc in bombs:
                    if (space[0] == bombLoc[0] and space[1] == bombLoc[1]):
                        # print("bomb at", bombLoc)
                        bombScore += -10

                    if (start[0] == bombLoc[0] and start[1] == bombLoc[1]):
                        onBomb = True

                # try not to walk onto an explosion
                explosionScore = 0
                for explosionLoc in explosions:
                    if (space[0] == explosionLoc[0] and space[1] == explosionLoc[1]):
                        explosionScore += -5

                # unless you are standing on a bomb, try not to walk into a space that is going to get exploded
                for futureExplosionLoc in futureExplosions:
                    if (space[0] == futureExplosionLoc[0] and space[1] == futureExplosionLoc[1]) and not onBomb:
                        explosionScore += -5

                enemyScore = 0
                for enemyLoc in enemies:
                    futureX = space[0]
                    futureY = space[1]

                    enemyDis = math.sqrt((enemyLoc[0] - futureX) ** 2 + (enemyLoc[1] - futureY) ** 2)
                    if (enemyDis < 4):
                        enemyScore = enemyScore - ((4 - enemyDis) * 6)

                a_star_score = 0
                # if(a_star_move == allDirections[i]):
                if (start[0] + a_star_move[0] == space[0]) and (start[1] + a_star_move[1] == space[1]):
                    a_star_score = 5

                totalScore = livingScore + a_star_score + bombScore + explosionScore + enemyScore
                # print(space[0] - start[0], space[1] - start[1], totalScore)
                if (totalScore > highestScore):
                    highestScore = totalScore
                    bestMove = (space[0] - start[0], space[1] - start[1])

        return bestMove


    # Gets the locations of the enemies location
    #
    # PARAM: [world] wrld: the current state of the world
    #
    # RETURNS: [list [entities]] enemies: a list of enemies
    #
    def getEnemy(self, wrld):
        enemies = []

        x = 0
        y = 0
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

        x = 0
        y = 0
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


    # Goes to the goal / exit
    #
    # PARAM: [[int, int], [int, int]]: [start.x, start.y]: the x and y coordinated the agent is located at
    #                                  [goal.x, goal.y]: the x and y coordinated of the goal / exit
    # RETURNS: [int, int] x, y: the x and y coordinates of the direction the of the goal
    #
    def goToGoal(self, start, goal):
        # Move Vertical Down
        if start == (goal[0], goal[1] - 1):
            return self.move(0, 1)
        # Move Vertical Up
        elif start == (goal[0], goal[1] + 1):
            return self.move(0, -1)
        # Move Horizontal Right
        elif start == (goal[0] - 1, goal[1]):
            return self.move(1, 0)
        # Move Horizontal Left
        elif start == (goal[0] + 1, goal[1]):
            return self.move(-1, 0)
        # Move Diagonal Right-Down
        elif start == (goal[0] - 1, goal[1] - 1):
            return self.move(1, 1)
        # Move Diagonal Right-Up
        elif start == (goal[0] - 1, goal[1] + 1):
            return self.move(1, -1)
        # Move Diagonal Left-Down
        elif start == (goal[0] + 1, goal[1] - 1):
            return self.move(-1, 1)
        # Move Diagonal Left-Up
        elif start == (goal[0] - 1, goal[1] - 1):
            return self.move(-1, -1)

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
                    # print(x,y)
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
                    new_cost = cost_so_far[current] + 20 # + graph.cost(current, next)
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

    # Returns list of all the possible moves for agent (up, down, left, right, diagonal)
    #
    # PARAM: [world, [int, int]] wrld: the current world configuration
    #        [start.x, start.y] currentLocation: the x and y coordinated the agent is located at
    # RETURNS: [list [int, int]] allMoves: a list of all the possible moves for agent
    #
    def getAllSpaces(self, wrld, currentLocation):
        movesList = []

        # Look right
        # Avoid out of bound look ups
        if((currentLocation[0] + 1) < wrld.width()):
            if(wrld.empty_at(currentLocation[0] + 1, currentLocation[1])):
                movesList.append((currentLocation[0] + 1, currentLocation[1]))
        # Look left
        # Avoid out of bound look ups
        if ((currentLocation[0] - 1) >= 0):
            if (wrld.empty_at(currentLocation[0] - 1, currentLocation[1])):
                movesList.append((currentLocation[0] - 1, currentLocation[1]))
        # Look down
        # Avoid out of bound look ups
        if ((currentLocation[1] - 1) >= 0):
            if (wrld.empty_at(currentLocation[0], currentLocation[1] - 1)):
                movesList.append((currentLocation[0], currentLocation[1] - 1))
        # Look up
        # Avoid out of bound look ups
        if ((currentLocation[1] + 1) < wrld.height()):
            if (wrld.empty_at(currentLocation[0], currentLocation[1] + 1)):
                movesList.append((currentLocation[0], currentLocation[1] + 1))
        # Look diagonal right, up
        # Avoid out of bound look ups
        if ((currentLocation[0] + 1) < wrld.width() and (currentLocation[1] + 1) < wrld.height()):
            if (wrld.empty_at(currentLocation[0] + 1, currentLocation[1] + 1)):
                movesList.append((currentLocation[0] + 1, currentLocation[1] + 1))
        # Look diagonal right, down
        # Avoid out of bound look ups
        if ((currentLocation[0] + 1) < wrld.width() and (currentLocation[1] - 1) >= 0):
            if (wrld.empty_at(currentLocation[0] + 1, currentLocation[1] - 1)):
                movesList.append((currentLocation[0] + 1, currentLocation[1] - 1))
        # Look diagonal left, up
        # Avoid out of bound look ups
        if ((currentLocation[0] - 1) >= 0 and (currentLocation[1] + 1) < wrld.height()):
            if (wrld.empty_at(currentLocation[0] - 1, currentLocation[1] + 1)):
                movesList.append((currentLocation[0] - 1, currentLocation[1] + 1))
        # Look diagonal left, down
        # Avoid out of bound look ups
        if ((currentLocation[0] - 1) >= 0 and (currentLocation[1] - 1) >= 0):
            if (wrld.empty_at(currentLocation[0] - 1, currentLocation[1] - 1)):
                movesList.append((currentLocation[0] - 1, currentLocation[1] - 1))

        return movesList

    # Returns list of all the possible directions for agent (up, down, left, right, diagonal)
    #
    # PARAM: [world, [int, int]] wrld: the current world configuration
    #        [start.x, start.y] currentLocation: the x and y coordinated the agent is located at
    # RETURNS: [list [int, int]] allMoves: a list of all the possible moves for agent
    #
    def getAllDirections(self, wrld, currentLocation):
        directionList = []

        # Look right
        # Avoid out of bound look ups
        if ((currentLocation[0] + 1) < wrld.width()):
            if (wrld.empty_at(currentLocation[0] + 1, currentLocation[1])):
                directionList.append((1, 0))
        # Look left
        # Avoid out of bound look ups
        if ((currentLocation[0] - 1) >= 0):
            if (wrld.empty_at(currentLocation[0] - 1, currentLocation[1])):
                directionList.append((-1, 0))
        # Look down
        # Avoid out of bound look ups
        if ((currentLocation[1] - 1) >= 0):
            if (wrld.empty_at(currentLocation[0], currentLocation[1] - 1)):
                directionList.append((0, -1))
        # Look up
        # Avoid out of bound look ups
        if ((currentLocation[1] + 1) < wrld.height()):
            if (wrld.empty_at(currentLocation[0], currentLocation[1] + 1)):
                directionList.append((0, 1))
        # Look diagonal right, up
        # Avoid out of bound look ups
        if ((currentLocation[0] + 1) < wrld.width() and (currentLocation[1] + 1) < wrld.height()):
            if (wrld.empty_at(currentLocation[0] + 1, currentLocation[1] + 1)):
                directionList.append((1, 1))
        # Look diagonal right, down
        # Avoid out of bound look ups
        if ((currentLocation[0] + 1) < wrld.width() and (currentLocation[1] - 1) >= 0):
            if (wrld.empty_at(currentLocation[0] + 1, currentLocation[1] - 1)):
                directionList.append((1, -1))
        # Look diagonal left, up
        # Avoid out of bound look ups
        if ((currentLocation[0] - 1) >= 0 and (currentLocation[1] + 1) < wrld.height()):
            if (wrld.empty_at(currentLocation[0] - 1, currentLocation[1] + 1)):
                directionList.append((-1, 1))
        # Look diagonal left, down
        # Avoid out of bound look ups
        if ((currentLocation[0] - 1) >= 0 and (currentLocation[1] - 1) >= 0):
            if (wrld.empty_at(currentLocation[0] - 1, currentLocation[1] - 1)):
                directionList.append((-1, -1))

        return directionList

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
        path = []
        # lastMove = []

        # possibleGoalEnds = self.getAllSpaces(wrld, goal)
        # for endMove in possibleGoalEnds:
        #     for next in a_star_graph[1]:
        #         if (next == endMove):
        #             lastMove = endMove
        #
        # path.append(lastMove)
        move = goal
        while (move != start):
            lastMove = move
            move = a_star_graph[0][lastMove]
            # if(len(path) == 1):
            #     return path

            # for next in a_star_graph[0]:
            #     if(next == a_star_graph[0][lastMove]):
            #         lastMove = next
            #         path.append(lastMove)

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
        defaultMove = (0,0)
        a_star_path = self.a_star_search(wrld, start, goal)
        # print("****************************************")
        # print(a_star_path[0])
        # print(a_star_path[1])

        direction = self.findPath(start, goal, wrld, a_star_path)

        # # If no path found, then move towards goal
        # if(len(direction[0]) == 0):
        #     return self.moveTowardsGoal(start, goal)
        #
        # nextMove = direction[len(direction) - 2]
        nextMove = direction
        # print(start)
        # print(nextMove)
        a_star_move = (nextMove[0] - self.x, nextMove[1] - self.y)
        return a_star_move

    # Returns the move that leads the agent closer to the goal
    #
    # PARAM: [[int, int], [int, int]] wrld: [start.x, start.y]: the x and y coordinated the agent is located at
    #                                       [goal.x, goal.y]: the x and y coordinated of the goal / exit
    #
    # RETURNS: [int, int]: (x, y): the directions towards th goal
    #
    def moveTowardsGoal(self, start, goal):
        x = 0
        y = 0
        # Get x component
        if (start[0] > goal[0]):
            x = -1
        if (start[0] < goal[0]):
            x = 1
        # Get y component
        if (start[1] > goal[1]):
            y = -1
        if (start[1] < goal[1]):
            y = 1

        return (x, y)

    def learn(self, wrld):
        # replace this with reading from a file after the first run
        w = self.getW()
        f = self.getF()
        for feat in f:
            print(feat(wrld, (0,1)))

        if len(w) != len(f):
            print("YOUR CODE IS BROKE")
            sys.exit(-1)

        # DISCOUNT FACTOR
        gamma = 0.9

        # INITIAL LEARNING RATE
        # will decrease by 0.1 every 2500 iterations
        alpha = 0.5
        alphaDecFreq = 2500
        alphaDecRate = 0.1


        # EXPLORATION VS EXPLOITATION FACTOR
        # will decrease by 0.1 every 1500
        epsilon = 0.8
        epsilonDecFreq = 1500
        epsilonDecRate = 0.1

        # Learning will start from this world
        s = SensedWorld.from_world(wrld)

        done = False
        for i in range(10000):
            if i == 100 or i == 200 or i == 300 or i == 400 or i == 500 or i % 1000 == 0:
                print(i)
            if i % epsilonDecFreq == 0:
                epsilon -= epsilonDecRate
            if i % alphaDecFreq == 0:
                alpha -= alphaDecRate

            if done:
                # print("Reset")
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
            # print(me.x, me.y, act)
            if act == (0, 0):
                me.place_bomb()
                # print("Bombed")
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

        with open('../S1V1weights.txt', 'w+') as out:
            for weight in w:
                out.write(str(weight))
                out.write("\n")
        print("LEARNING IS FINISHED!!!!!!!!!!!!!!!!!!!!!")
        print(w)
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
                # print("Won")
                return 1000
                # score = 1000
                # break
            elif e.tpe == Event.CHARACTER_KILLED_BY_MONSTER or e.tpe == Event.BOMB_HIT_CHARACTER:
                # print("Dead")
                return -10000
                # score = -10000
                # break
            elif e.tpe == Event.BOMB_HIT_MONSTER:
                score += 100
            elif e.tpe == Event.BOMB_HIT_WALL:
                score += 10
        me = s.me(self)
        temp = (s.width() * s.height()) - self.a_star_heuristic(self.findGoal(s), (me.x, me.y))
        score += temp

        return score

    # returns a list of all legal actions
    def getActions(self, s):
        me = s.me(self)
        a = []
        # print("********Getting Actions*******")
        for x in range(-1, 2):
            # print(x)
            for y in range(-1, 2):
                # print(x, y, x < 0, x >= s.width(), y < 0, y >= s.height(), s.wall_at(x, y))
                # if x == 0 and y == 0:
                #     print(me.x+x > 0, me.x+x <= s.width(), me.y+y > 0, me.y+y <= s.height())
                #     continue
                if me.x+x >= 0 and me.x+x < s.width() and me.y+y >= 0 and me.y+y < s.height() and not s.wall_at(me.x+x, me.y+y):
                    a.append((x, y))
        return a

    # returns number of monsters
    def numMonsters(self, s, a):
        me = s.me(self)
        if a == (0, 0):
            me.place_bomb()
        else:
            # print(a)
            me.move(a[0], a[1])
        newS, events = s.next()
        monsters = self.getEnemy(newS)
        return len(monsters)

    # Returns 1/(1 + distance to closest monster)
    # if there are no monsters, just makes the distance monstrous
    def distToCloseMonster(self, s, a):
        me = s.me(self)
        # if a == (0, 0):
        #     me.place_bomb()
        # else:
        #     me.move(a[0], a[1])
        # newS, events = s.next()
        # me = newS.me(self)
        # for e in events:
        #     if e.tpe == Event.BOMB_HIT_CHARACTER or e.tpe == Event.CHARACTER_KILLED_BY_MONSTER:
        #         return 1
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
        # if a == (0, 0):
        #     me.place_bomb()
        # else:
        #     me.move(a[0], a[1])
        # newS, events = s.next()
        # for e in events:
        #     if e.tpe == Event.BOMB_HIT_CHARACTER or e.tpe == Event.CHARACTER_KILLED_BY_MONSTER:
        #         return 0
        # me = newS.me(self)
        goal = self.findGoal(s)
        dist = self.a_star_heuristic2((me.x + a[0], me.y + a[1]), goal)
        return float(1) / (1 + dist)

    # returns 1 for True, 0 for False
    def charIsOnFutureExplosion(self, s, a):
        me = s.me(self)
        # if a == (0, 0):
        #     me.place_bomb()
        # else:
        #     me.move(a[0], a[1])
        # newS, events = s.next()
        # me = newS.me(self)
        # for e in events:
        #     if e.tpe == Event.BOMB_HIT_CHARACTER or e.tpe == Event.CHARACTER_KILLED_BY_MONSTER:
        #         return 1
        futExps = self.getFutureExplosions(s)
        for fe in futExps:
            if fe[0] == me.x+a[0] and fe[1] == me.y+a[1]:
                return 1
        return 0

    # Returns 1/(1 + distance to closest explosion)
    # if there are no explosions, just makes the distance monstrous
    def distToExplosion(self, s, a):
        me = s.me(self)
        # if a == (0, 0):
        #     me.place_bomb()
        # else:
        #     me.move(a[0], a[1])
        # newS, events = s.next()
        # me = newS.me(self)
        # for e in events:
        #     if e.tpe == Event.BOMB_HIT_CHARACTER or e.tpe == Event.CHARACTER_KILLED_BY_MONSTER:
        #         return 1
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
        # if a == (0, 0):
        #     me.place_bomb()
        # else:
        #     me.move(a[0], a[1])
        # newS, events = s.next()
        # me = newS.me(self)
        # for e in events:
        #     if e.tpe == Event.BOMB_HIT_CHARACTER or e.tpe == Event.CHARACTER_KILLED_BY_MONSTER:
        #         return 1
        bomb = self.getBomb(s)
        closestDist = 9999999
        for b in bomb:
            dist = self.a_star_heuristic2((me.x + a[0], me.y + a[1]), b)
            if dist <= closestDist:
                closestDist = dist
        return float(1) / (1 + closestDist)

    # returns 1 if True 0 if False
    def rowBelowIsAllWalls(self, s, a):
        me = s.me(self)
        if me.y + 1 == s.height():
            # this is an edge case, and bombing here would be pointless, so just say its not true
            return 0

        for x in range(s.width()):
            if not s.wall_at(x, me.y + 1):
                return 0
        return 1

    # returns 1 if True 0 if False
    def closestMonsterToRight(self, s, a):
        me = s.me(self)
        # if a == (0, 0):
        #     me.place_bomb()
        # else:
        #     me.move(a[0], a[1])
        # newS, events = s.next()
        # me = newS.me(self)
        monsters = self.getEnemy(s)

        if len(monsters) == 0:
            return False
        closestDist = 9999999
        closestMonster = monsters[0]
        for m in monsters:
            dist = self.a_star_heuristic2((me.x, me.y), m)
            if dist <= closestDist:
                closestDist = dist
                closestMonster = m
        if closestMonster[0] > me.x:
            return 1
        else:
            return 0

    # returns 1 if True 0 if False
    def closestMonsterToLeft(self, s, a):
        me = s.me(self)
        # if a == (0, 0):
        #     me.place_bomb()
        # else:
        #     me.move(a[0], a[1])
        # newS, events = s.next()
        # me = newS.me(self)
        monsters = self.getEnemy(s)

        if len(monsters) == 0:
            return False
        closestDist = 9999999
        closestMonster = monsters[0]
        for m in monsters:
            dist = self.a_star_heuristic2((me.x, me.y), m)
            if dist <= closestDist:
                closestDist = dist
                closestMonster = m
        if closestMonster[0] < me.x:
            return 1
        else:
            return 0

    # returns 1 if True 0 if False
    def closestMonsterInLine(self, s, a):
        me = s.me(self)
        monsters = self.getEnemy(s)

        if len(monsters) == 0:
            return False
        closestDist = 9999999
        closestMonster = monsters[0]
        for m in monsters:
            dist = self.a_star_heuristic2((me.x, me.y), m)
            if dist <= closestDist:
                closestDist = dist
                closestMonster = m
        if closestMonster[0] == me.x:
            return 1
        else:
            return 0

    # returns 1 if True 0 if False
    def moveIsAStar(self, s, a):
        me = s.me(self)
        aStarMove = self.get_a_star_move(s, self.findGoal(s), (me.x, me.y))
        if a[0] == aStarMove[0] and a[1] == aStarMove[1]:
            return 1
        else:
            return 0

    # returns 1 if True 0 if False
    def putsMonsterInChaseRange(self, s, a):
        me = s.me(self)
        monsters = self.getEnemy(s)
        MONSTER_RANGE = 1
        for m in monsters:
            if (m[0] < me.x + a[0] + MONSTER_RANGE and m[0] > me.x + a[0] - MONSTER_RANGE and \
                m[1] < me.y + a[1] + MONSTER_RANGE and m[1] > me.y + a[1] - MONSTER_RANGE):
                return 1
        return 0


    def getW(self):
        if not os.path.isfile('../S1V1weights.txt'):
            return [123.83037171415249, 16.29366378320882, 2673.8763744646444, -744.1394860357999, -2326.0931442411047,
                    -450.32324619610006, 13.635223346116918, 27.463532128963898, 10.0]
        w = []
        with open('../S1V1weights.txt', 'r') as fd:
            for line in fd:
                w.append(float(line))

        f = self.getF()
        if len(w) != len(f):
            print("Using default weights")
            return [123.83037171415249, 16.29366378320882, 2673.8763744646444, -744.1394860357999, -2326.0931442411047, -450.32324619610006, 13.635223346116918, 27.463532128963898, 10.0]
        else:
            return w

    def getF(self):
        return [self.numMonsters, self.distToCloseMonster, self.distToGoal,
             self.charIsOnFutureExplosion, self.distToExplosion, self.distToBomb,
             self.rowBelowIsAllWalls,
             self.closestMonsterInLine, self.moveIsAStar]
