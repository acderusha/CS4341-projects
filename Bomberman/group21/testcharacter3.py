# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
# Imports for code implementation
from queue import PriorityQueue
import math

infinity = 1000000

class TestCharacter(CharacterEntity):

    def do(self, wrld):
        # Your code here

        # Prints the current position of the character after the character moves
        print(self.x, self.y)

        # Find the start (current position) and goal
        start = (self.x, self.y)
        goal = self.findGoal(wrld)

        # Get all possible directions fro agent
        allDirections = self.getAllDirections(wrld, start)
        allSpaces = self.getAllSpaces(wrld, start)

        # Find the current best move for the agent
        bestScoreMove = self.scoreMoves(wrld, start, goal, allDirections, allSpaces)
        if(bestScoreMove == 'B'):
            self.place_bomb()
        else:
            self.move(bestScoreMove[0], bestScoreMove[1])

        # Go to the goal state if the path leads to a space next to it. ie Terminal Test
        self.goToGoal(start, goal)

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
            if (wrld.wall_at(start[0]+a_star_move[0], start[1]+a_star_move[1]) and len(explosions) == 0):
                bestMove = 'B'
                return bestMove

            highestScore = -1
            for space in allSpaces:
                print("****************************")
                print(space)
                livingScore = abs(wrld.time)

                enemyScore = self.getEnemyScore(wrld, space)
                locationScore = self.getLocationScore(wrld, space)

                a_star_score = 0
                # if(a_star_move == allDirections[i]):
                if (start[0]+a_star_move[0] == space[0]) and (start[1]+a_star_move[1] == space[1]):
                    a_star_score = 5

                totalScore = livingScore + a_star_score + enemyScore + locationScore
                print(space[0] - start[0], space[1]-start[1], totalScore)
                if(totalScore > highestScore):
                    highestScore = totalScore
                    bestMove = (space[0] - start[0], space[1]-start[1])

        # if there is a bomb, ignore a* and just stay alive. also don't put another bomb down
        else:

            highestScore = -1
            for space in allSpaces:
                print("****************************")
                print(space)
                livingScore = abs(wrld.time)

                # try not to walk onto a bomb
                bombScore = 0
                onBomb = False
                for bombLoc in bombs:
                    if (space[0] == bombLoc[0] and space[1] == bombLoc[1]):
                        print("bomb at", bombLoc)
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
                # for enemyLoc in enemies:
                #     futureX = space[0]
                #     futureY = space[1]
                #
                #     enemyDis = math.sqrt((enemyLoc[0] - futureX) ** 2 + (enemyLoc[1] - futureY) ** 2)
                #     if (enemyDis < 3):
                #         enemyScore = enemyScore - ((3 - enemyDis) * 6)

                a_star_score = 0
                # if(a_star_move == allDirections[i]):
                if (start[0] + a_star_move[0] == space[0]) and (start[1] + a_star_move[1] == space[1]):
                    a_star_score = 5

                totalScore = livingScore + a_star_score + bombScore + explosionScore + enemyScore
                print(space[0] - start[0], space[1] - start[1], totalScore)
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

    # Gets an enemy score based on enemy locations
    #
    # PARAM: [world] wrld: the current state of the world
    #
    # RETURNS: [int] enemyScore: a score based on enemy locations
    #
    def getEnemyScore(self, wrld, space=False):
        minDis = 3.5
        weight = 10

        if(space == False):
            enemyScore = 0
            enemies = self.getEnemy(wrld)
            for enemyLoc in enemies:
                enemyDis = math.sqrt((enemyLoc[0] - self.x) ** 2 + (enemyLoc[1] - self.y) ** 2)
                if (enemyDis < minDis):
                    enemyScore = enemyScore - ((minDis - enemyDis) * weight)
                if(enemyDis < 3):
                    enemyScore = -infinity

        else:
            enemies = self.getEnemy(wrld)
            enemyScore = 0
            for enemyLoc in enemies:
                futureX = space[0]
                futureY = space[1]

                enemyDis = math.sqrt((enemyLoc[0] - futureX) ** 2 + (enemyLoc[1] - futureY) ** 2)
                if (enemyDis < minDis):
                    enemyScore = enemyScore - ((minDis - enemyDis) * weight)
                if (enemyDis < 3):
                    enemyScore = -infinity

        return enemyScore


    # Gets a score based on self location
    #
    # PARAM: [world] wrld: the current state of the world
    #
    # RETURNS: [int] locationScore: a score based on self location
    #
    def getLocationScore(self, wrld, space=False):
        # Being next to an edge offers less mobility
        locationScore = 0
        width = wrld.width()
        height = wrld.height()

        if (space == False):
            if(self.x == 0 or self.x == width - 1):
                locationScore = locationScore - 2

            if(self.y == 0 or self.y == height - 1):
                locationScore = locationScore - 2

        else:
            if (space[0] == 0 or space[0] == width - 1):
                locationScore = locationScore - 2

            if (self.y == 0 or self.y == height - 1):
                locationScore = locationScore - 2

        return locationScore


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
                    print(x,y)
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
                    priority = new_cost + self.a_star_heuristic2(goal, next, wrld)
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
    def a_star_heuristic2(self, goal, next, wrld):
        enemyScore = self.getEnemyScore(wrld)
        locationScore = self.getLocationScore(wrld, next)

        return math.sqrt(((goal[0] - next[0]) ** 2) + ((goal[1] - next[1]) ** 2)) + enemyScore + locationScore

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
        print("****************************************")
        print(a_star_path[0])
        print(a_star_path[1])

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
