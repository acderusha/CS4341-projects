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
depth = 3

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

        # checks if there is an enemy within 2 blocks
        enemyInRange = False
        for enemy in enemies:
            if (abs(enemy[0] - start[0]) <= 7 or abs(enemy[1] - start[1]) <= 7):
                enemyInRange = True

        if not enemyInRange:
            # if there are no bombs, don't worry about avoiding them
            # if len(bombs) == 0:
            #     if (wrld.wall_at(start[0]+a_star_move[0], start[1]+a_star_move[1]) and len(explosions) == 0):
            #         bestMove = 'B'
            #         return bestMove
            #
            #     highestScore = -1
            #     for space in allSpaces:
            #         print("****************************")
            #         print(space)
            #         livingScore = abs(wrld.time)
            #
            #         enemyScore = 0
            #         for enemyLoc in enemies:
            #             futureX = space[0]
            #             futureY = space[1]
            #
            #             enemyDis = math.sqrt((enemyLoc[0] - futureX) ** 2 + (enemyLoc[1] - futureY) ** 2)
            #             if (enemyDis < 4):
            #                 enemyScore = enemyScore - ((4 - enemyDis) * 6)
            #
            #         a_star_score = 0
            #         # if(a_star_move == allDirections[i]):
            #         if (start[0]+a_star_move[0] == space[0]) and (start[1]+a_star_move[1] == space[1]):
            #             a_star_score = 5
            #
            #         totalScore = livingScore + a_star_score + enemyScore
            #         print(space[0] - start[0], space[1]-start[1], totalScore)
            #         if(totalScore > highestScore):
            #             highestScore = totalScore
            #             bestMove = (space[0] - start[0], space[1]-start[1])
        #
        #     else: # enemy is in range
        print("EXECUTING MINIMAX")

        # Consider placing a bomb as an action
        allDirections.append('B')

        bestMove = -1
        bestValue = -1
        index = 0
        for action in allDirections:
            value = self.maximize(start, goal, action, wrld, depth, -infinity, infinity)
            print(action, value)

            if (bestMove == -1):
                bestMove = allDirections[index]
                bestValue = value

            else:
                if (value > bestValue):
                    bestValue = value
                    bestMove = allDirections[index]

            index += 1

        return bestMove



        # if there is a bomb, ignore a* and just stay alive. also don't put another bomb down
        # else:
        #
        #     highestScore = -1
        #     for space in allSpaces:
        #         print("****************************")
        #         print(space)
        #         livingScore = abs(wrld.time)
        #
        #         # try not to walk onto a bomb
        #         bombScore = 0
        #         onBomb = False
        #         for bombLoc in bombs:
        #             if (space[0] == bombLoc[0] and space[1] == bombLoc[1]):
        #                 print("bomb at", bombLoc)
        #                 bombScore += -10
        #
        #             if (start[0] == bombLoc[0] and start[1] == bombLoc[1]):
        #                 onBomb = True
        #
        #         # try not to walk onto an explosion
        #         explosionScore = 0
        #         for explosionLoc in explosions:
        #             if (space[0] == explosionLoc[0] and space[1] == explosionLoc[1]):
        #                 explosionScore += -5
        #
        #         # unless you are standing on a bomb, try not to walk into a space that is going to get exploded
        #         for futureExplosionLoc in futureExplosions:
        #             if (space[0] == futureExplosionLoc[0] and space[1] == futureExplosionLoc[1]) and not onBomb:
        #                 explosionScore += -5
        #
        #         enemyScore = 0
        #         for enemyLoc in enemies:
        #             futureX = space[0]
        #             futureY = space[1]
        #
        #             enemyDis = math.sqrt((enemyLoc[0] - futureX) ** 2 + (enemyLoc[1] - futureY) ** 2)
        #             if (enemyDis < 4):
        #                 enemyScore = enemyScore - ((4 - enemyDis) * 6)
        #
        #         a_star_score = 0
        #         # if(a_star_move == allDirections[i]):
        #         if (start[0] + a_star_move[0] == space[0]) and (start[1] + a_star_move[1] == space[1]):
        #             a_star_score = 5
        #
        #         totalScore = livingScore + a_star_score + bombScore + explosionScore + enemyScore
        #         print(space[0] - start[0], space[1] - start[1], totalScore)
        #         if (totalScore > highestScore):
        #             highestScore = totalScore
        #             bestMove = (space[0] - start[0], space[1] - start[1])
        #
        # return bestMove


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


    # Returns true if the enemy is right next to you
    #
    # PARAM: [list[int, int]] enemies: the location of all the enemies
    #        [(int, int)] action: the direction the agent wants to move
    #        [(int, int)] start: the current location of the agent
    #
    # RETURNS: [boolean] nextTo: specifies if an enemy is next to the agent
    #
    def isEnemeyNextTo(self, start, action, enemies):
        if(action != 'B'):
            nextTo = False
            newSpot = (start[0] + action[0], start[1] + action[1])
            for enemy in enemies:
                if(abs(enemy[0] - newSpot[0]) <= 1 or abs(enemy[1] - newSpot[1]) <= 1):
                    nextTo = True
        else:
            nextTo = False
            newSpot = (start[0], start[1])
            for enemy in enemies:
                if (abs(enemy[0] - newSpot[0]) <= 1 or abs(enemy[1] - newSpot[1]) <= 1):
                    nextTo = True

        return nextTo



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
        return self.dist(next, goal)

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

    # Returns the maximum move score that leads the agent closer to the goal based on a minimax decision
    #
    # PARAM: [(int, int)] start: the x and y coordinated the agent is located at
    #        [(int, int)] goal: the x and y coordinate of the goal
    #        [(int, int)] action: the dx and dy direction the
    #        [world] wrld: the current state of the world
    #        [int] depth: the current depth of the board
    #        [int] alpha
    #        [int] beta
    #
    # RETURNS: [int]: the maximum possible score for that direction
    #
    def maximize(self, start, goal, action, wrld, depth, alpha, beta):
        # Terminal Test for if the Agent has died

        # Update the world
        newWrld = self.UpdateWrld(start, goal, action, wrld)

        # Retrieve all possible next moves of the updated world
        allDirections = self.getAllDirections(newWrld, start)
        # Add bomb as a possible action
        allDirections.append('B')

        # If max depth is reached, then evaluate current state
        if (depth == 0):  # call heuristics and return heruistic score
            first = True
            maxVal = 0
            value = 0
            for action in allDirections:
                value = self.scoreState(wrld, start, goal, action)

                if (first):
                    maxVal = value
                    first = False
                else:
                    if (value > maxVal):
                        maxVal = value

            return maxVal

        else:
            value = -infinity
            for action in allDirections:
                value = max(value, self.minimize(start, goal, action, newWrld, depth-1, alpha, beta))
                if (value >= beta):
                    return value
                alpha = max(alpha, value)

            return value

    # Returns the minimum move score that leads the agent closer to the goal based on a minimax decision
    #
    # PARAM: [(int, int)] start: the x and y coordinated the agent is located at
    #        [(int, int)] goal: the x and y coordinate of the goal
    #        [(int, int)] action: the dx and dy direction the
    #        [world] wrld: the current state of the world
    #        [int] depth: the current depth of the board
    #        [int] alpha
    #        [int] beta
    #
    # RETURNS: [int]: the maximum possible score for that direction
    #
    def minimize(self, start, goal, action, wrld, depth, alpha, beta):
        # Terminal Test for if the Agent has died

        # Update the world
        newWrld = self.UpdateWrld(start, goal, action, wrld)

        # Retrieve all possible next moves of the updated world
        allDirections = self.getAllDirections(newWrld, start)
        # Add bomb as a possible action
        allDirections.append('B')

        # If max depth is reached, then evaluate current state
        if (depth == 0):  # call heuristics and return heruistic score

            first = True
            minVal = 0
            value = 0
            for action in allDirections:
                value = self.scoreState(wrld, start, goal, action)

                if (first):
                    minVal = value
                    first = False
                else:
                    if (value < minVal):
                        minVal = value

            return minVal

        else:
            value = infinity
            for action in allDirections:
                value = min(value, self.maximize(start, goal, action, newWrld, depth-1, alpha, beta))
                if (value <= alpha):
                    return value
                beta = min(beta, value)

            return value

    # Gets the score of the current state based on heuristics
    #
    # PARAM: [ world, [int, int], (int, int)]: wrld: the current state of the world
    #                                          [start.x, start.y]: the x and y coordinated the agent is located at
    #                                          [goal.x, goal.y]: the x and y coordinated of the goal / exit
    #                                          [(int, int)] action: the action for the agent to evaluate
    #
    # RETRUNS: the best move for the agent
    #
    def scoreState(self, wrld, start, goal, action):
        enemies = self.getEnemy(wrld)
        bombs = self.getBomb(wrld)
        explosions = self.getExplosion(wrld)

        # Get A* heuristic
        if(action != 'B'):
            start = (start[0] + action[0], start[1] + action[1])
        a_star_score = self.dist(start, goal)

        # Get enemy heuristic
        enemyScore = self.enemyScore(start, enemies)

        # Get Bomb about to explode heuristic
        bombScore = self.bombScore(start, explosions)

        totalScore = a_star_score # + enemyScore + bombScore
        return totalScore


    # Gets the enemy score of the current state
    #
    # PARAM: [ world, [int, int], (int, int)]: wrld: the current state of the world
    #        [start.x, start.y]: start: the x and y coordinated the agent is located at
    #        [list[(int, int)]]: enemies: the x and y coordinated of all the enemies
    #
    # RETRUNS: a score based on the proximity of enemies
    #
    def enemyScore(self, start, enemies):
        # Max Number of enemies is 2
        maxEnemies = 2

        # The lower the number of enemies the better
        enemyExistanceScore = (maxEnemies - len(enemies)) * 10

        # Farther away enemies are the better
        enemyProx = 0
        for enemyLoc in enemies:
            enemyDis = math.sqrt((enemyLoc[0] - start[0]) ** 2 + (enemyLoc[1] - start[1]) ** 2)
            if (enemyDis < 4):
                enemyProx = enemyProx - ((4 - enemyDis) * 6)

        totalEnemyScore = enemyExistanceScore + enemyProx
        return -totalEnemyScore



    # Gets the bomb score of the current state
    #
    # PARAM: [start.x, start.y]: start: the x and y coordinated the agent is located at
    #        [list(x, y)] explosions: list of explosions
    #
    # RETRUNS: a score based on the proximity of explosion
    #
    def bombScore(self, start, explosions):
        bombScore = 0

        for fire in explosions:
            if(start[0] == fire[0]):
                bombScore = bombScore + 20
            if (start[1] == fire[1]):
                bombScore = bombScore + 20

        return -bombScore


    # Gets the next board state
    #
    # PARAM: [world, [int, int], (int, int)]: wrld: the current state of the world
    #        [int, int]: action: the action the agent is attempting to take
    #
    # RETRUNS: a new world state
    #
    def UpdateWrld(self, start, goal, action, wrld):
        newSensedWrld = wrld.next()
        newWrld = newSensedWrld[0]

        # Get the agent and monsters in the new world
        print(next(iter(newWrld.characters.values()))[0])
        agentSim = next(iter(newWrld.characters.values()))[0]
        monsterSim = next(iter(newWrld.monsters.values()))[0]

        # Move the agent
        if(action != 'B'):
            agentSim.move(action[0], action[1])
        else:
            agentSim.place_bomb()

        # Move the monsters
        # Get direction leading to agent
        monsterX = 0
        monsterY = 0
        if(monsterSim.x > agentSim.x):
            monsterX = -1
        elif (monsterSim.x < agentSim.x):
            monsterX = 1
        if (monsterSim.y > agentSim.y):
            monsterY = -1
        elif (monsterSim.y < agentSim.y):
            monsterY = 1
        monsterSim.move(monsterX, monsterY)

        return newWrld



    def dist(self, start, goal):
        return math.sqrt(((goal[0] - start[0]) ** 2) + ((goal[1] - start[1]) ** 2))
