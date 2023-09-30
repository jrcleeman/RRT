import math
import numpy as np
import obstacleBuilder, plotPath, collisionDetection

class Node:
    def __init__(self, positionArray):
        #Node X position
        self.x = positionArray[0]
        #Node Y position
        self.y = positionArray[1]
        #Node that comes before this node (Singularly Linked List)
        self.parent = None

class RRT:
    def __init__(self, obstacleTolernace, startNodeArray, goalNodeArray, stepLength, goalSampleRate, maxIterations):
        self.obsTolerance = obstacleTolernace
        self.startNode = Node(startNodeArray)
        self.goalNode = Node(goalNodeArray)
        self.stepLength = stepLength
        self.goalSampleRate = goalSampleRate
        self.maxIterations = maxIterations
        self.nodesExplored = [self.startNode] #List of all explored nodes
        
        #Build Env (Boundaries and Obstacles)
        tempEnv = obstacleBuilder.Env()
        self.xMin = tempEnv.xRange[0]
        self.xMax = tempEnv.xRange[1]
        self.yMin = tempEnv.yRange[0]
        self.yMax = tempEnv.yRange[1]

        #Build Utils Class 
        self.utils = collisionDetection.Utils(self.obsTolerance, tempEnv.boundary , tempEnv.circleObstacles, tempEnv.rectangleObstacles)

        #Build Plotter
        self.plotting = plotPath.Plotting(startNodeArray, goalNodeArray,\
                                           tempEnv.boundary , tempEnv.circleObstacles, tempEnv.rectangleObstacles)


    #Generate random node or goal node
    def generateRandomNode(self):
        #Is random number between 0 and 0.99 > goalSampleRate
        if np.random.random() > self.goalSampleRate:
            #Generate a node with an random X and Y value between bounds wrt tolerance
            xCoord = np.random.random() * ((self.xMax - self.obsTolerance) - (self.xMin + self.obsTolerance)) + (self.xMin + self.obsTolerance)
            yCoord = np.random.random() * ((self.yMax - self.obsTolerance) - (self.yMin + self.obsTolerance)) + (self.yMin + self.obsTolerance)
            return Node((xCoord, yCoord))
        
        #else return goal node
        return self.goalNode
    
    #Find nearest neighbor of randomNode  
    def findNearestNeighbor(self, randomNode):
        #Find index of explored node with shortest distance to randomNode
        closestNodeIndex = np.argmin([math.hypot(node.x - randomNode.x, node.y - randomNode.y) for node in self.nodesExplored])
        return self.nodesExplored[int(closestNodeIndex)]

    #Add New Node
    def createNewState(self, nearestNode, targetNode):
        dist, theta = self.getDistanceAndAngle(nearestNode, targetNode)
        #What is smaller stepLength or dist btwn nodes?
        dist = min(self.stepLength, dist)
        #new node move as far as possible in X and Y in dir of targetNode
        newNode = Node((nearestNode.x + dist * math.cos(theta),
                         nearestNode.y + dist * math.sin(theta)))
        #parent node assigned
        newNode.parent = nearestNode
        return newNode   

    #Get distance and angle off x axis between start and end
    @staticmethod
    def getDistanceAndAngle(startNode, endNode):
        dx = endNode.x - startNode.x
        dy = endNode.y - startNode.y
        return math.hypot(dx, dy), math.atan2(dy, dx)  

    #get path array for plotting
    def extractPath(self, endNode):
        #last node is the goal node
        path = [(self.goalNode.x, self.goalNode.y)]
        #node before goal node
        currentNode = endNode

        #until the node origin is reached recusive call linked list
        while currentNode.parent != None:
            #add node x,y info to path array
            path.append((currentNode.x, currentNode.y))
            #call node previous
            currentNode = currentNode.parent

        return path   

    #Execute RRT
    def execute(self):
        #Make nodes until max limit is reached
        for i in range(self.maxIterations):
            randomNode = self.generateRandomNode()
            nearestNode = self.findNearestNeighbor(randomNode)
            newNode = self.createNewState(nearestNode, randomNode)
            #If newNode exists and is not colliding
            if newNode and not self.utils.isCollision(nearestNode, newNode):
                #add node to list of traveled locations
                self.nodesExplored.append(newNode)
                #get dist between newNode and goalNode
                dist, _ = self.getDistanceAndAngle(newNode, self.goalNode)
            
                #path from initial to goal found 
                if dist <= self.stepLength and not self.utils.isCollision(newNode, self.goalNode):
                    #create goal node
                    self.createNewState(newNode, self.goalNode)
                    #returns array of (x,y) coordinates that lead back to origin
                    return self.extractPath(newNode)

        return None

#Main Method
def main():
    #Starting node
    startX = 2
    startY = 2
    startNode = (startX, startY)  
    #Goal node
    goalX = 49
    goalY = 24
    goalNode = (goalX, goalY)  
    #Obstacle Tolernace/Fit
    obstacleTolerance = 0.5

    #RRT Step Length (Max movement btwn Nodes)
    stepLength = 0.5
    #How often goal node is sampled
    goalSampleRate = 0.05
    #Maximum number of node explorations
    maxIterations = 10000
    #Create RRT  
    rrt = RRT(obstacleTolerance, startNode, goalNode, stepLength, goalSampleRate, maxIterations)
    #Execute path planning algo
    path = rrt.execute()

    #If path is found
    if path:
        rrt.plotting.animation(rrt.nodesExplored, path, True)
    
    #Couldnt find path in number of iters
    else:
        print("No Path Found!")

#Run Script
if __name__ == '__main__':
    #call main
    main()
