import math
import numpy as np
from rrt import Node

class Utils:
    def __init__(self, obstacleTolerance, boundary, circleObstacles, rectangleObstacles):
        self.obsTolerance = obstacleTolerance
        self.circleObstacles = circleObstacles
        self.rectangleObstacles = rectangleObstacles
        self.boundary = boundary

    def isInsideObs(self, node):       
        #check if node lies inside or at edge of any circle centered at (x,y)
        #(x - x1)^2 + (y - y1)^2 <= r point is in or on circle
        for (x, y, radius) in self.circleObstacles:
            if math.hypot(x - node.x, y - node.y) <= radius + self.obsTolerance:
                #collision detected
                return True

        #check if node lies in rectangle/boundary obstacles
        #xmin<=node.x<=xmax same for y
        for (x, y, width, height) in (self.rectangleObstacles+self.boundary):
            xMin = x - self.obsTolerance
            xMax = x + width + self.obsTolerance
            yMin = y - self.obsTolerance
            yMax = y + height + self.obsTolerance
            if xMin <= node.x <= xMax and yMin <= node.y <= yMax:
                return True

        #No collision detected
        return False
    
    #Get array with 4 Vertexes of each rectangle
    def getRectangleObsVertex(self):
        rectObsVertexArray = []
        #for all rectangles (x,y) is bottom left origin
        for (x, y, width, height) in self.rectangleObstacles:
            vertex_list = [[x - self.obsTolerance, y - self.obsTolerance],
                           [x + width + self.obsTolerance, y - self.obsTolerance],
                           [x + width + self.obsTolerance, y + height + self.obsTolerance],
                           [x - self.obsTolerance, y + height + self.obsTolerance]]
            rectObsVertexArray.append(vertex_list)

        return rectObsVertexArray

    #Line to line intersection forumla 
    def isRectInteresection(self, startNode, endNode, v1, v2):
        #A Point
        x1 = startNode.x
        y1 = startNode.y
        #B Point
        x2 = endNode.x
        y2 = endNode.y
        #Rectangle Vertex 1
        x3 = v1[0]
        y3 = v1[1]
        #Rectangle Vertex 2
        x4 = v2[0]
        y4 = v2[1]

        #line intersection formula derivation
        #http://paulbourke.net/geometry/pointlineplane/

        denominator = ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))

        # Lines are parallel when denominator equals to 0
        if denominator == 0:
            return False

        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / (denominator + 1e-16)
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / (denominator + 1e-16)

        #If ua and ub lie between 0 and 1
        #the intersection point is within both line segments.
        if (0 <= ua <= 1 and 0 <= ub <= 1):
            return True
       
        return False

    #basic distance calculation between 2 x,y points
    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)
    
    #circle intersection
    def isCircleIntersection(self, aPos, dispVector, center, radius):
        #magnitude of direction vector
        dispVectorMag = np.dot(dispVector, dispVector)

        #if magnitude is 0 then we havent moved anywhere
        if dispVectorMag == 0:
            return False

        #projection of line onto vector from center circle to start (aPos)
        projection = np.dot([center[0] - aPos[0], center[1] - aPos[1]], dispVector) / dispVectorMag

        #if projection is between 0 and 1 the closest point to circle center is on line
        if 0 <= projection <= 1:
            #get x,y values of closest point to circle center
            shot = Node((aPos[0] + projection * dispVector[0], aPos[1] + projection * dispVector[1]))
            #check distance from circle and see if it is greater than radius
            if self.get_dist(shot, Node(center)) <= radius + self.obsTolerance:
                return True

        return False
    
    #Collision Detecion Boolean
    def isCollision(self, startNode, endNode):
        #Check if startNode or endNode point is inside an obstacle
        if self.isInsideObs(startNode) or self.isInsideObs(endNode):
            return True

        #array filled with corners of rectangles
        rectObsVertexArray = self.getRectangleObsVertex()

        #for each rectangle (set of 4 vertexes) check for intersection
        for (v1, v2, v3, v4) in rectObsVertexArray:
            if self.isRectInteresection(startNode, endNode, v1, v2):
                return True
            if self.isRectInteresection(startNode, endNode, v2, v3):
                return True
            if self.isRectInteresection(startNode, endNode, v3, v4):
                return True
            if self.isRectInteresection(startNode, endNode, v4, v1):
                return True

        #A [X,Y] Position
        aPos = [startNode.x, startNode.y]
        #AB Displacement Vector
        dispVector = [endNode.x - startNode.x, endNode.y - startNode.y]

        #for each circle check intersection
        for (x, y, radius) in self.circleObstacles:
            if self.isCircleIntersection(aPos, dispVector, [x, y], radius):
                return True

        return False
