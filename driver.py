import numpy #allows use to create a distance matrix
import busRoute  #imports all bus stop data
from pprint import pprint #use to print arrays in a "pretty fashion"
from scipy.spatial import distance_matrix   #set of modules that allows us to solve TSP
from ortools.constraint_solver import pywrapcp 
from ortools.constraint_solver import routing_enums_pb2
import math

def getAllBusses():
  #Tuple of all busses in busRoute.py
  allBusses = ('1', '10', '108', '12', '14', '14L', '14X', '16X', '17', '18', '19', '1AX', '1BX', '2', 
      '21', '22', '23', '24', '27', '28', '28L', '29', '3', '30', '30X', '31', '31AX', '31BX',
      '33', '35', '36', '37', '38', '38AX', '38BX', '38L', '39', '41', '43', '44', '45', '47',
      '48', '49', '5', '52', '54', '56', '59', '5L', '6', '60', '61', '66', '67', '71', '71L',
      '76X', '81X', '82X', '83X', '88', '8AX', '8BX', '8X', '9', '90', '91', '9L', 'F', 'J',
      'K OWL', 'KT', 'L', 'L OWL', 'M', 'M OWL', 'N', 'N OWL', 'NX', 'T OWL') 
  return allBusses

def getLat(bus):
  bus = str(bus)
  lat = []     
  
  for i in range(0, len(busRoute.muniStops.get(bus))):
    lat.append(busRoute.muniStops[bus][i].get('lat'))
    
  #casts all latitude and longitude coordinates from strings to floats  
  for i in range(0, len(busRoute.muniStops)):
    lat[i] = float(lat[i])
  return lat

def getLong(bus):
  bus = str(bus)
  lon = []
  
  for i in range(0, len(busRoute.muniStops.get(bus))):
    lon.append(busRoute.muniStops[bus][i].get('lon'))

  #casts all latitude and longitude coordinates from strings to floats  
  for i in range(0, len(busRoute.muniStops)):
    lon[i] = float(lon[i])
  return lon

def getStops(bus):
  bus = str(bus)
  lon = getLong(bus)
  lat = getLat(bus)
  allStops = []

  #initializes allCords as a list of containing only lists
  #each sublist contains latitude and longitude coordinates
  for i in range(0, len(busRoute.muniStops.get(bus))):
    for j in range(0, 1):
      pairOfCoords = []
      pairOfCoords.append(lat[i])
      pairOfCoords.append(lon[i])
    allStops.append(pairOfCoords)
  return allStops  
  
def graphBusRoute(bus):
  #Bottom Left:   37,780410, -122.502327
  #Top Left:    37,780410, -122.477558
  #Bottom Right:  37.914398, -122.374201
  #Top Right:   37.914398, -122.410618
  bus = str(bus)
  lon = getLong(bus)  #x-coord
  lat = getLat(bus)   #y-coord

  plt.figure(figsize=(5, 5))  #sets length and width of graph
  plt.plot(lon, lat, 'k', markersize = 5)
  plt.tick_params(labelleft=False)
  plt.tick_params(labelbottom=False)
  plt.gca().invert_xaxis() #bandaid solution
  plt.show()

def findDistance(x1, x2, y1, y2):
  return float(math.sqrt((x1-x2)**2 + (y1-y2)**2))

def sameRoute(object):
  #Logic: excluding the first and last positions of a given 'list' named object, if each
  #       array position is decreasing then that means the original route must have not changed.
  #       Thus, the original route must have been the most efficient beforehand

  #Precondition: object is of type 'list'
  #Postcondition: returns whether given list is the same as the previous route
  isSameRoute = True
  errorFound = 0
  for i in object[len(object):1]:   #search starts at the last position and ends at the first position
    if (i-1 != i):
      errorFound += 1
  if errorFound > 0:
    isSameRoute = False
  return isSameRoute 

def main():
  numpy.set_printoptions(threshold=numpy.nan)  #allows to print entire NumPy-type array

  bus = str(43) #userinput here
  allStops = getStops(bus)
  allStops = numpy.array(allStops, dtype=numpy.float16)

  #Finds the shortest Hamiltonian circuit distance possible
  distMatrix = distance_matrix(allStops, allStops)  #creates a distance matrix of type 'numpy array'
  distMatrix = distMatrix.tolist()      #converts from type 'numpy array' to python's type 'list'
  numLocations = len(allStops)
  NUM_ROUTES = 1  #because we only want one route
  DEPOT = 0   #because we want "no empty vectors in place of empty routes"

  if numLocations > 0:  #if given set of locations contains more than one node
    routing = pywrapcp.RoutingModel(numLocations, NUM_ROUTES, DEPOT)
    searchParameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    assignment = routing.SolveWithParameters(searchParameters)

    if assignment:  #if assignment value exists, that is able to solve with parameters
      routeNum = 0
      node = routing.Start(routeNum)
      startNode = node
      route = " "
      altRoute = " "
      routeList = []
      totalDistance = 0
      DEGREES_TO_MILES = 69

      while not routing.IsEnd(node):  #while finding the most efficient route until reaching final node
        routeList.append(node)    #creates a list of node positions, used to compare whether list IS ACTUALLY most efficient
        altRoute += str(node) + " -> "
        route += "\n" + "\t" + "[" + str(node) + "]: " + busRoute.muniStops.get(bus)[node].get('title')
        node = assignment.Value(routing.NextVar(node))

        #calculating cost of route (as in total distance covered)
        try:
          x1 = float(busRoute.muniStops.get(bus)[node-1].get('lon')) * DEGREES_TO_MILES
          x2 = float(busRoute.muniStops.get(bus)[node].get('lon')) * DEGREES_TO_MILES
          y1 = float(busRoute.muniStops.get(bus)[node-1].get('lat')) * DEGREES_TO_MILES
          y2 = float(busRoute.muniStops.get(bus)[node].get('lat')) * DEGREES_TO_MILES
          totalDistance += findDistance(x1, x2, y1, y2)
        except IndexError:  #if variable 'stops' is out of range, ignore calculating (double check if valid)
          pass
      
      altRoute += "0"   #because it is a circuit, must start at it's end
      route += "\n" + "\t" + "[0]: " + busRoute.muniStops.get(bus)[0].get('title')  #because it is a circuit, must end at it's start

      print("Bus Line: " + str(bus))
      print("Most Efficient Route by Position: " + "\n" + "\t" + str(altRoute))
      print("\nMost Efficient Circuit Route by Street: " + str(route))
      print("\nTotal Shortest Distance: " + str(round(totalDistance,3)) + " miles")
    else: #if unable to find a route
      print("Cannot find most efficient route")
  else: #if unable to solve with parameters (because given list only has one node)
    print("Please specify a list of stops with more than zero stops")

  print("Is this the same as original route: " + str(sameRoute(routeList)).lower())
if __name__ == '__main__':
  main()
