import sys, getopt
import math
import cv2
import numpy as np

#Cairn: RobotHome 4000 4000 0.000000 "" ICON ""

def main(argv):

    #getting input and output file
   inputfile = ''
   outputfile = ''

   # cell size in mm
   cellSize=0
#   cellsize = 1000/60 = 17

   try:
      opts, args = getopt.getopt(argv,"hi:o:s:",["hhelp", "ifile=","ofile=","sample="])
   except getopt.GetoptError:
      print 'generateWallsfromMap.py -i <inputfile> -o <outputfile> -s <cellsize>'
      sys.exit(2)
#   print "opts:",opts," args:",args
   for opt, arg in opts:
      if opt == '-h':
         print 'generateWallsfromMap.py -i <inputfile> -o <outputfile> -s <cellsize>'
         sys.exit()
      elif opt in ("-i", "--ifile"):
         inputfile = arg
      elif opt in ("-o", "--ofile"):
         outputfile = arg
      elif opt in ("-s", "--sample"):
         if is_number(arg): 
            cellSize = float(arg)
         else:
            print '-s stands for the map cell discretization in milimeters.'             
            sys.exit(2)


                 
   #opening the file
   readMapFile(inputfile, cellSize, outputfile)

# reading the input file
def readMapFile(aMap, sampling, outputfile):

   # getting map size
   width  = 0
   height = 0
   minX = 0
   minY = 0
   maxX = 0
   maxY = 0
#   dlimit = 57
   dlimit = 5

   # data required for line extraction
   robotPose = (0,0)
   hasRobotPose = False
   numLines = 0
   expectedNumLines = 0
   lineCountStarted = False
   lineCountEnded = False
   lineList = []
   endoffile=False

   # read map file
   with open(aMap, 'r') as f:
       data = f.readlines()

   for line in data:
       words = line.split()
       # Check for true 2D Map file format
       if words[0] == '2D-Map':
          isMapFile = True
          print bcolors.HEADER, aMap, 'is probably a valid .map file. Processing...', bcolors.ENDC 
          print bcolors.WARNING, 'Map sampling set to', sampling, 'mm.', bcolors.ENDC

       # get the Map dimensions
       elif words[0] == 'LineMinPos:':
            if len(words) == 3:
                if is_number(words[1]) and is_number(words[2]):
                   minX = float(words[1])
                   minY = float(words[2])
                else:
                   print bcolors.FAIL, aMap, 'has no min position to create a valid map.', bcolors.ENDC
                   sys.exit(2)
       elif words[0] == 'LineMaxPos:':
            if len(words) == 3:
                if is_number(words[1]) and is_number(words[2]):
                   maxX = float(words[1])
                   maxY = float(words[2])
                else:
                   print bcolors.FAIL, aMap, 'has no max position to create a valid map.', bcolors.ENDC
                   sys.exit(2)

       # get number of lines
       elif words[0] == 'NumLines:':
           if len(words) == 2:
               if is_number(words[1]):
                   expectedNumLines = float(words[1])
                   if expectedNumLines <= 0:
                       print bcolors.FAIL,'Invalid number of Lines.', bcolors.ENDC
                       sys.exit(2)
               else:
                   print bcolors.FAIL,'The number of lines in the file contains an error.', words[1], bcolors.ENDC
                   sys.exit(2)
           else:
               print bcolors.FAIL,'Number of lines not available, the .map file lacks information.', bcolors.ENDC
               sys.exit(2)

       # detect line sequence start
       elif words[0] == 'LINES':
           if len(words)==1:
               lineCountStarted = True
           else:
               print bcolors.WARNING,'The line start section contains an error.', bcolors.ENDC
               sys.exit(2)
       # extract lines vector if count started and not finished
       elif lineCountStarted and not (lineCountEnded):
           if len(words) == 4: # check number of elements
               numElem = 0
               for e in words: # check if the 4 elements are numbers
                   if is_number(e): 
                       numElem=numElem+1
               if numElem == 4:
                   numLines = numLines + 1
                   # generate discrete line points with minimum coordinates 0,0 displaced by 57 -- the discretization limit
                   dlimit = 57
                   lineList.append( np.array( ( ( int(round((float(words[0])-minX)/sampling)+dlimit),  # p1x
                                                  int(round((float(words[1])-minY)/sampling)+dlimit) ),# p1y
                                                ( int(round((float(words[2])-minX)/sampling)+dlimit),  # p2x
                                                  int(round((float(words[3])-minY)/sampling)+dlimit) ) # p2y
                                              )  ))
                   if expectedNumLines == numLines: # Line list ended
                       lineCountEnded = True
               else:
                   print bcolors.FAIL,'A line from the list has a non-number element.',   bcolors.ENDC
                   sys.exit(2)
           else:
               print bcolors.FAIL,'A line from the list has a wrong number of elements.', bcolors.ENDC
               sys.exit(2)

       # store robot pose
       elif words[0] == 'Cairn:':
           if len(words)>=4:
               if words[1] == 'RobotHome':
                   if is_number(words[2]) and is_number(words[3]):
                       hasRobotPose=True
                       robotPose = ( int(round((float(words[2])-minX)/sampling)+dlimit), int(round((float(words[3])-minY)/sampling)+dlimit) )
                   else:
                       print bcolors.FAIL,'The given robot position is not a valid coordinate. This info is MANDATORY.', bcolors.ENDC
                       sys.exit(2)
               else:
                   print bcolors.WARNING,'A line expected to possibly containing the robot position was present, but no robot pose was found.', bcolors.ENDC
           else:
               print bcolors.WARNING,'No sufficient info found about the robot pose was found. Errors may happen.', bcolors.ENDC

       # detect end of file
       elif words[0] == 'DATA':
           endoffile = True
   
   # Calculating image width and height
   print bcolors.OKGREEN, 'There are', int(expectedNumLines), 'lines in the .map file', bcolors.ENDC
   width  = maxX-minX
   height = maxY-minY
   print bcolors.OKGREEN, 'The map has width', width, 'mm and height', height, 'mm',  bcolors.ENDC
   print bcolors.OKGREEN, 'Adding 57 cells at each of the map borders and sampling:', bcolors.ENDC
   width =int(round(width/ sampling)+2*dlimit)
   height=int(round(height/sampling)+2*dlimit)
   print bcolors.OKGREEN, '\tnew width: ', width,  bcolors.ENDC
   print bcolors.OKGREEN, '\tnew height:', height, bcolors.ENDC
   if hasRobotPose:
      print bcolors.OKGREEN, 'The robot pose was extracted', robotPose, bcolors.ENDC
   else:
      print bcolors.WARNING, 'WARNING: NO ROBOT POSE DETECTED, seting to 0, 0', bcolors.ENDC
   if endoffile:
       print bcolors.OKGREEN, 'Data succesfully extracted... now loading', bcolors.ENDC
   
   # constructing discrete map
   print bcolors.WARNING, 'All lines discretized -- map centered in the image.\n Contructing map...', bcolors.ENDC
   constructMap(lineList, width, height, numLines, robotPose, outputfile)



def constructMap(lineList, width, height, numLines, robotPose, outputfile):
    
    # The image
    size = height, width, 3
    m = np.zeros(size, dtype=np.uint8)
    m[:] = (150, 150, 150)
    m = cv2.cvtColor(m, cv2.COLOR_BGR2GRAY)

    # Drawing the lines
    for n in lineList:
       p1 = (n[0][0], height-n[0][1])
       p2 = (n[1][0], height-n[1][1])
       cv2.line(m, p1, p2, 0, 2) # line width, not pixel count!!! e.g. 2 => 2+1

    dlimit=5

    #robotPose = (robotPose[0], height-robotPose[1])
    robotPose = (width/2, height/3)
    print robotPose

    # Fill image with gray color
    msize = height+2, width+2, 3
    mask = np.zeros(msize, dtype=np.uint8)
    mask[:] = 0
    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
  
    # Dilate walls (erode free-space)
    kernel = np.ones((3,3),np.uint8)
    m = cv2.erode(m,kernel,iterations = 1)

    # fill free space with white color
    diff = (6, 6, 6)
    cv2.floodFill(m, mask, robotPose, 255, diff, diff, cv2.FLOODFILL_FIXED_RANGE)
    
    #print m

    # store image in archive file a text
    f = open(outputfile, 'w')
    f.write(str(width)+' '+str(height)+'\n')
    counter = 0
    for i in range(height):
        s = ''
        for j in range(width):
            if m[i,j] == 150:
                s+="- "
            if m[i,j] == 0:
                s+="1 "
                counter+=1
            if m[i,j] == 255:
                s+="0 "
                counter+=1
                 
        s+='\n'
        f.write(s)
    f.close()
    print counter

    cv2.namedWindow("The map")
    rm = cv2.resize(m, (0,0), fx=0.4, fy=0.4)

    while True:
        cv2.imshow("The map", rm)    
        ch = 0xFF & cv2.waitKey(1)
        if ch == 27:
            break
    cv2.destroyAllWindows()


def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False           

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'


if __name__ == "__main__":
   main(sys.argv[1:])
