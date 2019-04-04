







####################################### # Importing required libraries ########################################


# For calculations
import numpy as np
import math

#OpenCv
import cv2
import cv2.aruco as aruco

#OpenGl
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

#Pillow
from PIL import Image

#Pygame
import pygame

from objloader import *

#pyserial
import serial

#For delays
import time



############################# Global variables ######################33

# OpenGL related globals

#Varibale for holding pitcher aruco id
PitcherId = None

#Flag to check if the Pitcher has been reached with a pebble
PitcherChanged = 0

#List for holding pebble aruco ids
PebbleIds = []

#Variable used as flag while bot traversing
changeflag = 0

#Variable to hold number of pebble
NumberOfPebbles = None

#Flag to check if the first pebble has been picked up
Pebble1Changed = False

#Variable to hold aruco id of first pebble
Pebble1 = None

#List to hold all other pebble ids in sequence
PebbleSeq = []

#Variable to check which pebble has been currently picked up
PebbleChoose = 0

#List to hold information of already picked up pebbles
AlreadyDimnished = []

#List which holds data , which when received are to be treated as flags and not as actual data
ExceptList = ['q','w','e'] # start to pebble, pebble to pitcher, pitcher to pebble


# OpenGL essentials
texture_object = None
texture_background = None
camera_matrix = None
dist_coeff = None

INVERSE_MATRIX = np.array([[ 1.0, 1.0, 1.0, 1.0],
                           [-1.0,-1.0,-1.0,-1.0],
                           [-1.0,-1.0,-1.0,-1.0],
                           [ 1.0, 1.0, 1.0, 1.0]])


# Crow object

crow = None
dimnishedrock = None

#Rock object
rock = None

#Pot Full Object
fullpot = None

#Pot Empty Object
emptypot = None

#Pot Half Full Object
halfpot = None

#Initilising Camera
cap = cv2.VideoCapture(1)


# Algorithm related gloabls

# List for containing the shortest path Instruction set
ShortPath = []

# To bring bot to the first node (common step)
ShortPath.append('f')

#Variable to index th commands
Index = 0;

#Initialising the data variable 
Data = 'T'

#Global variables for Arena

  #Arena mapping as a graph
Arena = {
        #Cell 10
          'a' : ['b','f'],
          'b' : ['s','c','a'],
          'c' : ['v','b', 'd'],
          'd' : ['c','g','e'],
          'e' : ['d','f','z'],
          'f' : ['a','e','w'],

        #Cell 7

          'g' : ['h','d','l'],
          'h' : ['D','i','g'],
          'i' : ['h','M','j'],
          'j' : ['i','m','k'],
          'k' : ['j','l','N'],
          'l' : ['g','k','E'],

        #cell 14

          'm' : ['n','j','r'],
          'n' : ['U','o','m'],
          'o' : ['Ab','n','p'],
          'p' : ['o','q'],
          'q' : ['p','r','Y'],
          'r' : ['m','q','V'],


        #cell 5

          's' : ['t','b'],
          't' : ['u','s'],
          'u' : ['A','t','v'],
          'v' : ['u','D','c'],

        #cell 15
          'w' : ['f','x'],
          'x' : ['w','y'],
          'y' : ['z','x','H'],
          'z' : ['e','E','y'],

        #cell 2

          'A' : ['B', 'u'],
          'B' : ['I','A', 'C'],
          'C' : ['B','L', 'D'],
          'D' : ['C','v', 'h'],


        #cell 6
        #Connected nodes with cells 5,2,7,10

        #cell 11

         'E' : ['l','z','F'],

        #Cell 16

         'F' : ['E','O','G'],
         'G' : ['F','H','R'],
         'H' : ['y','G'],


       #Cell 1
         'I' : ['J','B'],
         'J' : ['I','K'],
         'K' : ['J','S','L'],
         'L' : ['K','C','M'],

       #Cell 3
         'M' : ['L','U','i'],


       #Cell 12
         'N' : ['k','V','O'],
         'O' : ['N','F','P'],

       #Cell 17

         'P' : ['O','X','Q'],
         'Q' : ['P','R'],
         'R' : ['G','Q'],


       #Cell 4
         'S' : ['K','T'],
         'T' : ['S','Aa','U'],
         'U' : ['T','M','n'],

       #Cell 8
        #Connect with cell nodees 14 4 3 7

       #Cell 13
         'V' : ['r','N','W'],

       #cell 18
         'W' : ['V','Z', 'X'],
         'X' : ['W', 'P'],

       #Cell 19
         'Y' : ['q','Z'],
         'Z' : ['Y','W'],

       #Cell 9
         'Aa' : ['T','Ab'],
         'Ab' : ['Aa','o']

          }


  #Arena mapping for bot traversing( Not for creating a graph)
   #Notice the zeros instead of no value for generalising code

Arena_path = {
        #Cell 10
          'a' : ['b',0,'f'],
          'b' : ['s','c','a'],
          'c' : ['v','b', 'd'],
          'd' : ['c','g','e'],
          'e' : ['d','f','z'],
          'f' : ['a','e','w'],

        #Cell 7

          'g' : ['h','d','l'],
          'h' : ['D','i','g'],
          'i' : ['M','h','j'],
          'j' : ['i','m','k'],
          'k' : ['j','l','N'],
          'l' : ['g','k','E'],

        #cell 14

          'm' : ['n','j','r'],
          'n' : ['U','o','m'],
          'o' : ['Ab','n','p'],
          'p' : ['o',0,'q'],
          'q' : ['p','r','Y'],
          'r' : ['m','q','V'],


        #cell 5

          's' : ['t',0,'b'],
          't' : [0,'u','s'],
          'u' : ['A','t','v'],
          'v' : ['u','D','c'],

        #cell 15
          'w' : ['f',0,'x'],
          'x' : ['w','y',0],
          'y' : ['z','x','H'],
          'z' : ['e','E','y'],

        #cell 2

          'A' : [0,'B', 'u'],
          'B' : ['I','A', 'C'],
          'C' : ['B','L', 'D'],
          'D' : ['C','v', 'h'],


        #cell 6
        #Connected nodes with cells 5,2,7,10

        #cell 11

         'E' : ['l','z','F'],

        #Cell 16

         'F' : ['E','O','G'],
         'G' : ['F','H','R'],
         'H' : ['y','G',0],


       #Cell 1
         'I' : [0,'J','B'],
         'J' : [0,'I','K'],
         'K' : ['J','S','L'],
         'L' : ['K','C','M'],

       #Cell 3
         'M' : ['L','U','i'],


       #Cell 12
         'N' : ['k','V','O'],
         'O' : ['N','F','P'],

       #Cell 17

         'P' : ['O','X','Q'],
         'Q' : ['P','R',0],
         'R' : ['G','Q',0],


       #Cell 4
         'S' : [0,'K','T'],
         'T' : ['S','Aa','U'],
         'U' : ['T','M','n'],

       #Cell 8
        #Connect with cell nodees 14 4 3 7

       #Cell 13
         'V' : ['r','N','W'],

       #cell 18
         'W' : ['V','Z', 'X'],
         'X' : ['W', 'P',0],

       #Cell 19
         'Y' : ['q',0,'Z'],
         'Z' : ['Y','W',0],

       #Cell 9
         'Aa' : [0,'T','Ab'],
         'Ab' : ['Aa',0,'o']

          }
  # Arena Axis information




Cells = { 1: ["1-1","B","K","2-2","I","L","3-3","C","J"],
          2: ["1-1","u","C","2-2","A","D","3-3","v","B"],
          3: ["1-1","D","M","2-2","C","i","3-3","h","L"],
          4: ["1-1","L","T","2-2","K","U","3-3","M","S"],
          5: ["1-1","s","v","2-2","t","c","3-3","b","u"],
          6: ["1-1","c","h","2-2","v","g","3-3","d","D"],
          7: ["1-1","g","J","2-2","h","k","3-3","l","i"],
          8: ["1-1","i","n","2-2","M","m","3-3","j","U"],
          9: ["1-1","U","Ab","2-2","T","o","3-3","n","Aa"],
          10:["1-1","a","d","2-2","b","e","3-3","f","c"],
          11:["1-1","e","l","2-2","d","E","3-3","z","g"],
          12:["1-1","E","N","2-2","l","O","3-3","F","k"],
          13:["1-1","k","r","2-2","j","V","3-3","N","m"],
          14:["1-1","m","p","2-2","n","q","3-3","r","o"],
          15:["1-1","w","z","2-2","f","y","3-3","x","e"],
          16:["1-1","y","f","2-2","z","G","3-3","H","E"],
          17:["1-1","G","P","2-2","F","Q","3-3","R","O"],
          18:["1-1","O","W","2-2","N","X","3-3","P","V"],
          19:["1-1","V","Y","2-2","r","Z","3-3","W","q"]
        }



'''                 Defining motions

*
* Forward - 'f'
* Backward - 'b'
* Right - 'r'
* Left - 'l'
* Stop - 's'
* Reached first pebble - 'p'
* Reached pitcher - 'P'
* Reached pebble - 'O'
* U -  turn  180 degress
* R - right before reaching destination
* L - left before reaching destination
* u - 180 turn before reaching destination
* q - Flag reached first pebble
* w - Flag reached pitcher
* e - Flag reached pebble
'''


# Arena configurations

# Arena config 1
arena_config = {0: ("Water Pitcher", 6, "2-2"), 2: ("Pebble",8,"3-3") , 4: ("Pebble",16,"2-2"), 6: ("Pebble",19,"1-1") }

# Arena config 2
#arena_config= {0: ("Water Pitcher", 9, "2-2"), 2: ("Pebble",2,"3-3") }

#Robot's start position
Robot_start = "START-1"


# Global variables during shortest path calculation

# Flag to denote bot movement
  #0 - left , 1 - Forward , 2 - Right
flag = None

# Bot's orientation according to the sorrounding
  #0 - Bot is facing in the direction it originally started, 1 - Bot is facing in a opposite direction
BotOrientation= 0

#List to hold all final instructions
Motions = []

#Since forward is the very first instruction
Motions.append('f')


# Variable to avoid run time exceptions.
ChangeLevel=None



#Serial Communication
ser = serial.Serial("COM4", 9600, timeout=None)


##################################################################












################## Functions #######################





########################## pyserial related functions ###############################

'''
*
* Function name  :     Send
* Input :              Command - > Character to be send through Serial Communication
* Output :             None
* Logic :              Character is send through the serial port opened earlier and output buffer is reset
* Example call :       Send('f') - > To make bot move forward
*
'''


def Send(Command):

    # Checking if Connection is established
    if ser.isOpen():

        #Converting the command into string (for integer commands)
        Command  = str(Command)

        #Sending the Command through Serial Port
        #Command needs to be encoded before sending in python version above 2
        ser.write(Command.encode())
        time.sleep(0.2)
        #Resetting output buffer
        ser.reset_output_buffer()

    #if port open failed
    else :
        print(" Error : Serial not opened")


'''
*
* Function name  :     Receive
* Input :              None
* Output :             Prints the character receievd through Serial
* Logic :              Character is received through serial communication
* Example call :       Receive()
*
'''

def Receive():
     # Checking if Connection is established
    if ser.isOpen():

        # Waiting to receive a character
        while ser.inWaiting() == 0:
            continue

        # Checking if received a valid character
        size = ser.inWaiting()
        received = ser.read(1)

        #bol check
        if size:
            return received

        else :
            print("Invalid")
        time.sleep(0.2)
    #if port open failed
    else :
        print(" Error : Serial not opened")

    ser.reset_input_buffer()
    #print("")

'''
*
* Function name  :     Receive2
* Input :              None
* Output :             Prints the character receievd through Serial
* Logic :              Character is received through serial communication,Skiped if no character
* Example call :       Receive()
*
'''

def Receive2():
     # Checking if Connection is established
    if ser.isOpen():
        
        msg = ser.read(ser.in_waiting)
        msg = msg.strip()
        return msg
       
    #if port open failed
    else :
        print(" Error : Serial not opened")

    ser.reset_input_buffer()
    #print("")





########################################################################################






################################# Bot traversing/path planning related functions ############################




        





'''
*
* Function name  :     FindShortestPath
* Input :              graph - > To find shortest distance on , start - > Starting node , end - > Destination node , path - > path that is being calculated
* Output :             shortest - > list conataining shortest path from one source to destination
* Logic :              It compares the length of the path with a variable named as shortest which is initialized with the None value.
*                      If the length of generated path is less than the length of shortest,
*                      if shortest is not None, the newly generated path is set as the value of shortest. Again, if there is no path, it returns None
* Example call :       FindShortestPath(Arena, 'a', 'f')
*
'''

def FindShortestPath(graph, start, end, path =[]):
        # Adding the starting node to the path
        path = path + [start]

        #If start is the end
        if start == end:
            return path

        #Variable to hold shortest path
        shortest = None

        #For every adjencent node of current node
        for node in graph[start]:

            #If node is already not in the path
            if node not in path:

                #Recuursion to update the path and to get newpath
                newpath = FindShortestPath(graph, node, end, path)

                #bol check
                if newpath:
                     if not shortest or len(newpath) < len(shortest):
                        shortest = newpath
       #Return the shortest path
        return shortest

'''
*
* Function name  :     DecidePath
* Input :              path - > The shortest path to traverse , Previous - > last two nodes traversed , Cell no - > Cell no to goto or at , flag - > for randome errors
* Output :             List contatitng the set of motions bot should perform
* Logic :              Arena sotred in the form of a python dictionary has adjcent nodes stored in a " Left Center Right" manner
*                      If the next node is at the "Left" Index ( for example at [0] of A node) decision to take left will be given and similar to other turns
* Example call :       DecidePath(path)
*
'''

def DecidePath(Path, Previous=[0,0], CellNo = None, flag = None):

    # Including the global variables
    global BotOrientation, ChangeLevel, ShortPath,Motions

    #Flag for avoiding multiple occurene
    TurnAround = 0

    #Printing Current Path for debugging
    print("")
    print(" P A T H  - ", Path)
    print("")

    # Getting the Current node
    CurrentNode = Path[0]

    # Checking if there is actually a node to traverse to
    if len(Path)>1:

      # Setting the next index element as the next node to traverse to
      NextNode = Path[1]

      #Checking if bot moved atleast two nodes
       #Because atleast two node traversing is required for bot's orientation to change
      if Previous[0] != 0:

        #Error checking prints
        #print("Level 1 ")

        if Previous[1] != 0:

          #Error checking prints
          #print("Level 2 ")

          #If current node is the left of the Previously traversed node
          if CurrentNode == Arena_path[Previous[1]][0]:

            #Error checking prints
            #print("Level 3 ")

            #If the node Previous to the Previous node is right of the Previous node
            if Previous[0] == Arena_path[Previous[1]][2]:

              #If all these conditions are met that means that bot moved 180
              #Or in other words that bot is now facing opposite to the way it originally moved

              #That means Bot's Orientation has changed

              #Error checking prints
             # print("Level 4")

              if BotOrientation == 0:

                BotOrientation = 1

              else :

                BotOrientation = 0



           #This condition is same as previous one except its opossite
          elif Previous[0] == Arena_path[Previous[1]][0] :

            if CurrentNode == Arena_path[Previous[1]][2]:

              #Error checking prints
              #print("Level 5")

              if BotOrientation == 0:

                BotOrientation = 1

              else:

                BotOrientation = 0



      #If the next node is directly opposite of the current node
      if Path[1] == Previous[1]:

        #Error checking Prints
        #print("Level 6")

        #Then bot takes a U turn and Its orientation changes
        if BotOrientation == 0:

          BotOrientation = 1

        else :

          BotOrientation = 0

          #Bot takes a 180 degress turn
        ShortPath.append('U')
        Motions.append('U')

        #Setting the flag
        TurnAround = 1

      #If Robot started from position one
      if Robot_start == "START-1" and TurnAround == 0:

        #Error checking prints
        #print("Level 7")

        #If the next node is at the left of the current node
        if NextNode == Arena_path[CurrentNode][0]:

          #Error checking prints
         # print("Level 8")

          #If bot is facing in the original direction
          if BotOrientation == 0:

            #Bot moves left
            ShortPath.append('l')
            Motions.append('l')

          #If the bot is facing oppositely to its original direction
          else:

            #Error checking prints
           # print("level 9")

            #Bot moves right
            ShortPath.append('r')
            Motions.append('r')

        #If the next node is at the right of the current node
        elif NextNode == Arena_path[CurrentNode][2]:

          #Error checking prints
          #print("Level 10")

          #If bot is facing in the original direction
          if BotOrientation == 0:

            #Bot moves right
            ShortPath.append('r')
            Motions.append('r')
            

          #If the bot is facing oppositely
          else:

            #Bot moves left
            ShortPath.append('l')
            Motions.append('l')

        #If the next node is at the center of the current node
        elif NextNode == Arena_path[CurrentNode][1]:

          #Error checking prints
          #print("Level 11")

          #If the bot is facing in the original direction
          if BotOrientation == 0 :

            #If the last node it came from is on the right of the current node
            if Previous[1] == Arena_path[CurrentNode][2]:

              #Error checking prints
             # print("Level 12")

              #Bot moves right
              ShortPath.append('r')
              Motions.append('r')

            #If the last node it came from is on the left of the current node
            if Previous[1] == Arena_path[CurrentNode][0]:

              #Error checking prints
              #print("Level 13")

              #Bot moves left
              ShortPath.append('l')
              Motions.append('l')

          #If the bot is facing the opposite way
          else :

             #If the last node it came from is on the right of the current node
            if Previous[1] == Arena_path[CurrentNode][2]:

              #Error checking prints
              #print("Level 14")

              #Bot moves left
              ShortPath.append('l')
              Motions.append('l')

            #If the last node it came from is on the left of the current node
            if Previous[1] == Arena_path[CurrentNode][0]:

              #Error checking prints
              #print("Level 15")

              #Bot moves right
              ShortPath.append('r')
              Motions.append('r')


      #If bot started  from start 2
      elif Robot_start == "START-2" and TurnAround == 0 :

         #If the next node is at the left of the current node
        if NextNode == Arena_path[CurrentNode][0]:

          #If bot is facing in the original direction
          if BotOrientation == 0:

            #Bot moves right
            ShortPath.append('r')
            Motions.append('r')

          #If the bot is facing oppositely to its original direction
          else:

            #Bot moves left
            ShortPath.append('l')
            Motions.append('l')

        #If the next node is at the right of the current node
        elif NextNode == Arena_path[CurrentNode][2]:

          #If bot is facing in the original direction
          if BotOrientation == 0:

            #Bot moves left
            ShortPath.append('l')
            Motions.append('l')

          #If the bot is facing oppositely
          else:

            #Bot moves right
            ShortPath.append('r')
            Motions.append('r')

        #If the next node is at the center of the current node
        elif NextNode == Arena_path[CurrentNode][1]:

          #If the bot is facing in the original direction
          if BotOrientation == 0 :

            #If the last node it came from is on the right of the current node
            if Previous[1] == Arena_path[CurrentNode][2]:

              #Bot moves left
              ShortPath.append('l')
              Motions.append('l')

            #If the last node it came from is on the left of the current node
            if Previous[1] == Arena_path[CurrentNode][0]:

              #Bot moves right
              ShortPath.append('r')
              Motions.append('r')

          #If the bot is facing the opposite way
          else :

             #If the last node it came from is on the right of the current node
            if Previous[1] == Arena_path[CurrentNode][2]:

              #Bot moves right
              ShortPath.append('r')
              Motions.append('r')

            #If the last node it came from is on the left of the current node
            if Previous[1] == Arena_path[CurrentNode][0]:

              #Bot moves left
              ShortPath.append('l')
              Motions.append('l')

      # If the last node is reached after traversing
    if len(Path) < 2:

        #Bot reached
        #print("Level Stop")
        ShortPath.append('s')
        Motions.append('s')


      # Updating the "Previous" list so that it contains only last two nodes traversed as others are not required

    if len(Path) >=2:
      #If the bot only traversed one node
       if Previous[0] == 0:

        #Deleteing first element of the list to make room for new node
         del Previous[0]

      #If the len of the "Previous" list is greator than 1
       #Notice the previous line;to avoid multiple removing when not requried
       if len(Previous) > 1:

        # If no nodes have been traversed yet
         if Previous[1] == 0:

          #Delete the first one
           del Previous[1]

        #Delete the 0th index element to make room for new traversed node
         del Previous[0]

      #Appending newly traversed node
       Previous.append(Path[0])

      #Updating the shortest path so that currently traversed node is removed from it
       del Path[0]

      #Recurrsion for traversing all nodes
       DecidePath(Path,Previous,CellNo)




'''
*
* Function name  :     FromWhichNode
* Input :              None
* Output :             None
* Logic :              This function decides which node to start from
* Example call :        WhichNodeToGo
*
'''

def FromWhichNode():

  #If Start is start one
  if Robot_start=="START-1":

    #The starting node is "a"
    Node = 'a'

  #If start is start two
  elif Robot_start == "START-2":

    #The starting node is "p"
    Node = 'p'

  #Return node to start from
  return Node

'''
*
* Function name  :     WhichNodeToGo
* Input :              None
* Output :             None
* Logic :              This function decides which node to goto
* Example call :        WhichNodeToGo
*
'''

def WhichNodeToGo():

  #Pebble information list
  Pebble = []

  #Pitcher information list
  Pitcher = []

  #List to hold Pebble Cell numbers
  PebbleCellNo =[]

  #List to hold pebble axis
  PebbleAxis = []

  #Variable to increment in the while loop
  Entry = 0

  #List to hold destinations
  Destinations = []


  #Getting the Pebble and pitcher information
  Pebble,Pitcher = ExtractInfo()

  #Getting number of elements in pebble list
  NumberOfPebbleAr = len(Pebble)
  Number = NumberOfPebbleAr

  #Finding how many pebbless are there , since there is pebble id and axis info together (// - returns only integer value)
  NumberOfPebbleAr //= 2


  #A loop till last entry of the pebble list
  while Entry < Number:

    #Extracting Pebble ids and Axis information differrently
    PebbleCellNo.append(Pebble[Entry])

    Entry+=1

    PebbleAxis.append(Pebble[Entry])

    Entry+=1

   #Finding the destination node

   #A loop for number of times as the number of pebble
  for Each in range(NumberOfPebbleAr):

    #Find destintions for each entry

    Destinations.append(FindDestination(PebbleAxis[Each],PebbleCellNo[Each]))



  return Destinations,PebbleAxis,Pitcher,PebbleCellNo







'''
*
* Function name  :     GetNode
* Input :              Axis - > Orientation of the Ar_object, Destination - > The node it has to reach
* Output :             Node - > The node that it should traverse previously to the destination
* Logic :              Gets which node to traverl to before reaching the final node so it doesn't has to take extra turns
* Example call :       GetNode
*
'''

def GetNode(Axis, Destination,Mode = None):


  #If mode is None means no prespective was defined that means the bot has to stick to the original
  if Mode == None:

    #If Robot's start is 1 the mode should be 1
    if Robot_start == "START-1":

      Mode = 1

    #If robot's start is 2 the mode should be 2
    elif Robot_start == "START-2":

      Mode = 2
  
  #For inverting bot motions    
  if Mode == "Invert":

     #If Robot's start is 1 the mode should be 1
    if Robot_start == "START-1":

      Mode = 2

    #If robot's start is 2 the mode should be 2
    elif Robot_start == "START-2":

      Mode = 1
    



  #If Start-1 Prespective
  if Mode == 1 :

   #If the axis is 1-1
    if Axis == "1-1":

    #The node to travel to before the destination should be at the center of the destination
     Node  = Arena_path[Destination][1]

  #If the axis is 2-2
    elif Axis == "2-2":

    #The node to travel to before the destination should be at the left of the destination
     Node = Arena_path[Destination][0]

  #If the axis is 3-3
    elif Axis == "3-3":

    #The node to travel to before the destination should be at the right of the destination
     Node = Arena_path[Destination][2]

  if Mode == 2:

    #If the axis is 1-1
    if Axis == "1-1":

    #The node to travel to before the destination should be at the center of the destination
     Node  = Arena_path[Destination][1]

  #If the axis is 2-2
    elif Axis == "2-2":

   #The node to travel to before the destination should be at the rigth of the destination
     Node = Arena_path[Destination][2]
     #print(" H E R E 4")

  #If the axis is 3-3
    elif Axis == "3-3":

    #The node to travel to before the destination should be at the left of the destination
     Node = Arena_path[Destination][0]




  #Returning the calcuated node
  return Node


'''
*
* Function name  :     FindDestination
* Input :              Axis  - Axis of the Ar_Object, CellNo - > Cell number of the Ar_Object. Mode - > From which side (start1 or start2)
* Output :             Destination - > THe destination node
* Logic :              This functions searchs through the dictionary a finds the last node to reach to
* Example call :       FindDestination("1-1","2")
*
'''
def FindDestination(Axis,CellNo,Mode=None):


  #Variable to hold Destination node
  Destination = None

  #If mode is None means no prespective was defined that means the bot has to stick to the original
  if Mode == None:

    #If Robot's start is 1 the mode should be 1
    if Robot_start == "START-1":

      Mode = 1

    #If robot's start is 2 the mode should be 2
    elif Robot_start == "START-2":

      Mode = 2
  #For inverting bot motions    
  if Mode == "Invert":

     #If Robot's start is 1 the mode should be 1
    if Robot_start == "START-1":

      Mode = 2

    #If robot's start is 2 the mode should be 2
    elif Robot_start == "START-2":

      Mode = 1
    
  #If From start 1's prepective
  if Mode == 1:

    #If the axis of the Ar_object is 1-1
    if Axis == "1-1":

      #The node to be set as destination
      Destination = Cells[CellNo][1]

    #If axis is 2-2
    elif Axis == "2-2":

      #The destination node would be
      Destination = Cells[CellNo][4]


     #If axis is 3-3
    elif Axis == "3-3":

      #The destination node would be
      Destination = Cells[CellNo][7]

  #If from start 2's prespective
  if Mode == 2 :

    #If the axis of the Ar_object is 1-1
    if Axis == "1-1":

      #The node to be set as destination
      Destination = Cells[CellNo][2]

    #If axis is 2-2
    elif Axis == "2-2":

      #The destination node would be
      Destination = Cells[CellNo][5]



    #If axis is 3-3
    elif Axis == "3-3":

      #The destination node would be
      Destination = Cells[CellNo][8]




  return Destination






'''
*
* Function name  :     ExtractInfo
* Input :              None
* Output :             None
* Logic :              This function finds the cell number of the pebbles and the pitcher
* Example call :       ExtractInfo()
*
'''

def ExtractInfo():

  #Including the arena config
  global arena_config

  # List for holding information about different pebble placements in the arena
  PebblesInfo = []

  #Variable to hold  information about pitcher , such as the cell number and orientation
  PitcherInfo = []

  #For every element in arena_config Dictionary
  for Info in arena_config:

    #If the element has "Water Pitcher" written in its 0th index
    if arena_config[Info][0] == "Water Pitcher":

      #Store Cell number of the pitcher, which would be in the 1st Index
      PitcherInfo.append(arena_config[Info][1])

      #Then store its orientation axis, which would be in the 2nd index
      PitcherInfo.append(arena_config[Info][2])

    #If the element has "Pebble" written in its 0th index
    elif arena_config[Info][0] == "Pebble":

      #Store the Cell number of the pebble
      PebblesInfo.append(arena_config[Info][1])

      #Store the Orientation axis
      PebblesInfo.append(arena_config[Info][2])

  #Returning the Pebble and Pithcer Information
  return PebblesInfo,PitcherInfo


'''
*
* Function name  :     PrintMovements
* Input :              InstructionSet - > Instrunction set calculated by DecidePath Function
* Output :             Prints the required bot movements
* Logic :              This function doesn't play any role in final implementation, however this function
*                      when called will show the required movements the bot has make to reach its final position
* Example call :       PrintMovements(InstructionSet)
*
'''

def PrintMovements(InstructionSet) :

  #Printing the instruction set
  print("The instruction received are  - ", InstructionSet)
  print(" ")

  # For each movement that bot has to make in order to reach
  for Movement in InstructionSet :

    #If the Instruction is to move left
    if Movement == 'l':

      #Print that bot has to move left
      print("Bot takes a  left");



    #If the Instruction is to move right
    if Movement == 'r':

      #Print thatnbot has to move right
      print("Bot takes a right");

    #If instruction is to stop
    if Movement == 's':

      #Print that bot has to stop
      print("Bot stops")
      return 0
    #Followed by a forward movement

    #If instruction is to take a 180 turn
    if Movement == 'U':

      #Print that bot turs 180 degress
      print("Bot turns 180")
    print("Forward motion till next node")
    print(" ")


'''
*
* Function name  :     CheckingAlgo()
* Input :              None
* Output :             None
* Logic :              Function to check if path planning algorithm is working correctly
* Example call :       CheckingAlgo()
*
'''


def CheckingAlgo() :

  #Loading the Instruction Set
  global ShortPath,Motions

  while (1) :

    #Taking values of source and destination
    Source  = input("Enter Source node : - ")
    Destination = input("Enter Destination node : -")
    print(" ")

   #Finding Shortest  path between nodes
    Path = FindShortestPath(Arena,Source,Destination)

   #Planning the path from the source to destination
    DecidePath(Path);

   #Printing the motions
    PrintMovements(ShortPath)

    #Deleting instruction set to make room for new instruction.
     #as 'f' is still required as the first instruction , leaving that at it is.
    del ShortPath[0:]



'''
*
* Function name  :     Planning
* Input :              None
* Output :             None
* Logic :              FUnction That comibines all the other functions and starts initialises the path planning
* Example call :        WhichNodeToGo
*
'''
def Planning():

    #Including globals
    global ShortPath,Motions

    #Just to know this function was called
    print("Initializing .. ")

    #Getting which node to start from
    StartNode = FromWhichNode()

    #Getting lists of locations of all the Ar_objects nodes, Axis of the pebble,pitcher and Cell Number od the pebble pitcher
    Destination,PebbleAxis,Pitcher,PebbleCellNo = WhichNodeToGo()

    #Total number of Ar_Objects to traverse to
    TotalLength = len(Destination)

    print("Total number of pebbles found are -", TotalLength)

    #Variables for looping
    Each = 0
    Check = 0
    print("Calculating all the requied paths (Debugging prints below)")
    #While Each is less than the number of ar objects (pebbles likely)
    while Each <= TotalLength:
        
        #If each is 0 i.e first instance of the loop (since Start To Pebble is only going to happen once)
        if Each == 0:

            #Traversing the bot to the first pebble, Check is the last node it visited
            Check = StartToPebble(Destination[Each],PebbleAxis[Each],StartNode,PebbleCellNo[Each])
            
        #For rest of the instances
        else :
            #print("value of check is ",Check)
            #Moving the bot from the pebble to the pitcher
            Check  = PebbleToPitcher(Check,Pitcher)

            #Logic so that Each index doesn't get out of range
            if Each != TotalLength:

              #Traversing from pitcher to pebble
              Check  = PitcherToPebble(Pitcher,Destination[Each],PebbleAxis[Each],PebbleCellNo[Each])
            
        #Incrementing the loop
        Each +=1


'''
*
* Function name  :    CheckingException
* Input :              Node - > Node where exception ocuured , Axis - Axis of the Ar_object pointing the node , Cell of the Ar_Object
* Output :             NewNode - > The alternative node
* Logic :              Whenever exceptions in the algrithm occur , i.e. some nodes are missing or are not available , This function will assign the other node of the axis
* Example call :        CheckingException('a',"1-1",10)
*
'''

def CheckingException(Node,Axis,Cell):
  
  NewNode = None
  if Robot_start == "START-1":

   #If axis "1-1"
   if Axis == "1-1":

    #If the center node of the node doesn't exist
    if Arena_path[Node][1] == 0:

      #Finding the other 1-1 node
      NewNode = FindDestination(Axis,Cell,"Invert") 

   #If axis is "2-2"
   if Axis == "2-2":

   #If the leftnode of the node doesn't exist
    if Arena_path[Node][0] == 0:

      #Finding the other 2-2 node
      NewNode = FindDestination(Axis,Cell,"Invert")

   #If axis is "3-3"
   if Axis == "3-3":

     #If the rightnode of the node doesn't exist
    if Arena_path[Node][2] == 0:

      #Finding the other 3-3 node
      NewNode = FindDestination(Axis,Cell,"Invert")

  elif Robot_start == "START-2":

    #If axis "1-1"
   if Axis == "1-1":

    #If the center node of the node doesn't exist
    if Arena_path[Node][1] == 0:

      #Finding the other 1-1 node
      NewNode = FindDestination(Axis,Cell,"Invert") 

   #If axis is "2-2"
   if Axis == "2-2":

   #If the leftnode of the node doesn't exist
    if Arena_path[Node][2] == 0:

      #Finding the other 2-2 node
      NewNode = FindDestination(Axis,Cell,"Invert")

   #If axis is "3-3"
   if Axis == "3-3":

     #If the rightnode of the node doesn't exist
    if Arena_path[Node][0] == 0:

      #Finding the other 3-3 node
      NewNode = FindDestination(Axis,Cell,"Invert")


  

  
  return NewNode

   
  


'''
*
* Function name  :    PebbleToPitcher
* Input :              Destination - > The node to go to , PebbleAxis - > Axis of the pebble , StartNode  - > The starting node, PebbleCellNo -> The Cell number of the peblle to reach to
* Output :             Destination - > The node currently at
* Logic :              FUnction traverses the bot from the start to the first pebble in the list
* Example call :        PebbleToPitcher()
*
'''
def StartToPebble(Destination,PebbleAxis,StartNode,PebbleCellNo):

  #Including the instruction set
   global ShortPath,Motions,Pebble1

   for each in arena_config:

       if arena_config[each][0] == "Pebble":

           if arena_config[each][1] == PebbleCellNo:
               Pebble1= each
               print(each)
          
       

   Mode = None
   Check = CheckingException(Destination,PebbleAxis,PebbleCellNo)
  
   #If new node was found
   if Check !=None:
     
     #The new destination is check
     Destination  = Check
     Mode = "Invert"
     
   

   #Finding What node to traverse to before the final one so that bot doesn't has to take extra turns
   Node = GetNode(PebbleAxis,Destination,Mode)






   #Finding Shortest Path till that node
   Path = FindShortestPath(Arena,StartNode,Node)


   #Traversing the bot according to the path
   DecidePath(Path)

   #Creating path for the final bot movement
   FinalPath = [Path[-1],Destination]

   #Deleting the last Stop command
   del ShortPath[-1]
   del Motions[-1]

   #Making the final bot movement
   DecidePath(FinalPath)
   del Motions[-1]
   Motions.append('p')

   #Printing the movements for debugging reasons
   PrintMovements(ShortPath)

   #Updating the starting node with the node just reached
   

   #Making room for next move
   del ShortPath[0:]

   #PitherToPebble(Each)
  # StartNode = Node


   #Printing the path
   #print("The path selected is ", Path)

   return Destination



'''
*
* Function name  :    PitcherToPebble
* Input :              CurrentNode - > The node currently at , Pitcher - > Information about pitcher i.e. Cell Nummber and axis
* Output :             PitcherNode - > The Node Currently at
* Logic :              Function traverses the bot from the pebble to the pitcher
* Example call :        PithcerToPebble('a',[0,'1-1'])
*
'''

def PebbleToPitcher(CurrentNode,Pitcher):

 global ShortPath,Motions

  #Pitcher[0] is the cell number
  #Pitcher[1] is the axis

  #Pitcher Position
 PitcherNode = FindDestination(Pitcher[1],Pitcher[0])

 Mode = None

 #Checking for exceptions
 Check = CheckingException(PitcherNode,Pitcher[1],Pitcher[0])
  
 #If new node was found
 if Check !=None:
     
   #The new destination is check
   PitcherNode  = Check
   Mode = "Invert"
     

 
 

#Finding What node to traverse to before the final one so that bot doesn't has to take extra turns
 Node = GetNode(Pitcher[1],PitcherNode,Mode)

 #Findinf the shortest path till that node
 Path = FindShortestPath(Arena,CurrentNode,Node)

 #Traversing bot according to the path
 DecidePath(Path)

 #Creating path for the final bot movement
 FinalPath = [Path[-1],PitcherNode]
 
 #Deleteing the last stop Command
 del ShortPath[-1]
 del Motions[-1]

 #Making the final bot movement
 DecidePath(FinalPath)
 del Motions[-1]
 Motions.append('P')
 

 #Printing the movements for debugging reasons
 PrintMovements(ShortPath)

 

 #Making room for next move
 del ShortPath[0:]
 
 return PitcherNode



'''
*
* Function name  :     PitcherToPebble
* Input :              Pitcher - > information about the pitcher, Destination - > The node to reach to, Axis - > Axis of the pebble , PebbleCellNo - > Cell number of the pebble
* Output :             Destination - > The node bot currently at
* Logic :              FUnction traverses the bot from pitcher to the next pebble
* Example call :        PitcherTopebble
*
'''
def PitcherToPebble(Pitcher,Destination,Axis,PebbleCellNo):

 global ShortPath,Motions,PebbleSeq

  #Pitcher[0] is the cell number
  #Pitcher[1] is the axis

 for each in arena_config:

       if arena_config[each][0] == "Pebble":

           arena_config[each][1] == PebbleCellNo
           PebbleSeq.append(each)
  #Extra checks
 Mode = None
 Check = CheckingException(Destination,Axis,PebbleCellNo)
  
   #If new node was found
 if Check !=None:
     
     #The new destination is check
     Destination  = Check
     Mode = "Invert"


 #Pitcher Position
 PitcherNode = FindDestination(Pitcher[1],Pitcher[0])

 #Finding what node to traverse to before the final one sp that the bot doesn't has to take extra turns
 Node = GetNode(Axis,Destination,Mode)



 #Finding the shortestpath till that node
 Path = FindShortestPath(Arena,PitcherNode,Node)

 #Traversing bot according to the path
 DecidePath(Path)

 #Creating Path for final bot movement
 FinalPath = [Path[-1],Destination]

 #Deleting the last stop Command
 del ShortPath[-1]
 del Motions[-1]
 #Making the final bot movement
 DecidePath(FinalPath)

 del Motions[-1]
 Motions.append('O')

 #Printing the movements for debugging reasons
 
 
 
 
 PrintMovements(ShortPath)

 

 #Making room for next move
 del ShortPath[0:]

 return Destination
 

'''
*
* Function name  :     m()
* Input :              None
* Output :             Prints the final instruction set
* Logic :              Calls the required functions to produce final instruction set
* Example call :       m()
*
'''

def m():
  
 global ShortPath,Motions
 


#Bot's path planning
 Planning()

 print("Motions - ",Motions)

 #Converting the final Instruction set into readable form according to the Embedded C code 

 for Entry in range(len(Motions)):
    if Entry < len(Motions)-1:
     if Motions[Entry+1] == 'p' or Motions[Entry+1] == 'P'or Motions[Entry+1] == 'O':
         if Motions[Entry] == 'r':
             Motions[Entry] = 'R'
         elif Motions[Entry] == 'l':
             Motions[Entry] = 'L'
         elif Motions[Entry] == 'U':
             Motions[Entry] = 'u'
 

 for Entry in range(len(Motions)):

     if Entry < len(Motions)-1:
         if Motions[Entry] == Motions[Entry+1] :

             if Motions[Entry] == 'l':
                 Motions[Entry] = 'k'

             elif Motions[Entry] == 'r':
                 Motions[Entry] = 't'

             elif Motions[Entry] == 'u':
                 Motions[Entry] = 'i'
                 
 
 Motions.append('S')
 









###############################################################################

############# OpenGL related functions ########################


"""
Function Name : getCameraMatrix()
Input: None
Output: camera_matrix, dist_coeff
Purpose: Loads the camera calibration file provided and returns the camera and
         distortion matrix saved in the calibration file.
"""
def getCameraMatrix():
        global camera_matrix, dist_coeff
        with np.load('Camera.npz') as X:
                camera_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]


"""
Function Name : main()
Input: None
Output: None
Purpose: Initialises OpenGL window and callback functions. Then starts the event
         processing loop.
"""        
def main():
       

      
                
                glutInit()
                getCameraMatrix()
                glutInitWindowSize(640, 480)
                glutInitWindowPosition(625, 100)
                glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE)
                window_id = glutCreateWindow("OpenGL")
                init_gl()
                glutDisplayFunc(drawGLScene)
                glutIdleFunc(drawGLScene)
                glutReshapeFunc(resize)
                glutMainLoop()

                
                
        
        
        

"""
Function Name : init_gl()
Input: None
Output: None
Purpose: Initialises various parameters related to OpenGL scene.
"""  
def init_gl():
        global texture_object, texture_background
        global crow
        global rock
        global emptypot
        global halfpot
        global fullpot
        global dimnishedrock
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0) 
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)   
        glMatrixMode(GL_MODELVIEW)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        texture_background = glGenTextures(1)
        texture_object = glGenTextures(1)
        crow = OBJ('crow.obj', swapyz=True)
        dimnishedrock = OBJ('dimnishedrock.obj', swapyz=True)
        halfpot = OBJ('halfpot.obj', swapyz=True)
        emptypot = OBJ('emptypot.obj', swapyz=True)
        fullpot = OBJ('fullpot.obj', swapyz=True)
        rock = OBJ('rock.obj', swapyz=True)
"""
Function Name : resize()
Input: None
Output: None
Purpose: Initialises the projection matrix of OpenGL scene
"""
def resize(w,h):
        ratio = 1.0* w / h
        glMatrixMode(GL_PROJECTION)
        glViewport(0,0,w,h)
        gluPerspective(45, ratio, 0.1, 100.0)

"""
Function Name : drawGLScene()
Input: None
Output: None
Purpose: It is the main callback function which is called again and
         again by the event processing loop. In this loop, the webcam frame
         is received and set as background for OpenGL scene. ArUco marker is
         detected in the webcam frame and 3D model is overlayed on the marker
         by calling the overlay() function.
"""
def drawGLScene():

       global Index,Data,Pebble1Changed,PebbleChoose,PitcherChanged,AlreadyDimnished
       glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
       ar_list = []
       ret, frame = cap.read()

       #Checking if any data has been sent from the bot (i.e. the bot has been turned on)
       Data = Receive2()
       #Decoding the data received
       Data = Data.decode('utf-8')
       #print("Data", Data)
       if Data:
           
           print("Data received  - " , Data)
           print("Number of instruction at currently " , Index)

       #If the bot started from START and has reached the first pebble
       if Data == 'q':  

           #On bot Picking up pebble change the Pebble Object            
           Pebble1Changed = True

       #If the bot reached to a pitcher to drop pebble
       elif Data == 'w':

           #Change pitcher Object
           PitcherChanged +=1

       #If bot dropped a pebble and reached to next pebble
       elif Data == 'e':

           #Stroing Already Dimnished Pebble Aruco ids
           AlreadyDimnished.append(PebbleSeq[PebbleChoose])

           #Incrementing the number of pebbles that are dimnished
           PebbleChoose+=1
           
       #If the very first command is to be send ( M is received when bot is turned on)
       if Index == 0 and Data =='M'  :

          #Sending the command
          Send(Motions[Index])
          print("Sent Data", Motions[Index])

          #Incrementing the number of Commands Send
          Index = 1

          #Checking if any data is received
          Data = Receive2()
          Data = Data.decode('utf-8')
       
       #Sending more Commands till end of Instruction Set (Motions)
       #Also checking if Data received was an Actual data not a flag
       elif Index > 0 and Index < len(Motions):
          
          #Checking if Some Data was actually received
          if Data:
              if Data not in ExceptList:

            #If received then Send next Command
               #print("Data checked was ",Data)
               Send(Motions[Index])
               print("Sent Data ", Motions[Index])

               #Increment the Index for Command
               Index +=1
           
 

       
 
       if ret == True:
                draw_background(frame)
                
                
                glMatrixMode(GL_MODELVIEW)
                glLoadIdentity()
                ar_list = detect_markers(frame)
                for i in ar_list:
                        
                                
                 
                               
                                        overlay(frame, ar_list, i[0],"texture_1.png")
                                
                       
                                
                                
                aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
                parameters = aruco.DetectorParameters_create()
                corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
                frame  = aruco.drawDetectedMarkers(frame, corners,ids)
                cv2.imshow('frame', frame)
                cv2.waitKey(1)
                
         
       glutSwapBuffers()
        
########################################################################

######################## Aruco Detection Function ######################
"""
Function Name : detect_markers()
Input: img (numpy array)
Output: aruco list in the form [(aruco_id_1, centre_1, rvec_1, tvec_1),(aruco_id_2,
        centre_2, rvec_2, tvec_2), ()....]
Purpose: This function takes the image in form of a numpy array, camera_matrix and
         distortion matrix as input and detects ArUco markers in the image. For each
         ArUco marker detected in image, paramters such as ID, centre coord, rvec
         and tvec are calculated and stored in a list in a prescribed format. The list
         is returned as output for the function
"""
def detect_markers(img):

  #Defining the markers length
 markerLength = 100

 #List to store the aruco list
 aruco_list =[]

 #List to store aruco Ids
 ids = []

 #Findind Ids Cormers of aruco detected
 gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
 parameters = aruco.DetectorParameters_create()
 corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters= parameters)
 
 #Logic to get rid of the None Type Error
 count = 0
 for x in corners:
     count +=1
 
 #List to hold ids seperated 
 Id = [] 

 if count > 1:
     for x in ids:
         for y in x:
               Id.append(y)

     # Since center is not required for this algorithm
     center = []  


     #Listt to hold rvecs and tvecs
     rvec =[]
     tvec = []

     #Initialsing the rvec and tvec list with temporary values
     for x in range(len(Id)):
         rvec.append(x)
         tvec.append(x)
         center.append(x)
         
     
     #Finding rvecs and tvces and storing them into lists
     for x in range(len(Id)):
      rvec[x], tvec[x],_ = aruco.estimatePoseSingleMarkers(corners[x], markerLength, camera_matrix, dist_coeff)
      
      #Error correction
      (rvec[x]-tvec[x]).any()
 
     #Combing the information 
     aruco_list = zip(Id,center,rvec,tvec)

     #Creating a list of tuples
     aruco_list = list(aruco_list)

 
 
        
 ##################################################################
 return aruco_list


########################################################################


################# This is where the magic happens !! ###################
############### Complete these functions as  directed ##################
"""
Function Name : draw_background()
Input: img (numpy array)
Output: None
Purpose: Takes image as input and converts it into an OpenGL texture. That
         OpenGL texture is then set as background of the OpenGL scene
"""
def draw_background(img):

 bg_image = cv2.flip(img,0)
 bg_image = Image.fromarray(bg_image)
 ix = bg_image.size[0]
 iy = bg_image.size[1]
 bg_image = bg_image.tobytes("raw","BGRX",0,-1)
 glEnable(GL_TEXTURE_2D)
 glBindTexture(GL_TEXTURE_2D,texture_background)

 glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, bg_image)
 glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
 glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
 glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
 glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

 glBegin(GL_QUADS)
 glTexCoord2f(0.0,1.0); glVertex3f(-5.0, -3.7, -9.0)
 glTexCoord2f(1.0,1.0); glVertex3f(5.0, -3.7, -9.0)
 glTexCoord2f(1.0,0.0); glVertex3f(5.0, 3.7, -9.0)
 glTexCoord2f(0.0,0.0); glVertex3f(-5.0, 3.7, -9.0)
 glEnd()
 glPushMatrix()
 
 
 glPopMatrix()




 
 return None

"""
Function Name : init_object_texture()
Input: Image file path
Output: None
Purpose: Takes the filepath of a texture file as input and converts it into OpenGL
         texture. The texture is then applied to the next object rendered in the OpenGL
         scene.
"""
def init_object_texture(image_filepath):
 tex = cv2.imread(image_filepath)
 bg_image = cv2.flip(tex,0)
 bg_image = Image.fromarray(bg_image)
 ix = bg_image.size[1]
 iy = bg_image.size[0]
 bg_image = bg_image.tobytes("raw","BGRX",0,-1)
 glEnable(GL_TEXTURE_2D)
 glBindTexture(GL_TEXTURE_2D,texture_object)

 glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, bg_image)
 glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
 glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
 glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
 glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)


 glPushMatrix()
 
 
 glPopMatrix()
        
 return None

"""
Function Name : overlay()
Input: img (numpy array), aruco_list, aruco_id, texture_file (filepath of texture file)
Output: None
Purpose: Receives the ArUco information as input and overlays the 3D Model of a teapot
         on the ArUco marker. That ArUco information is used to
         calculate the rotation matrix and subsequently the view matrix. Then that view matrix
         is loaded as current matrix and the 3D model is rendered.

         Parts of this code are already completed, you just need to fill in the blanks. You may
         however add your own code in this function.
"""
def overlay(img, ar_list, ar_id, texture_file):
        
        for x in ar_list:
                if ar_id == x[0]:
                        centre, rvecs, tvecs = x[1], x[2], x[3]
        
        
        rmtx = cv2.Rodrigues(rvecs)[0]
        view_matrix = np.array([[rmtx[0][0],rmtx[0][1],rmtx[0][2],tvecs[0][0][0]/200],
                                [rmtx[1][0],rmtx[1][1],rmtx[1][2],tvecs[0][0][1]/200],
                                [rmtx[2][0],rmtx[2][1],rmtx[2][2],tvecs[0][0][2]/300],
                                [0.0       ,0.0       ,0.0       ,1.0   ]])
        view_matrix = view_matrix * INVERSE_MATRIX
        view_matrix = np.transpose(view_matrix)

        #print("Aruco list is ##############################",ar_list)
        
       #init_object_texture(texture_file)
        glPushMatrix()
        glLoadMatrixd(view_matrix)
        #glutSolidTeapot(0.5)
        #glCallList(crow.gl_list)
        #if ar_list == 1:
        if ar_id == 10:
            glCallList(crow.gl_list)
        #If the aruco  of pitcher is in the frame
        if ar_id == PitcherId:
            
            #If no pebble has been droped to the Pitcher
            if PitcherChanged == 0:   

                #Show a empty pitcher 
                glCallList(emptypot.gl_list)

            #If one or more than one Pebble was dropped in the pitcher
            elif PitcherChanged > 0 and PitcherChanged < NumberOfPebbles:

                 #Show half full pot
                 glCallList(halfpot.gl_list)

            #If All the Pebbles are dropped in the pitcher
            elif PitcherChanged == NumberOfPebbles:

                 #Show Full POt
                 glCallList(fullpot.gl_list)
               
        #If aruco of  First Pebble is in the frame
        if ar_id == Pebble1:

            #If The first pebble has not been picked up
            if Pebble1Changed == False:

                #Show original rock
                 glCallList(rock.gl_list)

           #If the first pebble was picked up
            else :
                 
                 #Show first pebble as dimnished
                 glCallList(dimnishedrock.gl_list)
            
        #If aruco of all pebbles except for first one is detect in the frame
        if ar_id in PebbleIds:

           #If no pebble has been picked up (Dont know about the first one)
            if PebbleChoose < 1:
            
                   #Show all of the pebbles as Original (Dont know about the first one)
                   glCallList(rock.gl_list)

            #If any pebble was picked up 
            else :
                   #If the pebble picked up is in frame  or the pebble earlier picked up are in frame
                   if ar_id == PebbleSeq[PebbleChoose]  :
                    #Show them as dimnished
                        
                            
                        glCallList(dimnishedrock.gl_list)
                   elif ar_id in AlreadyDimnished:

                        glCallList(dimnishedrock.gl_list)

                   #else show them as origianl pebble
                   else :
                       
                        glCallList(rock.gl_list)
                       
                
        glPopMatrix()
        

'''
*
* Function name  :     FindId
* Input :              None
* Output :             None
* Logic :              This function finds and stores aruco ids of pebble and pitcher in lists and also number of pebbles
* Example call :       FindId()
*
'''     


def FindId():
    global PitcherId, PebbleIds,NumberOfPebbles,Pebble1
    NumberOfPebbles = 0;

    #For each Key in dictionary
    for each in arena_config:
        
        #If the 0th index of a key has watcher pitcher in it
        if(arena_config[each][0] == "Water Pitcher"):

           #Get the aruco id of the water pitcher
            PitcherId = each
        
        #If the 0th index of a key has Pebble in it 
        if(arena_config[each][0] == "Pebble"):

          #Get the aruco id of pebble (list as more than one pebble is possible)
           PebbleIds.append(each)
           #Incresing the number of pebbles
           NumberOfPebbles+=1
    
    #Deleting first pebble as it is stored on Pebbel1 variable seprately
    del PebbleIds[0]    
    
#--------------------------------------------------------------------------------------------------------------------------------#


#Initialising Algorithm
m()

#initialising required function before opening OpenGl Window
FindId()

#Opening openGl Window
main()

