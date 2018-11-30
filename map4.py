import time
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import RPi.GPIO as GPIO
import Adafruit_PCA9685
import signal
import math
import random


LSHDN = 27
FSHDN = 22
RSHDN = 23
LSERVO = 0
RSERVO = 1
LENCODER = 17
RENCODER = 18
l = 0
r = 0
distance_of_cell = 18
revolutions_per_cell = 17.9 / 8.19956
ticks_per_cell = revolutions_per_cell * 32
orientation = "North"
current_cell = 13
visit = []
visit.append(current_cell)
maze = [["Wall cell form: west, north, east, south", "Visit status"],
        ["1 ->", "w", "w", "u", "u", "NotVis"],
        ["2 ->", "u", "w", "u", "u", "NotVis"],
        ["3 ->", "u", "w", "u", "u", "NotVis"],
        ["4 ->", "u", "w", "w", "u", "NotVis"],
        ["5 ->", "w", "u", "u", "u", "NotVis"],
        ["6 ->", "u", "u", "u", "u", "NotVis"],
        ["7 ->", "u", "u", "u", "u", "NotVis"],
        ["8 ->", "u", "u", "w", "u", "NotVis"],
        ["9 ->", "w", "u", "u", "u", "NotVis"],
        ["10 ->", "u", "u", "u", "u", "NotVis"],
        ["11 ->", "u", "u", "u", "u", "NotVis"],
        ["12 ->", "u", "u", "w", "u", "NotVis"],
        ["13 ->", "w", "u", "u", "w", "NotVis"],
        ["14 ->", "u", "u", "u", "w", "NotVis"],
        ["15 ->", "u", "u", "u", "w", "NotVis"],
        ["16 ->", "u", "u", "w", "w", "NotVis"]]

DEFAULTADDR = 0x29
LADDR = 0x2a
RADDR = 0x2b

def ctrlC(signum, frame):
    print("Exiting")
    
    # Stop the servos
    pwm.set_pwm(LSERVO, 0, 0);
    pwm.set_pwm(RSERVO, 0, 0);
    
    exit()

def onLeftEncode(pin):
    global l
    l+=1

def onRightEncode(pin):
    global r
    r+=1
    
def clear():
    global l
    global r
    l = 0
    r = 0

def status(c):
    global visit
    global maze
    if c in visit:
        maze[c][5] = "Vis"

def wall(ldist, fdist, rdist, orient, cell):
    global maze
    if orient == "West":
        if fdist < 430:
            maze[cell][1] = 1
        else:
            maze[cell][1] = 0
        if ldist < 430:
            maze[cell][4] = 1
        else:
            maze[cell][4] = 0
        if rdist < 430:
            maze[cell][2] = 1
        else:
            maze[cell][2] = 0
    if orient == "East":
        if fdist < 430:
            maze[cell][3] = 1
        else:
            maze[cell][3] = 0
        if ldist < 430:
            maze[cell][2] = 1
        else:
            maze[cell][2] = 0
        if rdist < 430:
            maze[cell][4] = 1
        else:
            maze[cell][4] = 0
    if orient == "North":
        if fdist < 430:
            maze[cell][2] = 1
        else:
            maze[cell][2] = 0
        if ldist < 430:
            maze[cell][1] = 1
        else:
            maze[cell][1] = 0
        if rdist < 430:
            maze[cell][3] = 1
        else:
            maze[cell][3] = 0
    if orient == "South":
        if fdist < 430:
            maze[cell][4] = 1
        else:
            maze[cell][4] = 0
        if ldist < 430:
            maze[cell][3] = 1
        else:
            maze[cell][3] = 0
        if rdist < 430:
            maze[cell][1] = 1
        else:
            maze[cell][1] = 0

def traverse_cells(o, c):
	global current_cell
	global maze
	if (current_cell == 1) and (o == "South"):
		current_cell = 5
		print(current_cell)
	elif (current_cell == 1) and (o == "East"):
		current_cell = 2
		print(current_cell)
	elif (current_cell == 2) and (o == "West"):
		current_cell = 1
		print(current_cell)
	elif (current_cell == 2) and (o == "South"):
		current_cell = 6
		print(current_cell)
	elif (current_cell == 2) and (o == "East"):
		current_cell = 3
		print(current_cell)
	elif (current_cell == 3) and (o == "West"):
		current_cell = 2
		print(current_cell)
	elif (current_cell == 3) and (o == "South"):
		current_cell = 7
		print(current_cell)
	elif (current_cell == 3) and (o == "East"):
		current_cell = 4
		print(current_cell)
	elif (current_cell == 4) and (o == "West"):
		current_cell = 3
		print(current_cell)
	elif (current_cell == 4) and (o == "South"):
		current_cell = 8
		print(current_cell)
	elif (current_cell == 5) and (o == "North"):
		current_cell = 1
		print(current_cell)
	elif (current_cell == 5) and (o == "East"):
		current_cell = 6
		print(current_cell)
	elif (current_cell == 5) and (o == "South"):
		current_cell = 9
		print(current_cell)
	elif (current_cell == 6) and (o == "East"):
		current_cell = 5
		print(current_cell)
	elif (current_cell == 6) and (o == "North"):
		current_cell = 2
		print(current_cell)
	elif (current_cell == 6) and (o == "West"):
		current_cell = 7
		print(current_cell)
	elif (current_cell == 6) and (o == "South"):
		current_cell = 10
		print(current_cell)
	elif (current_cell == 7) and (o == "East"):
		current_cell = 8
		print(current_cell)
	elif (current_cell == 7) and (o == "North"):
		current_cell = 3
		print(current_cell)
	elif (current_cell == 7) and (o == "West"):
		current_cell = 6
		print(current_cell)
	elif (current_cell == 7) and (o == "South"):
		current_cell = 11
		print(current_cell)
	elif (current_cell == 8) and (o == "North"):
		current_cell = 4
		print(current_cell)
	elif (current_cell == 8) and (o == "West"):
		current_cell = 7
		print(current_cell)
	elif (current_cell == 8) and (o == "South"):
		current_cell = 12
		print(current_cell)
	elif (current_cell == 9) and (o == "North"):
		current_cell = 5
		print(current_cell)
	elif (current_cell == 9) and (o == "East"):
		current_cell = 10
		print(current_cell)
	elif (current_cell == 9) and (o == "South"):
		current_cell = 13
		print(current_cell)
	elif (current_cell == 10) and (o == "East"):
		current_cell = 11
		print(current_cell)
	elif (current_cell == 10) and (o == "North"):
		current_cell = 6
		print(current_cell)
	elif (current_cell == 10) and (o == "West"):
		current_cell = 9
		print(current_cell)
	elif (current_cell == 10) and (o == "South"):
		current_cell = 14
		print(current_cell)
	elif (current_cell == 11) and (o == "North"):
		current_cell = 7
		print(current_cell)
	elif (current_cell == 11) and (o == "West"):
		current_cell = 10
		print(current_cell)
	elif (current_cell == 11) and (o == "South"):
		current_cell = 15
		print(current_cell)
	elif (current_cell == 11) and (o == "East"):
		current_cell = 12
		print(current_cell)
	elif (current_cell == 12) and (o == "West"):
		current_cell = 11
		print(current_cell)
	elif (current_cell == 12) and (o == "South"):
		current_cell = 16
		print(current_cell)
	elif (current_cell == 12) and (o == "North"):
		current_cell = 8
		print(current_cell)
	elif (current_cell == 13) and (o == "North"):
		current_cell = 9
		print(current_cell)
	elif (current_cell == 13) and (o == "East"):
		current_cell = 14
		print(current_cell)
	elif (current_cell == 14) and (o == "West"):
		current_cell = 13
		print(current_cell)
	elif (current_cell == 14) and (o == "North"):
		current_cell = 10
		print(current_cell)
	elif (current_cell == 14) and (o == "East"):
		current_cell = 15
		print(current_cell)
	elif (current_cell == 15) and (o == "West"):
		current_cell = 14
		print(current_cell)
	elif (current_cell == 15) and (o == "North"):
		current_cell = 11
		print(current_cell)
	elif (current_cell == 15) and (o == "East"):
		current_cell = 16
		print(current_cell)
	elif (current_cell == 16) and (o == "West"):
		current_cell = 15
		print(current_cell)
	elif (current_cell == 16) and (o == "North"):
		current_cell = 12
		print(current_cell)

signal.signal(signal.SIGINT, ctrlC)
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

GPIO.setmode(GPIO.BCM)

GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));

GPIO.add_event_detect(LENCODER, GPIO.RISING, onLeftEncode)
GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)

GPIO.setmode(GPIO.BCM)
GPIO.setup(LSHDN, GPIO.OUT)
GPIO.setup(FSHDN, GPIO.OUT)
GPIO.setup(RSHDN, GPIO.OUT)

GPIO.output(LSHDN, GPIO.LOW)
GPIO.output(FSHDN, GPIO.LOW)
GPIO.output(RSHDN, GPIO.LOW)
time.sleep(0.01)


lSensor = VL53L0X.VL53L0X(address=LADDR)
fSensor = VL53L0X.VL53L0X(address=DEFAULTADDR)
rSensor = VL53L0X.VL53L0X(address=RADDR)


GPIO.output(LSHDN, GPIO.HIGH)
time.sleep(0.01)
lSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)


GPIO.output(RSHDN, GPIO.HIGH)
time.sleep(0.01)
rSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Connect the front sensor and start measurement
GPIO.output(FSHDN, GPIO.HIGH)
time.sleep(0.01)
fSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

def pivot_180():
    global orientation
    global l
    global r
    clear()
    while l < 26:
        pwm.set_pwm(LSERVO, 0, math.floor(1.55 / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(1.55 / 20 * 4096));
        time.sleep(0.1)
    if orientation == "East":
        orientation = "West"
    elif orientation == "West":
        orientation = "East"
    elif orientation == "North":
        orientation = "South"
    elif orientation == "South":
        orientation = "North"
    print(orientation)
    clear()
    
def pivot_left():
    global orientation
    global l
    global r
    clear()
    while l < 13:
        pwm.set_pwm(LSERVO, 0, math.floor(1.45 / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(1.45 / 20 * 4096));
        time.sleep(0.1)
    if orientation == "East":
        orientation = "North"
    elif orientation == "West":
        orientation = "South"
    elif orientation == "North":
        orientation = "West"
    elif orientation == "South":
        orientation = "East"
    print(orientation)
    clear()

def pivot_right():
    global orientation
    global l
    global r
    clear()
    while l < 13:
        pwm.set_pwm(LSERVO, 0, math.floor(1.55 / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(1.55 / 20 * 4096));
        time.sleep(0.1)
    if orientation == "East":
        orientation = "South"
    elif orientation == "West":
        orientation = "North"
    elif orientation == "North":
        orientation = "East"
    elif orientation == "South":
        orientation = "West"
    print(orientation)
    clear()

for count in range(1, 10000):
    lDistance = lSensor.get_distance()
    fDistance = fSensor.get_distance()
    rDistance = rSensor.get_distance()
    rD = []
    lD = []
    fD = []
    rmini = 8192
    lmini = 8192
    status(current_cell)
    wall(lDistance, fDistance, rDistance, orientation, current_cell)
    print(maze)
    traverse_cells(orientation, current_cell)
    if l >= ticks_per_cell:
        lDistance = lSensor.get_distance()
        fDistance = fSensor.get_distance()
        rDistance = rSensor.get_distance()
        status(current_cell)
        wall(lDistance, fDistance, rDistance, orientation, current_cell)
        print(maze)
        traverse_cells(orientation, current_cell)
        pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
        time.sleep(1)
        """for i in range(50):
            pwm.set_pwm(LSERVO, 0, math.floor(1.485 / 20 * 4096));
            pwm.set_pwm(RSERVO, 0, math.floor(1.485 / 20 * 4096));
            rDistance = rSensor.get_distance()
            lDistance = lSensor.get_distance()
            fDistance = fSensor.get_distance()
            rD.append(rDistance)
            lD.append(lDistance)
            fD.append(fDistance)
            rmini = min(rD)
            lmini = min(lD)
            fmini = min(fD)
            time.sleep(0.02)
        for j in range(60):
            pwm.set_pwm(LSERVO, 0, math.floor(1.515 / 20 * 4096));
            pwm.set_pwm(RSERVO, 0, math.floor(1.515 / 20 * 4096));
            rDistance = rSensor.get_distance()
            lDistance = lSensor.get_distance()
            fDistance = fSensor.get_distance()
            rD.append(rDistance)
            lD.append(lDistance)
            fD.append(fDistance)
            rmini = min(rD)
            lmini = min(lD)
            fmini = min(fD)
            time.sleep(0.02)
        ultamini = min(rmini, lmini, fmini)
        if ultamini == lmini:
            while lDistance > (ultamini + 5):
                pwm.set_pwm(LSERVO, 0, math.floor(1.485 / 20 * 4096));
                pwm.set_pwm(RSERVO, 0, math.floor(1.485 / 20 * 4096));
                time.sleep(0.02)
                lDistance = lSensor.get_distance()
        elif ultamini == rmini:
            while rDistance > (5 + ultamini):
                pwm.set_pwm(LSERVO, 0, math.floor(1.485 / 20 * 4096));
                pwm.set_pwm(RSERVO, 0, math.floor(1.485 / 20 * 4096));
                time.sleep(0.02)
                rDistance = rSensor.get_distance()
        else:
            while fDistance > (5 + ultamini):
                pwm.set_pwm(LSERVO, 0, math.floor(1.485 / 20 * 4096));
                pwm.set_pwm(RSERVO, 0, math.floor(1.485 / 20 * 4096));
                time.sleep(0.02)
                fDistance = fSensor.get_distance()"""
        clear()
        if (fDistance <= 350) and (rDistance <= 350) and (lDistance <= 350):
            pivot_180()
        
        elif (lDistance > 500) and (fDistance <= 380) and (rDistance <= 380):
            pivot_left()
        
        elif (rDistance > 500) and (fDistance <= 380) and (lDistance <= 380):
            pivot_right()
        
        elif (lDistance < 380) and (rDistance < 380) and (fDistance > 430):
            pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
            pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
            time.sleep(1)
            clear()
        
        elif (lDistance < 380) and (fDistance > 430) and (rDistance > 430):
            o = random.randint(0,1)
            if o == 0:
                pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
                pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
                time.sleep(1)
                clear()
            elif o == 1:
                pivot_right()
        
        elif (fDistance < 380) and (lDistance > 430) and (rDistance > 430):
            o = random.randint(0,1)
            if o == 0:
                pivot_left()
            elif o == 1:
                pivot_right()
                
        elif (rDistance < 380) and (fDistance > 430) and (lDistance > 430):
            o = random.randint(0,1)
            if o == 0:
                pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
                pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
                time.sleep(0.92)
                clear()
            elif o == 1:
                pivot_left()
        
                
    else:
        fDistance = fSensor.get_distance()
        if fDistance < 200:
            l = 1000
        elif rDistance < 199:
            pwm.set_pwm(LSERVO, 0, math.floor(1.53 / 20 * 4096));
            pwm.set_pwm(RSERVO, 0, math.floor(1.46 / 20 * 4096));
            time.sleep(0.1)
        elif lDistance < 199:
            pwm.set_pwm(LSERVO, 0, math.floor(1.5415 / 20 * 4096));
            pwm.set_pwm(RSERVO, 0, math.floor(1.47 / 20 * 4096));
            time.sleep(0.1)
        else:
            pwm.set_pwm(LSERVO, 0, math.floor(1.54 / 20 * 4096));
            pwm.set_pwm(RSERVO, 0, math.floor(1.455 / 20 * 4096));
            time.sleep(0.1)
        


lSensor.stop_ranging()
fSensor.stop_ranging()
rSensor.stop_ranging()
