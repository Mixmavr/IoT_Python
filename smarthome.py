from machine import Pin, ADC, PWM, I2C
from time import sleep, time
import ssd1306
from hcsr04 import HCSR04
import dht

#GPIOs
PIN_LED1 = 0 
PIN_LED2 = 32
PIN_RAIN = 16
PIN_MOTOR = 17
PIN_BUTTON = 5
PIN_TILT = 26
PIN_LDR = 34 
PIN_POT = 36
PIN_RED = 4
PIN_GREEN = 33
PIN_BLUE = 12
PIN_DOOR = 23
PIN_WIN = 25
PIN_SCL = 22
PIN_SDA = 21
PIN_TRIG = 13
PIN_ECHO = 14
PIN_DHT = 27

#Constants
DOOR_OPEN = 25
DOOR_CLOSED = 125
WIN_OPEN = 125
WIN_CLOSED = 25
DHT_READ = 3
SENSORS_READ = 0.5
FAN_TEMP = 27 #Temp threshold
#Outputs
led1 = Pin(PIN_LED1, Pin.OUT)
led1.value(0)
led2 = Pin(PIN_LED2, Pin.OUT)
led2.value(0)
red = PWM(Pin(PIN_RED), 5000, 0)
red.duty(0)
green = PWM(Pin(PIN_GREEN), 5000, 0)
blue = PWM(Pin(PIN_BLUE), 5000, 0)
fanTrans = Pin(PIN_MOTOR, Pin.OUT)
fanTrans.value(0)
door=PWM(Pin(PIN_DOOR),50)
door.duty(DOOR_CLOSED)
win=PWM(Pin(PIN_WIN),50)
win.duty(WIN_CLOSED)
i2c = I2C(-1, scl=Pin(PIN_SCL), sda=Pin(PIN_SDA))
oled = ssd1306.SSD1306_I2C(128, 64, i2c) #width/height

#Inputs
rainSensor = Pin(PIN_RAIN, Pin.IN)
tilt = Pin(PIN_TILT, Pin.IN, Pin.PULL_UP)
button = Pin(PIN_BUTTON, Pin.IN, Pin.PULL_UP)
pot = ADC(Pin(PIN_POT))
pot.atten(ADC.ATTN_11DB) #full range voltage 3.3V
pot.width(ADC.WIDTH_10BIT) #range 0 to 1023
ultrasonic = HCSR04(PIN_TRIG, PIN_ECHO)
dht11 = dht.DHT11(Pin(PIN_DHT))

#global vars
doorState = 0
winState = 0
distance = 0
temperature = 0
humidity = 0
potValue = 0
lastDHTRead = 0
lastSensorsRead = 0
textList = []

def controlRGB(hexStr):
    redValue = hexStr[0] + hexStr[1]
    redValue = int(redValue, 16)
    redValue = int((redValue/255)*1023)
    greenValue = hexStr[2] + hexStr[3]
    greenValue = int(greenValue, 16)
    greenValue = int((greenValue/255)*1023)
    blueValue = hexStr[4] + hexStr[5]
    blueValue = int(blueValue, 16)
    blueValue = int((blueValue/255)*1023)
    red.duty(redValue)
    green.duty(greenValue)
    blue.duty(blueValue)

doorState = 0
door.duty(DOOR_CLOSED)
winState = 0
win.duty(WIN_CLOSED)

def controlLed(ledNumber, state):
    if ledNumber == 0:
        led1.value(state)
        led2.value(state)
    elif ledNumber == 1:
        led1.value(state)
    elif ledNumber == 2:
        led2.value(state)

def controlDOOR(state):
    global doorState
    if state == 0:
        door.duty(DOOR_CLOSED)
        doorState=0
    if state == 1:
        door.duty(DOOR_OPEN)
        doorState=1
        
def controlWINDOW(stateWin):
    global windowState
    if stateWin == 0:
        win.duty(WINDOW_CLOSED)
        winState=0
    if stateWin == 1:
        win.duty(WINDOW_OPEN)
        winState=1
        
def openDoor():
    global doorState
    
    door.duty(DOOR_OPEN)
    doorState = 1
    
def closeDoor():
    global doorState
    
    door.duty(DOOR_CLOSED)
    doorState = 0
    
def openWin():
    global winState
    
    win.duty(WIN_OPEN)
    winState = 1
    
def closeWin():
    global winState
    
    win.duty(WIN_CLOSED)
    winState = 0

        
def mapRange(var, firstMin, firstMax, secondMin, secondMax):
    newVar = secondMin + var*((secondMax-secondMin)/(firstMax-firstMin))
    return int(newVar)

def displayOLED(textList):
    oled.fill(0)
    height = 0
    for line in textList:
         oled.text(line, 0,height)
         height+=10
    oled.show()
    
def readSensors():
    global distance, temperature, humidity, potValue, lastDHTRead, lastSensorsRead
    global DHT_READ, SENSORS_READ
    
    if time()-lastDHTRead > DHT_READ:
        dht11.measure()
        temperature = dht11.temperature()
        humidity = dht11.humidity()
        lastDHTRead = time()
    
    if time()-lastSensorsRead > SENSORS_READ:
        distance = ultrasonic.distance_cm()
        potValue = pot.read()
        lastSensorsRead = time()
    
    
    

def updateOLED():
    global distance, temperature, humidity, potValue
    myList=[]
    myList.append("Distance: " + str (distance) +" cm")
    myList.append("Temperature: " + str (temperature))
    myList.append("Humidity: " + str (humidity))
    myList.append("PotValue: " + str (potValue))
    displayOLED(myList)

def action():
    global FAN_TEMP, temperature, potValue, DOOR_CLOSED, DOOR_OPEN
    if temperature > FAN_TEMP:
        fanTrans.value(1)
    else:
        fanTrans.value(0)
    door.duty(mapRange(potValue, 0, 1023, DOOR_CLOSED, DOOR_OPEN))

while True:
    readSensors()
    action()
    updateOLED()
    