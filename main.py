# Include the library files
import I2C_LCD_driver
import RPi.GPIO as GPIO
from time import sleep
import threading
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import json
import board
import busio
from digitalio import DigitalInOut

from adafruit_pn532.i2c import PN532_I2C

############################################## MQTT ##################################################
client = mqtt.Client()
client.username_pw_set("admin", "12345678")
client.connect("192.168.43.75", 1883, 60)

################################################ NFC ##################################################
i2c = busio.I2C(board.SCL, board.SDA)

allowed_card = "0xe70xd60x1b0xfb"

# harware reset
reset_pin = DigitalInOut(board.D6)
# On Raspberry Pi, you must also connect a pin to P32 "H_Request" for hardware
# wakeup! this means we don't need to do the I2C clock-stretch thing
req_pin = DigitalInOut(board.D12)
pn532 = PN532_I2C(i2c, debug=False, reset=reset_pin, req=req_pin)

ic, ver, rev, support = pn532.firmware_version
print("Found PN532 with firmware version: {0}.{1}".format(ver, rev))

# Configure PN532 to communicate with MiFare cards
pn532.SAM_configuration()

print("Waiting for RFID/NFC card...")

############################################ LCD I TASTATURA ##############################################

# Enter column pins
C4 = 5
C3 = 6
C2 = 13
C1 = 19
# Enter row pins
R4 = 12
R3 = 16
R2 = 20
R1 = 21
# Enter buzzer pin
buzzer = 17
# Enter LED pin
Relay = 27

# Create a object for the LCD
lcd = I2C_LCD_driver.lcd()
#Starting text
lcd.lcd_display_string("System loading",1,1)
for a in range (0,16):
    lcd.lcd_display_string(".",2,a)
    sleep(0.1)
lcd.lcd_clear()
# The GPIO pin of the column of the key that is currently
# being held down or -1 if no key is pressed
keypadPressed = -1
# Enter your PIN
secretCode = "1111"
input = ""
# Setup GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(buzzer,GPIO.OUT)
GPIO.setup(Relay,GPIO.OUT)
GPIO.output(Relay,GPIO.HIGH)
# Set column pins as output pins
GPIO.setup(R1, GPIO.OUT)
GPIO.setup(R2, GPIO.OUT)
GPIO.setup(R3, GPIO.OUT)
GPIO.setup(R4, GPIO.OUT)
# Set row pins as input pins
GPIO.setup(C1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(C2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(C3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(C4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# This callback registers the key that was pressed
# if no other key is currently pressed
def keypadCallback(channel):
    global keypadPressed
    if keypadPressed == -1:
        keypadPressed = channel
# Detect the rising edges
GPIO.add_event_detect(C1, GPIO.RISING, callback=keypadCallback)
GPIO.add_event_detect(C2, GPIO.RISING, callback=keypadCallback)
GPIO.add_event_detect(C3, GPIO.RISING, callback=keypadCallback)
GPIO.add_event_detect(C4, GPIO.RISING, callback=keypadCallback)

# Sets all rows to a specific state. 
def setAllRows(state):
    GPIO.output(R1, state)
    GPIO.output(R2, state)
    GPIO.output(R3, state)
    GPIO.output(R4, state)

num_of_tries = 0
masterCode = "0000"
input_blocked = False  # Global variable to control input blocking
pin_change_mode = False
correct_old_pin = False

# Check or clear PIN
def commands():
    global input
    global num_of_tries
    global masterCode
    global input_blocked, pin_change_mode
    global client
    pressed = False
    GPIO.output(R1, GPIO.HIGH)
    
    # Clear PIN 
    if (GPIO.input(C4) == 1):
        print("Input reset!")
        lcd.lcd_clear()
        lcd.lcd_display_string("Clear",1,5)
        sleep(1)
        pressed = True

    GPIO.output(R1, GPIO.LOW)
    GPIO.output(R2, GPIO.HIGH)
    
    # Change PIN 
    if (GPIO.input(C4) == 1):
        print("Change PIN option")
        lcd.lcd_clear()
        lcd.lcd_display_string("Change PIN mode",1,0)
        sleep(1)
        pressed = True
        pin_change_mode = True
        
    GPIO.output(R2, GPIO.LOW)
    
    GPIO.output(R4, GPIO.HIGH)
    # Check PIN
    if (not pressed and GPIO.input(C3) == 1):
        client.publish("tries", payload=json.dumps({"title":"try", "pin":input}), qos=0, retain=True)
        print(json.dumps({"title":"try", "pin":input}))
        if input == secretCode and not input_blocked:
            print("Code correct!")
            lcd.lcd_clear()
            lcd.lcd_display_string("Successful",1,3)
            
            GPIO.output(Relay,GPIO.LOW)
            GPIO.output(buzzer,GPIO.HIGH)
            sleep(0.3)
            GPIO.output(buzzer,GPIO.LOW)
            sleep(5)
            GPIO.output(Relay,GPIO.HIGH)

        elif input == masterCode and input_blocked:
            print("Master code correct!")
            lcd.lcd_clear()
            lcd.lcd_display_string("Tries reset!",1,3)
            num_of_tries = 0
            input_blocked = False
            
            GPIO.output(Relay,GPIO.LOW)
            GPIO.output(buzzer,GPIO.HIGH)
            sleep(0.3)
            GPIO.output(buzzer,GPIO.LOW)
            sleep(5)
            GPIO.output(Relay,GPIO.HIGH)      
            
        elif not input_blocked:
            print("Incorrect code!")
            num_of_tries += 1
            lcd.lcd_clear()
            lcd.lcd_display_string("Wrong PIN!",1,3)
            GPIO.output(buzzer,GPIO.HIGH)
            sleep(0.3)
            GPIO.output(buzzer,GPIO.LOW)
            sleep(0.3)
            GPIO.output(buzzer,GPIO.HIGH)
            sleep(0.3)
            GPIO.output(buzzer,GPIO.LOW)
            sleep(0.3)
            GPIO.output(buzzer,GPIO.HIGH)
            sleep(0.3)
            GPIO.output(buzzer,GPIO.LOW) 
            if num_of_tries >= 3:
                block_input()
        pressed = True
    GPIO.output(R4, GPIO.LOW)
    if pressed:
        input = ""
    return pressed

# reads the columns and appends the value, that corresponds
# to the button, to a variable
def read(column, characters):
    global input
    GPIO.output(column, GPIO.HIGH)
    if(GPIO.input(C1) == 1):
        input = input + characters[0]
        lcd.lcd_display_string(str(input),2,0)
    if(GPIO.input(C2) == 1):
        input = input + characters[1]
        lcd.lcd_display_string(str(input),2,0)
    if(GPIO.input(C3) == 1):
        input = input + characters[2]
        lcd.lcd_display_string(str(input),2,0)
    if(GPIO.input(C4) == 1):
        input = input + characters[3]
        lcd.lcd_display_string(str(input),2,0)
    GPIO.output(column, GPIO.LOW)

def read_no_write(column, characters):
    global input
    GPIO.output(column, GPIO.HIGH)
    if(GPIO.input(C1) == 1):
        input = input + characters[0]
    if(GPIO.input(C2) == 1):
        input = input + characters[1]
    if(GPIO.input(C3) == 1):
        input = input + characters[2]
    if(GPIO.input(C4) == 1):
        input = input + characters[3]
    GPIO.output(column, GPIO.LOW)

# Function to toggle input blocking
def block_input():
    global input_blocked
    input_blocked = True
    print(input_blocked)

    print(f'Failed attempts: {num_of_tries}')
    # Wait for 5 seconds in a separate thread
    timer_thread = threading.Timer(2 * num_of_tries, unblock_input)
    timer_thread.start()

def unblock_input():
    global input_blocked
    input_blocked = False
    print(input_blocked)
    print(f'Failed attempts: {num_of_tries}')

def change_pin_commands():
    global input, pin_change_mode, secretCode, correct_old_pin
    pressed = False
    GPIO.output(R1, GPIO.HIGH)
    
    # Clear PIN 
    if (GPIO.input(C4) == 1):
        print("Input reset!")
        lcd.lcd_clear()
        lcd.lcd_display_string("Clear",1,5)
        sleep(1)
        pressed = True
        if correct_old_pin:
            lcd.lcd_clear()
            lcd.lcd_display_string("Enter new PIN:",1,0)
        else:
            lcd.lcd_clear()
            lcd.lcd_display_string("Enter old PIN:",1,0)

    GPIO.output(R1, GPIO.LOW)

    GPIO.output(R4, GPIO.HIGH)
    if (GPIO.input(C1) == 1):
        print("Exit change PIN mode")
        lcd.lcd_clear()
        lcd.lcd_display_string("Exiting...",1,0)
        sleep(1)
        pin_change_mode = False
        correct_old_pin = False
        pressed = True

    # Check PIN
    if (not pressed and GPIO.input(C3) == 1):
        if input == secretCode and pin_change_mode and not correct_old_pin:
            print("Code correct!")
            lcd.lcd_clear()
            lcd.lcd_display_string("Correct PIN",1,0)
            sleep(1)
            lcd.lcd_clear()
            lcd.lcd_display_string("Enter new PIN:",1,0)
            correct_old_pin = True
        elif pin_change_mode and correct_old_pin:
            if len(input) > 4 or len(input) < 4 or not input.isdigit():
                print(f'Incorrect new code: {input}')
                lcd.lcd_clear()
                lcd.lcd_display_string("Only 4 digits",1,0)
                lcd.lcd_display_string("allowed for PIN",2,0)
                sleep(1)
                lcd.lcd_clear()
                lcd.lcd_display_string("Enter new PIN:",1,0)
            else:
                secretCode = input
                lcd.lcd_clear()
                lcd.lcd_display_string("Successful",1,3)
                sleep(1)
                pin_change_mode = False
                correct_old_pin = False
        else:
            print(f'Incorrect old code: {input}')
            lcd.lcd_clear()
            lcd.lcd_display_string("Incorect PIN!",1,0)
            sleep(1)
            lcd.lcd_clear()
            lcd.lcd_display_string("Enter old PIN:",1,0)
        pressed = True
            
            
    GPIO.output(R4, GPIO.LOW)
    if pressed:
        input = ""
    return pressed


def handle_pin_change():
    global pin_change_mode, input, secretCode, keypadPressed

    lcd.lcd_clear()
    lcd.lcd_display_string("Enter old PIN:", 1, 0)

    while pin_change_mode:
        if keypadPressed != -1:
                setAllRows(GPIO.HIGH)
                if GPIO.input(keypadPressed) == 0:
                    keypadPressed = -1
                else:
                    sleep(0.1)
            # Otherwise, just read the input
        else:
            if not change_pin_commands():
                read(R1, ["1","2","3","A"])
                read(R2, ["4","5","6","B"])
                read(R3, ["7","8","9","C"])
                read(R4, ["*","0","#","D"])
                sleep(0.1)
            else:
                sleep(0.1)

lcd_lock = threading.Lock()

def check_pin():
    global keypadPressed
    while True:       
        if not input_blocked:
            with lcd_lock:
                lcd.lcd_display_string("Enter your PIN:", 1, 0)
            
            # If a button was previously pressed,
            # check, whether the user has released it yet
            if keypadPressed != -1:
                setAllRows(GPIO.HIGH)
                if GPIO.input(keypadPressed) == 0:
                    keypadPressed = -1
                else:
                    sleep(0.1)
            # Otherwise, just read the input
            else:
                if not commands():
                    if pin_change_mode:
                        handle_pin_change()
                    else:
                        read(R1, ["1","2","3","A"])
                        read(R2, ["4","5","6","B"])
                        read(R3, ["7","8","9","C"])
                        read(R4, ["*","0","#","D"])
                        sleep(0.1)
                else:
                    sleep(0.1)
        else:
            with lcd_lock:
                lcd.lcd_display_string("Wait to retry",1,0)
            if keypadPressed != -1:
                setAllRows(GPIO.HIGH)
                if GPIO.input(keypadPressed) == 0:
                    keypadPressed = -1
                else:
                    sleep(0.1)
            # Otherwise, just read the input
            else:
                if not commands():
                    read_no_write(R1, ["1","2","3","A"])
                    read_no_write(R2, ["4","5","6","B"])
                    read_no_write(R3, ["7","8","9","C"])
                    read_no_write(R4, ["*","0","#","D"])
                    sleep(0.1)
                else:
                    sleep(0.1)

def check_card():
    global allowed_card
    while True:
        uid = pn532.read_passive_target(timeout=0.5)
        # Try again if no card is available.
        if uid is None:
            continue
        list = [hex(x) for x in uid]
        uid_string = ''.join(list)
        client.publish("tries", payload=json.dumps({"title":"try", "pin":uid_string}), qos=0, retain=True)
        print(json.dumps({"title":"try", "pin":input}))
        with lcd_lock:
            if uid_string == allowed_card or uid_string.startswith("0x80"):
                lcd.lcd_clear()
                lcd.lcd_display_string("Successfull!",1,1)
                GPIO.output(Relay,GPIO.LOW)
                GPIO.output(buzzer,GPIO.HIGH)
                sleep(0.3)
                GPIO.output(buzzer,GPIO.LOW)
                sleep(5)
                GPIO.output(Relay,GPIO.HIGH)
            else:
                lcd.lcd_clear()
                lcd.lcd_display_string("Invalid card!",1,0)
                GPIO.output(buzzer,GPIO.HIGH)
                sleep(0.3)
                GPIO.output(buzzer,GPIO.LOW)
                sleep(0.3)
                GPIO.output(buzzer,GPIO.HIGH)
                sleep(0.3)
                GPIO.output(buzzer,GPIO.LOW)
                sleep(0.3)
                GPIO.output(buzzer,GPIO.HIGH)
                sleep(0.3)
                GPIO.output(buzzer,GPIO.LOW) 

# Create threads for the two checks
pin_thread = threading.Thread(target=check_pin)
card_thread = threading.Thread(target=check_card)

try:
    pin_thread.start()
    card_thread.start()

    # Wait for the threads to finish (this will never happen since they run indefinitely)
    pin_thread.join()
    card_thread.join()

except KeyboardInterrupt:
    print("Stopped!")
    lcd.lcd_clear()