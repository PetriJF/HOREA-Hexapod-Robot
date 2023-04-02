import RPi.GPIO as GPIO
import time

BUTTON1_PIN = 16
BUTTON2_PIN = 20
BUTTON3_PIN = 21
GPIO.setmode(GPIO.BCM)

GPIO.setup(BUTTON1_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(BUTTON2_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(BUTTON3_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)

previous_button_state = [ GPIO.input(BUTTON1_PIN), GPIO.input(BUTTON2_PIN), GPIO.input(BUTTON3_PIN) ]

try:
    while True:
        time.sleep(0.01)
        button_state = [ GPIO.input(BUTTON1_PIN), GPIO.input(BUTTON2_PIN), GPIO.input(BUTTON3_PIN) ]
        if(button_state != previous_button_state):

            if button_state[0] == GPIO.HIGH and button_state[0] != previous_button_state[0]:
                print("Button 1 released")
            elif button_state[0] == GPIO.LOW and button_state[0] != previous_button_state[0]:
                print("Button 1 pressed")
            
            if button_state[1] == GPIO.HIGH and button_state[1] != previous_button_state[1]:
                print("Button 2 released")
            elif button_state[1] == GPIO.LOW and button_state[1] != previous_button_state[1]:
                print("Button 2 pressed")
            
            if button_state[2] == GPIO.HIGH and button_state[2] != previous_button_state[2]:
                print("Button 3 released")
            elif button_state[2] == GPIO.LOW and button_state[2] != previous_button_state[2]:
                print("Button 3 pressed")

            previous_button_state = button_state
except KeyboardInterrupt:
    print("end")
    GPIO.cleanup()
