import RPi.GPIO as GPIO
import time

BUTTONLB_PIN = 16
BUTTONLM_PIN = 12
BUTTONLF_PIN = 5

BUTTONRB_PIN = 13
BUTTONRM_PIN = 19
BUTTONRF_PIN = 26

GPIO.setmode(GPIO.BCM)

GPIO.setup(BUTTONLB_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(BUTTONLM_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(BUTTONLF_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(BUTTONRB_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(BUTTONRM_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(BUTTONRF_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)

previous_button_state = [ 
    GPIO.input(BUTTONRF_PIN), 
    GPIO.input(BUTTONRM_PIN), 
    GPIO.input(BUTTONRB_PIN),
    GPIO.input(BUTTONLB_PIN), 
    GPIO.input(BUTTONLM_PIN), 
    GPIO.input(BUTTONLF_PIN)
]
try:
    while True:
        time.sleep(0.01)
        button_state = [ 
            GPIO.input(BUTTONRF_PIN), 
            GPIO.input(BUTTONRM_PIN), 
            GPIO.input(BUTTONRB_PIN),
            GPIO.input(BUTTONLB_PIN), 
            GPIO.input(BUTTONLM_PIN), 
            GPIO.input(BUTTONLF_PIN)
        ]

        if(button_state != previous_button_state):

            if button_state[0] == GPIO.HIGH and button_state[0] != previous_button_state[0]:
                print("Button RF released")
            elif button_state[0] == GPIO.LOW and button_state[0] != previous_button_state[0]:
                print("Button RF pressed")
            
            if button_state[1] == GPIO.HIGH and button_state[1] != previous_button_state[1]:
                print("Button RM released")
            elif button_state[1] == GPIO.LOW and button_state[1] != previous_button_state[1]:
                print("Button RM pressed")
            
            if button_state[2] == GPIO.HIGH and button_state[2] != previous_button_state[2]:
                print("Button RB released")
            elif button_state[2] == GPIO.LOW and button_state[2] != previous_button_state[2]:
                print("Button RB pressed")
            
            if button_state[3] == GPIO.HIGH and button_state[3] != previous_button_state[3]:
                print("Button LB released")
            elif button_state[3] == GPIO.LOW and button_state[3] != previous_button_state[3]:
                print("Button LB pressed")
            
            if button_state[4] == GPIO.HIGH and button_state[4] != previous_button_state[4]:
                print("Button LM released")
            elif button_state[4] == GPIO.LOW and button_state[4] != previous_button_state[4]:
                print("Button LM pressed")
            
            if button_state[5] == GPIO.HIGH and button_state[5] != previous_button_state[5]:
                print("Button LF released")
            elif button_state[5] == GPIO.LOW and button_state[5] != previous_button_state[5]:
                print("Button LF pressed")

            previous_button_state = button_state
except KeyboardInterrupt:
    print("end")
    GPIO.cleanup()
