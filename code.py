# import required modules
from array import array
import math
import board  # type: ignore
import analogio  # type: ignore
import digitalio  # type: ignore
import usb_hid  # type: ignore
import time
import rp2pio  # type: ignore
import adafruit_pioasm  # type: ignore

btn_matrix = adafruit_pioasm.assemble(
    """
; https://github.com/GitJer/Some_RPI-Pico_stuff/blob/main/button_matrix_4x4/4x4_button_matrix.pio
; in the c-program the 'set' pins are the 4 output pins to the 4x4 button matrix
; in the c-program the 'in' pins are the 4 input pins from the 4x4 button matrix, these are pulled_down

.program button_matrix

start:
    set pindirs 1
    set pins 1 [31]  ; set 0001 on the 4 output pins (activate first row) and wait for the signal to stabilize
    in pins 4        ; shift the input pins into the ISR
    set pindirs 2
    set pins 2 [31]  ; set 0010 on the 4 output pins (activate second row) and wait for the signal to stabilize
    in pins 4        ; shift the input pins into the ISR
    set pindirs 4
    set pins 4 [31]  ; set 0100 on the 4 output pins (activate third row) and wait for the signal to stabilize
    in pins 4        ; shift the input pins into the ISR
    set pindirs 8
    set pins 8 [31]  ; set 1000 on the 4 output pins (activate fourth row) and wait for the signal to stabilize
    in pins 4        ; shift the input pins into the ISR
    push block     ; a key was pressed, push the ISR into the RX FIFO
    jmp start        ; start over
"""
)

btn_matrix_params = {
    "frequency": 1_000_000,
    "auto_push": False,
    "in_shift_right": False,
    "wait_for_txstall": False,
    "in_pin_count": 4,
    "set_pin_count": 4,
    "initial_set_pin_direction": 0xFFFF,
    "pull_in_pin_down": 0xFFFF,
}

xAxis = analogio.AnalogIn(board.GP27)  # X axis reference
yAxis = analogio.AnalogIn(board.GP26)  # X axis reference
SW = digitalio.DigitalInOut(board.GP22)  # Switch button reference
SW.direction = digitalio.Direction.INPUT
SW.pull = digitalio.Pull.UP

import collections

# Define the constant window size
WINDOW_SIZE = 3


def moving_average(new_value, window):
    # Add the new value to the window
    window.append(new_value)

    # Calculate the average of the current window
    return sum(window) / len(window)


dt = 16000000


# Wait until after deadline_ns has passed
def sleep_deadline(deadline_ns):
    while time.monotonic_ns() < deadline_ns:
        pass


def sleep_relative(rel_ns):
    dead = time.monotonic_ns() + rel_ns
    sleep_deadline(dead)


xWindow = collections.deque([], WINDOW_SIZE)
yWindow = collections.deque([], WINDOW_SIZE)
t0 = deadline = time.monotonic_ns()

hasCenter = 0
centerX = 0
centerY = 0

from hid_gamepad import Gamepad

gp = Gamepad(usb_hid.devices)


# Equivalent of Arduino's map() function.
def clam(x):
    return max(-32767, min(32767, x))


BUTTON_MATRIX_BASE_INPUT = board.GP6
BUTTON_MATRIX_BASE_OUTPUT = board.GP2

# btnPins = [digitalio.DigitalInOut(pin) for pin in [board.GP2, board.GP3, board.GP4, board.GP5, board.GP6, board.GP7, board.GP8, board.GP9]]

with rp2pio.StateMachine(
    btn_matrix,
    first_in_pin=BUTTON_MATRIX_BASE_INPUT,
    first_set_pin=BUTTON_MATRIX_BASE_OUTPUT,
    **btn_matrix_params
) as sm:

    btnBuffer = array("I", [0])

    def read_button_matrix():
        sm.clear_rxfifo()

        sleep_relative(1000000)

        if sm.rxfifo != None and len(sm.rxfifo) == 0:
            return [0]

        sm.readinto(btnBuffer)

        #print(f"{btnBuffer[0].to_bytes(2).hex()}")
        return btnBuffer

    while True:
        deadline += dt
        sleep_deadline(deadline)

        xRef = -(xAxis.value - 32768) - centerX  # read X axis value
        yRef = -(yAxis.value - 32768) - centerY  # read Y axis value
        SWPushed = not SW.value  # read Switch value

        if hasCenter < 10:
            hasCenter += 1
            centerX += xRef
            centerY += yRef

        xFiltered = round(moving_average(xRef, xWindow))
        yFiltered = round(moving_average(yRef, yWindow))

        if SWPushed:
            gp.press_buttons(1)
        else:
            gp.release_buttons(1)

        btnMatrix = read_button_matrix()
        btnBytes = btnMatrix[0].to_bytes(2)
        for i in range(16):
            btn = btnBytes[i // 8] & (1 << (i % 8))
            if btn:
                gp.press_buttons(i + 2)
            else:
                gp.release_buttons(i + 2)

        # for i in range(8):
        #     btnPins[i].direction = digitalio.Direction.OUTPUT
        #     btnPins[i].value = True
        #     for j in range(8):
        #         if j != i:
        #             btnPins[j].direction = digitalio.Direction.INPUT
        #             btnPins[j].pull = digitalio.Pull.DOWN
        #     sleep_relative(1e6)
        #     hasNonZero = False
        #     for j in range(8):
        #         if j != i:
        #             if btnPins[j].value:
        #                 hasNonZero = True
        #                 break
        #     if hasNonZero:
        #         print(f"Row {i}: ", end="")
        #         for j in range(8):
        #             if j != i:
        #                 if btnPins[j].value:
        #                     print("1", end="")
        #                 else:
        #                     print("0", end="")
        #         print()
        #     btnPins[i].value = False
        #     sleep_relative(1e6)

        gp.move_joysticks(
            x=clam(xFiltered),
            y=clam(yFiltered),
        )

        #print((xFiltered, yFiltered, SWPushed))
