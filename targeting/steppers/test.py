import gpiozero as GPIO
import time
from DRV8825 import DRV8825

try:
    Motor1 = DRV8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
    Motor2 = DRV8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))

    """
    # 1.8 degree: nema23, nema14
    # softward Control :
    # 'fullstep': A cycle = 200 steps
    # 'halfstep': A cycle = 200 * 2 steps
    # '1/4step': A cycle = 200 * 4 steps
    # '1/8step': A cycle = 200 * 8 steps
    # '1/16step': A cycle = 200 * 16 steps
    # '1/32step': A cycle = 200 * 32 steps
    """
    Motor2.set_micro_step("softward", "fullstep")
    for _ in range(10):
        Motor2.turn_step(dir="forward", steps=200 * 32, stepdelay=0.00500)
        # Motor1.TurnStep(Dir="backward", steps=200 * 32, stepdelay=0.00000)

    Motor2.stop()

    """
    # 28BJY-48:
    # softward Control :
    # 'fullstep': A cycle = 2048 steps
    # 'halfstep': A cycle = 2048 * 2 steps
    # '1/4step': A cycle = 2048 * 4 steps
    # '1/8step': A cycle = 2048 * 8 steps
    # '1/16step': A cycle = 2048 * 16 steps
    # '1/32step': A cycle = 2048 * 32 steps

    Motor2.SetMicroStep('hardward' ,'halfstep')    
    Motor2.TurnStep(Dir='forward', steps=2048, stepdelay=0.002)
    time.sleep(0.5)
    Motor2.TurnStep(Dir='backward', steps=2048, stepdelay=0.002)
    Motor2.Stop()
    """

    Motor1.stop()
    Motor2.stop()

except:
    # GPIO.cleanup()
    print("\nMotor stop")
    Motor1.stop()
    Motor2.stop()
    exit()
