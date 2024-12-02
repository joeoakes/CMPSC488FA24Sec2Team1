from gpiozero import PWMOutputDevice
import time

speaker = PWMOutputDevice(13)

C = 261.6256
E = 311.1270
F = 349.2282
G = 391.9954
A = 415.3047
B = 466.1638
C2 = 523.2511

QUARTER = 0.75
EIGHTH = QUARTER * (1 / 2)
SIXTEENTH = QUARTER * (1 / 4)


def play_tone():
    speaker.value = 0.5
    for freq, sleep in [
        (C, QUARTER),
        (E, QUARTER),
        (F, EIGHTH + SIXTEENTH),
        (E, SIXTEENTH + EIGHTH),
        (F, EIGHTH),
        (F, EIGHTH),
        (F, EIGHTH),
        (B, EIGHTH),
        (A, EIGHTH),
        (G, SIXTEENTH),
        (F, EIGHTH),
        (G, SIXTEENTH + QUARTER),
        (G, QUARTER),
        (B, QUARTER),
        (C2, EIGHTH + SIXTEENTH),
        (F, SIXTEENTH + EIGHTH),
        (E, EIGHTH),
        (B, EIGHTH),
        (B, EIGHTH),
        (G, EIGHTH),
        (B, EIGHTH),
        (B, EIGHTH + SIXTEENTH),
        (C2, QUARTER + QUARTER),
    ]:
        speaker.frequency = freq
        time.sleep(sleep)
        speaker.value = 0
        time.sleep(0.01)
        speaker.value = 0.5
    speaker.off()


play_tone()
