from gpiozero import PWMOutputDevice
import time
import librosa

speaker = PWMOutputDevice(13)
notes = [
    "A#3",
    "B3",
    "C4",
    "D4",
    "D#4",
    "E4",
    "F4",
    "G4",
    "G#4",
    "A4",
    "A#4",
    "B4",
    "C5",
    "D5",
    "D#5",
    "E5",
    "F5",
    "G5",
    "G#5",
    "A5",
    "A#5",
    "B5",
    "C6",
]
frequencies = {note: librosa.note_to_hz(note) for note in notes}

QUARTER = 0.75
EIGHTH = QUARTER * (1 / 2)
SIXTEENTH = QUARTER * (1 / 4)
HALF = QUARTER * 2


def play_tone():
    speaker.value = 0.5
    for freq, sleep in [
        # Bar1
        (frequencies["C4"], QUARTER),
        (frequencies["D#4"], QUARTER),
        (frequencies["F4"], EIGHTH + SIXTEENTH),
        (frequencies["D#4"], SIXTEENTH + EIGHTH),
        (frequencies["F4"], EIGHTH),
        # Bar2
        (frequencies["F4"], EIGHTH),
        (frequencies["F4"], EIGHTH),
        (frequencies["A#4"], EIGHTH),
        (frequencies["G#4"], EIGHTH),
        (frequencies["G4"], SIXTEENTH),
        (frequencies["F4"], EIGHTH),
        (frequencies["G4"], SIXTEENTH + QUARTER),
        # Bar3
        (frequencies["G4"], QUARTER),
        (frequencies["A#4"], QUARTER),
        (frequencies["C5"], EIGHTH + SIXTEENTH),
        (frequencies["G4"], SIXTEENTH + EIGHTH),
        (frequencies["F4"], EIGHTH),
        # Bar4
        (frequencies["A#4"], EIGHTH),
        (frequencies["A#4"], EIGHTH),
        (frequencies["G4"], EIGHTH),
        (frequencies["A#4"], EIGHTH),
        # Bar5
        (frequencies["A#4"], EIGHTH + SIXTEENTH),
        (frequencies["C5"], SIXTEENTH + QUARTER + HALF),
        # Bar6
        (frequencies["C4"], QUARTER),
        (frequencies["D#4"], QUARTER),
        (frequencies["F4"], EIGHTH + SIXTEENTH),
        (frequencies["D#4"], SIXTEENTH + EIGHTH),
        (frequencies["F4"], EIGHTH),
        # Bar7
        (frequencies["F4"], EIGHTH),
        (frequencies["F4"], EIGHTH),
        (frequencies["A#4"], EIGHTH),
        (frequencies["G#4"], EIGHTH),
        (frequencies["G4"], SIXTEENTH),
        (frequencies["F4"], EIGHTH),
        (frequencies["G4"], SIXTEENTH + EIGHTH),
        # Bar8
        (frequencies["G4"], QUARTER),
        (frequencies["A#4"], QUARTER),
        (frequencies["C5"], EIGHTH + SIXTEENTH),
        (frequencies["G4"], SIXTEENTH + EIGHTH),
        (frequencies["F4"], EIGHTH),
        # Bar9
        (frequencies["A#4"], EIGHTH),
        (frequencies["A#4"], EIGHTH),
        (frequencies["G4"], EIGHTH),
        (frequencies["A#4"], EIGHTH),
        (frequencies["A#4"], EIGHTH + SIXTEENTH),
        (frequencies["C5"], SIXTEENTH + QUARTER),
        # Bar10
        (0, QUARTER),
        (frequencies["D#4"], EIGHTH),
        (frequencies["A#3"], SIXTEENTH),
        (frequencies["A#3"], SIXTEENTH + QUARTER),
        (0, EIGHTH),
        (frequencies["D#4"], EIGHTH),
        # Bar11
        (frequencies["D#4"], EIGHTH + SIXTEENTH),
        (frequencies["F4"], SIXTEENTH + EIGHTH),
        (frequencies["A#3"], EIGHTH),
        (frequencies["A#3"], QUARTER),
        (0, EIGHTH),
        (frequencies["A#3"], EIGHTH),
        # Bar12
        (frequencies["G4"], EIGHTH + SIXTEENTH),
        (frequencies["G#4"], SIXTEENTH + EIGHTH),
        (frequencies["G4"], EIGHTH),
        (frequencies["F4"], EIGHTH + SIXTEENTH),
        (frequencies["D#4"], SIXTEENTH + EIGHTH),
        (frequencies["F4"], EIGHTH),
        # Bar13
        (frequencies["G4"], EIGHTH + SIXTEENTH),
        (frequencies["G#4"], SIXTEENTH + EIGHTH),
        (frequencies["G4"], EIGHTH),
        (frequencies["C4"], QUARTER),
        (0, EIGHTH),
        (frequencies["C4"], SIXTEENTH),
        (frequencies["D4"], SIXTEENTH),
        # Bar14
        (frequencies["D#4"], EIGHTH + SIXTEENTH),
        (frequencies["D#4"], SIXTEENTH + EIGHTH),
        (frequencies["D4"], EIGHTH),
        (frequencies["D4"], QUARTER),
        (0, EIGHTH),
        (frequencies["D#4"], SIXTEENTH),
        (frequencies["F4"], SIXTEENTH),
        # Bar15
        (frequencies["G#4"], EIGHTH + SIXTEENTH),
        (frequencies["G4"], SIXTEENTH + EIGHTH),
        (frequencies["F4"], EIGHTH),
        (frequencies["D#4"], QUARTER),
        (0, EIGHTH),
        (frequencies["G4"], EIGHTH),
        # Bar16
        (frequencies["G4"], EIGHTH + SIXTEENTH),
        (frequencies["F4"], SIXTEENTH + EIGHTH),
        (frequencies["E4"], EIGHTH),
        (frequencies["F4"], QUARTER),
        (frequencies["C4"], QUARTER),
        # Bar17
        (frequencies["C4"], QUARTER + EIGHTH),
        (frequencies["D4"], EIGHTH),
        (frequencies["D4"], HALF),
        # Bar18
        (0, QUARTER),
        (frequencies["D#4"], EIGHTH),
        (frequencies["A#3"], SIXTEENTH),
        (frequencies["A#3"], SIXTEENTH + QUARTER),
        (0, EIGHTH),
        (frequencies["D#4"], EIGHTH),
        # Bar19
        (frequencies["D#4"], EIGHTH + SIXTEENTH),
        (frequencies["F4"], SIXTEENTH + EIGHTH),
        (frequencies["A#3"], EIGHTH),
        (frequencies["A#3"], QUARTER),
        (0, EIGHTH),
        (frequencies["A#3"], EIGHTH),
        # Bar20
        (frequencies["G4"], EIGHTH + SIXTEENTH),
        (frequencies["G#4"], SIXTEENTH + EIGHTH),
        (frequencies["G4"], EIGHTH),
        (frequencies["F4"], EIGHTH + SIXTEENTH),
        (frequencies["D#4"], SIXTEENTH + EIGHTH),
        (frequencies["F4"], EIGHTH),
        # Bar21
        (frequencies["G4"], EIGHTH + SIXTEENTH),
        (frequencies["G#4"], SIXTEENTH + EIGHTH),
        (frequencies["G4"], EIGHTH),
        (frequencies["C4"], QUARTER),
        (0, EIGHTH),
        (frequencies["C4"], SIXTEENTH),
        (frequencies["D4"], SIXTEENTH),
        # Bar22
        (frequencies["D#4"], EIGHTH + SIXTEENTH),
        (frequencies["D#4"], SIXTEENTH + EIGHTH),
        (frequencies["D4"], EIGHTH),
        (frequencies["D4"], QUARTER),
        (0, EIGHTH),
        (frequencies["D#4"], SIXTEENTH),
        (frequencies["F4"], SIXTEENTH),
        # Bar23
        (frequencies["G#4"], EIGHTH + SIXTEENTH),
        (frequencies["G4"], SIXTEENTH + EIGHTH),
        (frequencies["F4"], EIGHTH),
        (frequencies["D#4"], QUARTER),
        (0, EIGHTH),
        (frequencies["G4"], EIGHTH),
        # Bar24
        (frequencies["G4"], EIGHTH + SIXTEENTH),
        (frequencies["F4"], SIXTEENTH + EIGHTH),
        (frequencies["E4"], EIGHTH),
        (frequencies["F4"], EIGHTH + SIXTEENTH),
        (frequencies["G4"], SIXTEENTH + EIGHTH),
        (frequencies["G#4"], EIGHTH),
        # Bar25
        (frequencies["G4"], HALF + QUARTER),
        (0, QUARTER),
        # Bar26
        (frequencies["D#4"], EIGHTH + SIXTEENTH),
        (frequencies["D#4"], SIXTEENTH + EIGHTH),
        (frequencies["D4"], EIGHTH),
        (frequencies["D#4"], EIGHTH + SIXTEENTH),
        (frequencies["D#4"], SIXTEENTH + EIGHTH),
        (frequencies["D4"], EIGHTH),
    ]:
        if freq == 0:
            speaker.value = 0
            time.sleep(sleep)
            time.sleep(0.01)
            speaker.value = 0.5
        else:
            speaker.frequency = freq
            time.sleep(sleep)
            speaker.value = 0
            time.sleep(0.01)
            speaker.value = 0.5
    speaker.off()


play_tone()
