# LEGO type:standard slot:0 autostart

from spike import PrimeHub
from spike.control import wait_for_seconds

hub = PrimeHub()

hub.speaker.set_volume(100)

hub.speaker.beep(60, 0.5)
hub.speaker.beep(67, 0.5)
hub.speaker.beep(60, 0.5)
wait_for_seconds(0.5)
hub.speaker.beep(60, 0.5)
hub.speaker.beep(67, 0.5)
hub.speaker.beep(60, 0.5)
wait_for_seconds(0.5)
hub.speaker.beep(60, 0.25)
hub.speaker.beep(60, 0.25)
hub.speaker.beep(60, 0.25)
hub.speaker.beep(60, 0.25)
hub.speaker.beep(65, 0.25)
