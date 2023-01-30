# LEGO type:standard slot:0 autostart

from spike import PrimeHub
from spike.control import wait_for_seconds

hub = PrimeHub()

images = [
    "ANGRY",
    "ARROW_E",
    "ARROW_N",
    "ARROW_NE",
    "ARROW_NW",
    "ARROW_S",
    "ARROW_SE",
    "ARROW_SW",
    "ARROW_W",
    "ASLEEP",
    "BUTTERFLY",
    "CHESSBOARD",
    "CLOCK1",
    "CLOCK10",
    "CLOCK11",
    "CLOCK12",
    "CLOCK2",
    "CLOCK3",
    "CLOCK4",
    "CLOCK5",
    "CLOCK6",
    "CLOCK7",
    "CLOCK8",
    "CLOCK9",
    "CONFUSED",
    "COW",
    "DIAMOND",
    "DIAMOND_SMALL",
    "DUCK",
    "FABULOUS",
    "GHOST",
    "GIRAFFE",
    "GO_RIGHT",
    "GO_LEFT",
    "GO_UP",
    "GO_DOWN",
    "HAPPY",
    "HEART",
    "HEART_SMALL",
    "HOUSE",
    "MEH",
    "MUSIC_CROTCHET",
    "MUSIC_QUAVER",
    "MUSIC_QUAVERS",
    "NO",
    "PACMAN",
    "PITCHFORK",
    "RABBIT",
    "ROLLERSKATE",
    "SAD",
    "SILLY",
    "SKULL",
    "SMILE",
    "SNAKE",
    "SQUARE",
    "SQUARE_SMALL",
    "STICKFIGURE",
    "SURPRISED",
    "SWORD",
    "TARGET",
    "TORTOISE",
    "TRIANGLE",
    "TRIANGLE_LEFT",
    "TSHIRT",
    "UMBRELLA",
    "XMAS",
    "YES",
]

hub.light_matrix.write("3")
wait_for_seconds(1)
hub.light_matrix.write("2")
wait_for_seconds(1)
hub.light_matrix.write("1")
wait_for_seconds(1)

for x in images:
    hub.light_matrix.show_image(x)
    wait_for_seconds(0.1)
