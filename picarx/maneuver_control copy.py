from picarx_improved import PicarxManeuvers

if __name__ == "__main__":
    maneuvers = PicarxManeuvers()
    maneuvers.move_forward(speed=50, duration=2)
    maneuvers.parallel_park_left(speed=50, duration=1)
    maneuvers.three_point_turn_right(speed=50, duration=1.5)
