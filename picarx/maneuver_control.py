from picarx_improved import PicarxManeuvers

def main():
    maneuvers = PicarxManeuvers()

    print("Welcome to PiCarX Maneuver Control!")
    print("Controls:")
    print("  w: Move forward")
    print("  s: Move backward")
    print("  a: Parallel park left")
    print("  d: Parallel park right")
    print("  q: Three-point turn left")
    print("  e: Three-point turn right")
    print("  x: Exit")

    while True:
        user_input = input("Enter a command: ").strip().lower()

        if user_input == 'w':
            maneuvers.move_forward(speed=50, duration=2)
        elif user_input == 's':
            maneuvers.move_backward(speed=50, duration=2)
        elif user_input == 'a':
            maneuvers.parallel_park_left(speed=50, duration=1.5)
        elif user_input == 'd':
            maneuvers.parallel_park_right(speed=50, duration=1.5)
        elif user_input == 'q':
            maneuvers.three_point_turn_left(speed=50, duration=1.5)
        elif user_input == 'e':
            maneuvers.three_point_turn_right(speed=50, duration=1.5)
        elif user_input == 'x':
            print("Exiting. Goodbye!")
            break
        else:
            print("Invalid input! Please try again.")

if __name__ == "__main__":
    main()
