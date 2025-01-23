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
    print("\nYou can also specify duration and angle by entering them after the command.")
    print("Example: 'w 2 15' will move forward for 2 seconds at an angle of 15 degrees.")

    while True:
        user_input = input("Enter a command: ").strip().lower().split()

        if len(user_input) == 0:
            print("Invalid input! Please try again.")
            continue

        command = user_input[0]
        try:
            duration = float(user_input[1]) if len(user_input) > 1 else 2  # Default duration = 2 seconds
            angle = float(user_input[2]) if len(user_input) > 2 else 0    # Default angle = 0 degrees
        except (IndexError, ValueError):
            print("Invalid duration or angle! Please enter valid numeric values.")
            continue

        if command == 'w':
            maneuvers.move_forward(speed=50, duration=duration, angle=angle)
        elif command == 's':
            maneuvers.move_backward(speed=50, duration=duration, angle=angle)
        elif command == 'a':
            maneuvers.parallel_park_left(speed=50, duration=duration)
        elif command == 'd':
            maneuvers.parallel_park_right(speed=50, duration=duration)
        elif command == 'q':
            maneuvers.three_point_turn_left(speed=50, duration=duration)
        elif command == 'e':
            maneuvers.three_point_turn_right(speed=50, duration=duration)
        elif command == 'x':
            print("Exiting. Goodbye!")
            break
        else:
            print("Invalid input! Please try again.")

if __name__ == "__main__":
    main()
