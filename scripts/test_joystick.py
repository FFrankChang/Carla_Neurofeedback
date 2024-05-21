import pygame

def main():
    pygame.init()
    pygame.joystick.init()

    # Check for joysticks
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joystick detected!")
        return

    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    try:
        print("Press any button on the steering wheel. Press CTRL+C to exit.")
        # Loop until the user clicks the close button or presses CTRL+C
        running = True
        while running:
            for event in pygame.event.get():  # User did something
                if event.type == pygame.JOYBUTTONDOWN:  # Check for joystick button presses
                    print(f"Button {event.button} pressed")
                elif event.type == pygame.QUIT:  # If user clicked close
                    running = False
    except KeyboardInterrupt:
        print("Exiting program.")

    pygame.quit()

if __name__ == "__main__":
    main()
