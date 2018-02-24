from Game import Game


def main():
    game = Game()
    game.initial_input()

    while True:
        game.receive_input()
        game.calculate_thrusts()
        game.calculate_dests()
        game.send_output()


if __name__ == "__main__":
    main()
