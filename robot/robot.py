from state import StateManager

def main(ab=False):
    sm = StateManager()
    res = sm.main(ab)
    print("returned code" + str(res))
    if res == -1:
        print("restarting...")
        # TODO: restart hardware
        main(ab)
    elif res == 1:
        print("error")
    elif res == 2:
        print("logic error")
    else:
        print("finished")

main()
    