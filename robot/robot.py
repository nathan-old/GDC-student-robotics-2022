from state import StateManager

def main(autoboot=False):
    sm = StateManager()
    print(sm.state)
    res = sm.main(autoboot)
    print("Returned code: " + str(res))
    if res == -1:
        print("Handler Error")
    elif res == 1:
        print("Program error")
    elif res == 2:
        print("Unaccounted logic error")

main()
