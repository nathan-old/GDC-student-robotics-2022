from state import StateManager
import sr.robot3 as sr


def main(autoboot=False):
    sm = StateManager()
    res = sm.main(autoboot)
    print("returned code" + str(res))
    if res == -1:
        print("Handler Error")
    elif res == 1:
        print("Program error")
    elif res == 2:
        print("Unaccounted logic error")
    else:
        pass
    
    return

main()
