
def stubbornTry(fn):
    doing = True
    retq = None
    while doing:
        try:
            retq = fn()
            doing = False
        except:
            continue
    return retq

