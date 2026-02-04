def _median(arr):
    if (not arr or arr.length == 0):
        return None
    c = arr.sort()
    m = round(len(c)/2);
    if c%2 == 1:
        return c[m]
    else:
        return 0.5*(c[m-1] + c[m])