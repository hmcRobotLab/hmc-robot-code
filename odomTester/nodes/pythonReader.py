f = open('data', 'r')
string = f.readline().strip()[2:][:-2]
arr1 = string.split('], [')
arr2 = map(lambda x: map(lambda y: float(y), x), map(lambda x: x.split(','), arr1))

