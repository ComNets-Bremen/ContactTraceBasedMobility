from operator import itemgetter
import timeit

with open('ty.txt') as f:
    lines = [line.split(" ") for line in f]
output = open("input(sorted).txt", 'w')


for line in sorted(lines, key=itemgetter(0)):
    
    output.write(" ".join(line))

output.close()
