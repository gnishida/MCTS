import sys
import pylab as plt

def main(filename):
	values = []
	f = open(filename, 'r')
	for line in f:
		values.append(float(line))
	
	x = xrange(len(values))
	plt.plot(x, values)
	plt.show()
		

if __name__ == '__main__':
	if len(sys.argv) != 2:
		print("Usage: graph.py <filename>")
		quit()

	main(sys.argv[1])
