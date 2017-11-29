CXXFLAGS=-Wall -Wextra -std=c++11 -O2 

main: main.o compute_hull.o
	g++ $^ -o $@ $(CXXFLAGS)

clean:
	rm *.o main
