all: carp

carp: carp.cpp Graph.hpp FrontierBasedSearch.hpp DegreeConstraint.hpp TourSpec.hpp
	g++ carp.cpp -o carp -O3 -I. -fopenmp

clean:
	rm -f carp carp.exe *.o