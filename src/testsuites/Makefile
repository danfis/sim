-include ../../Makefile.include
CXXFLAGS = -g -Wall -pedantic
CXXFLAGS += -I./ -I../ -Icu/
CXXFLAGS += $(OSG_CXXFLAGS) $(ODE_CXXFLAGS) $(BT_CXXFLAGS)
LDFLAGS = -L./ -Lcu/ -lcu -lm -L../ -lsim
LDFLAGS += $(OSG_LDFLAGS) $(ODE_LDFLAGS) $(BT_LDFLAGS)

CHECK_REG=cu/check-regressions

OBJS = component.o message.o


all: test

test: cu $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ main.cpp $(OBJS) $(LDFLAGS)

%.o: %.cpp %.hpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

check: all
	@echo ""
	@echo "----------------------------------------";
	./test
	@echo "----------------------------------------";
	@echo "Checking regressions:";
	./$(CHECK_REG) regressions
	@echo ""
check-valgrind: all
	valgrind -q --leak-check=full --show-reachable=yes --trace-children=yes ./test

cu:
	$(MAKE) -C cu/

clean:
	rm -f *.o
	rm -f objs/*.o
	rm -f test
	rm -f tmp.*
	rm -f regressions/tmp.*

.PHONY: all clean check check-valgrind cu
	
