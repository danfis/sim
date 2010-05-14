-include ../Makefile.include
CXXFLAGS += -I../src $(OSG_CXXFLAGS) $(ODE_CXXFLAGS) $(BT_CXXFLAGS)
LDFLAGS += $(OSG_LDFLAGS) $(ODE_LDFLAGS) $(BT_LDFLAGS)


TARGETS = demo

all: $(TARGETS)

demo: demo.o
	$(CXX) $(CXXFLAGS) -o $@ $< -L../src/bullet -lsim-bullet -L../src -lsim $(LDFLAGS)

%.o: %.cpp %.hpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.moc: %.hpp
	$(MOC) -p'.' $< > $@

clean:
	rm -f *.o *~ *.a
	rm -f $(TARGETS)

.PHONY: all clean help