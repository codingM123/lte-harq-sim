# Builds the simulator and the unit tests. Works with g++ (Linux) and clang++ (macOS).
#
#   make           build bin/harq_sim and bin/run_tests
#   make test      run the unit tests
#   make results   run every experiment and write the CSV files to results/
#   make clean     delete build output

CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS += -Iinclude
LDLIBS   += -pthread

LIB_SRCS := src/crc24a.cpp src/conv_code.cpp src/rate_matcher.cpp src/modem.cpp \
            src/harq_process.cpp src/simulator.cpp
LIB_OBJS := $(LIB_SRCS:src/%.cpp=build/%.o)

all: bin/harq_sim bin/run_tests

bin/harq_sim: $(LIB_OBJS) build/main.o | bin
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDLIBS)

bin/run_tests: $(LIB_OBJS) build/test_main.o | bin
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDLIBS)

build/%.o: src/%.cpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

build/test_main.o: tests/test_main.cpp | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

bin build:
	mkdir -p $@

test: bin/run_tests
	./bin/run_tests

results: bin/harq_sim
	./bin/harq_sim --out results
	./bin/harq_sim --validate --out results
	./bin/harq_sim --process-sweep --snr 4 --out results

clean:
	rm -rf build bin

-include $(wildcard build/*.d)

.PHONY: all test results clean
