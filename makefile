#!c++
# https://github.com/ompl/ompl/issues/243

OMPL_DIR = /usr/local
CXX_FLAGS = -O2 
INCLUDE_FLAGS = -I/usr/include/eigen3 -I/usr/local/include/ompl-1.6
LD_FLAGS = -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lompl_app -lompl_app_base -l boost_system -lboost_serialization -lboost_program_options -Wl,-rpath -lompl_app_base
CXX=c++


rigid: se3_rigid.o
	$(CXX) $(CXX_FLAGS) -o se3_rigid se3_rigid.o $(LD_FLAGS)
	
clean:
	rm *.o
	
%.o: %.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@