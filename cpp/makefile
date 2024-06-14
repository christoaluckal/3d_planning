#!c++
# https://github.com/ompl/ompl/issues/243

OMPL_DIR = /usr/local
CPP_FILES = /cpp
CXX_FLAGS = -O2 
INCLUDE_FLAGS = -I/usr/include/eigen3 -I/usr/local/include/ompl-1.6
LD_FLAGS = -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lompl_app -lompl_app_base -l boost_system -lboost_serialization -lboost_program_options -Wl,-rpath -lompl_app_base
CXX=c++


se3rigid: se3_rigid.o
	$(CXX) $(CXX_FLAGS) -o exe_se3_rigid se3_rigid.o $(LD_FLAGS)
	rm se3_rigid.o 

optimal: optimal_planning.o
	$(CXX) $(CXX_FLAGS) -o exe_optimal optimal_planning.o $(LD_FLAGS) -L/usr/local/lib/
	rm optimal_planning.o

ppm: point2d.o
	$(CXX) $(CXX_FLAGS) -o exe_point2d point2d.o $(LD_FLAGS) -L/usr/local/lib/
	rm point2d.o

optimal_ppm: optimal_image.o
	$(CXX) $(CXX_FLAGS) -o exe_optimal_image optimal_image.o $(LD_FLAGS) -L/usr/local/lib/
	rm optimal_image.o
	
clean:
	rm *.o
	rm exe_*
	
%.o: %.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@