cppflags= -O3 -Wall -Wextra -Wfatal-errors
source=simpleImplementation
voxelSource=voxelImplementation
cudaSource=cudaImplementation
programName=BooleanRender
cudaProgramName=CudaBooleanRender
voxelProgramName=VoxelBooleanRender

$(programName): $(source).cc
	g++ $(cppflags) -I ./ $(source).cc -o $(programName)
clean:
	rm -f $(cudaProgramName) $(programName) $(programName) *.o
cuda:
	nvcc $(cudaSource).cu -o $(cudaProgramName) -O3 --ptxas-options -v --gpu-architecture=sm_61 --compiler-bindir /usr/local/gcc-6.4.0/bin -std=c++11
voxel:
	g++ $(cppflags) -I ./ $(voxelSource).cc -o $(voxelProgramName)