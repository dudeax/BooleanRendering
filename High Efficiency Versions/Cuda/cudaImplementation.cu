#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <curand.h>
#include <curand_kernel.h>

#include "Vector.h"


#define PI 3.141592653589793238463

// Camera Constants
// TODO: Turn these bad boys into variables someday
#define X_RESOLUTION 4096
#define Y_RESOLUTION 4096

#define CAMERA_X_START 1.0
#define CAMERA_Y_START 1.0
#define CAMERA_X_END -1.0
#define CAMERA_Y_END -1.0
#define CAMERA_DISTANCE 2.0

#define RAY_LENGTH 50.0
#define RAY_DETAIL 300.0 // Checks per unit
#define BINARY_SEARCH_DETAIL 0.0000001
#define NORMAL_BRANCH_DISTANCE 0.001

#define RANDOM_SEED 1234
#define LIGHTING_SAMPLES 400

#define checkCudaErrors(val) check_cuda( (val), #val, __FILE__, __LINE__ )

void check_cuda(cudaError_t result, char const *const func, const char *const file, int const line) {
    if (result) {
        std::cerr << "CUDA error = " << static_cast<unsigned int>(result) << " at " <<
            file << ":" << line << " '" << func << "' \n";
        // Make sure we call CUDA Device Reset before exiting
        cudaDeviceReset();
        exit(99);
    }
}

void writeImageOut(std::string fileName, float *imageData) {
    std::ofstream outputFile;
    
    outputFile.open(fileName);
    
    if (!outputFile.is_open()) throw std::runtime_error("Error opening output file");
    
    // Setup Headers
    outputFile << "P3" << '\n';
    outputFile << "#Created by Alex Undy" << '\n';
    outputFile << X_RESOLUTION << " " << Y_RESOLUTION << '\n';
    outputFile << "255" << '\n';
    
    // Write Image Data
    for (int y = 0; y < Y_RESOLUTION; y++) {
        for (int x = 0; x < X_RESOLUTION; x++) {
            double pixelValue = imageData[y*X_RESOLUTION + x];
            outputFile << (int)(std::min(1.0, pixelValue) * 255) << " ";
            outputFile << (int)(std::min(1.0, pixelValue) * 255) << " ";
            outputFile << (int)(std::min(1.0, pixelValue) * 255) << "  ";
        }
        outputFile << '\n';
    }
    outputFile.close();
}


__device__ bool function(const Vector& position) {
   float scale = 0.86;
    if (position.y < -10) return true;
    if (position.x * position.x / 2 + position.y * position.y / 10 + position.z * position.x > 5) return false;
    float value = std::sin((position.y + position.x + position.z) * scale) * std::sin((position.y * position.x * position.z) * scale);
    if (value > 0.75) return true;
    return false;
}

// Taken from https://stackoverflow.com/questions/11832202/cuda-random-number-generating?rq=1
__device__ float generateRandom( curandState* globalState, int ind) {
    curandState localState = globalState[ind];
    float RANDOM = curand_uniform( &localState );
    globalState[ind] = localState;
    return RANDOM;
}

__device__ Vector getNormal(Vector& position, Vector& direction) {
    Vector axis1 = direction.cross(Vector(0, 1, 0)).unit();
    Vector axis2 = direction.cross(axis1).unit();
    double angle1 = PI / 2, angle2 = PI / 2;
    Vector point1, point2;
    for (double step = PI / 2; step > BINARY_SEARCH_DETAIL; step *= 0.5) {
        point1 = direction * std::cos(angle1) * NORMAL_BRANCH_DISTANCE + axis1 * std::sin(angle1) * NORMAL_BRANCH_DISTANCE;
        point2 = direction * std::cos(angle2) * NORMAL_BRANCH_DISTANCE + axis2 * std::sin(angle2) * NORMAL_BRANCH_DISTANCE;
        if (function(position + point1)) {angle1 -= step;}
        else {angle1 += step;}
        if (function(position + point2)) {angle2 -= step;}
        else {angle2 += step;}
    }
    Vector normal = point1.cross(point2).unit();
    if (normal.dot(direction) < 0) return normal * -1;
    return normal;
}

__device__ double rayCast(const Vector& position, const Vector& direction, float length, bool type) {
    for (double distance = 0; distance < length; distance += (1.0/RAY_DETAIL)) {
        if (function(position + direction * distance) == type) {
            for (double step = 0.5 / RAY_DETAIL; step > BINARY_SEARCH_DETAIL; step *= 0.5) {
                if (function(position + direction * distance) == type) {distance -= step;}
                else {distance += step;}
            }
            return distance;
        }
    }
    return -1;
}

__device__ Vector getVectorInUnitSphere(curandState *randomState, int index) {
    float x = generateRandom(randomState, index) * 2 - 1;
    float y = generateRandom(randomState, index) * 2 - 1;
    float z = generateRandom(randomState, index) * 2 - 1;
    while (x * x + y * y + z * z > 1) {
        x = generateRandom(randomState, index) * 2 - 1;
        y = generateRandom(randomState, index) * 2 - 1;
        z = generateRandom(randomState, index) * 2 - 1;
    }
    return Vector(x, y, z);
}

__device__ double calculatePointLight(Vector position, Vector normal, Vector light, curandState *randomState, int index) {
    double lightValue = 0;
    for (int sample = 0; sample < LIGHTING_SAMPLES; sample++) {
        Vector lightDirection = light + getVectorInUnitSphere(randomState, index) * 2 - position;
        float lightDistance = lightDirection.magnitude();
        lightDirection = lightDirection.unit();

        double distance = rayCast(position + lightDirection * 0.01, lightDirection, lightDistance, true);
        if (distance < 0) lightValue += std::abs(normal.dot(lightDirection));
    }
    return lightValue / LIGHTING_SAMPLES;
}

__device__ double lightCast(Vector position, Vector direction, int deapth, curandState *randomState, int index) {
    double distance = rayCast(position, direction, RAY_LENGTH, true);
    
    if (distance > 0) {
        
        Vector hitPosition = position + direction * distance;
        Vector normal = getNormal(hitPosition, direction);
        Vector light1 = Vector(0, 0, 0);
        double lightValue = calculatePointLight(hitPosition, normal, light1, randomState, index);
        
        return lightValue;
    }
    return 0;
}

__global__ void render(float *image, curandState *randomState) {
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    
    Vector look(0, 0, 0);
    Vector eye(20, 10, 16);
    
    Vector cameraDirection = (look - eye).unit();
    Vector right = cameraDirection.cross(Vector(0, 1, 0)).unit();
    Vector up = right.cross(cameraDirection).unit(); 
    
    for (int i = index; i < (X_RESOLUTION * Y_RESOLUTION); i += stride) {
        int x = i % X_RESOLUTION;
        int y = i / Y_RESOLUTION;
        double xPosition = (x * 1.0 / (X_RESOLUTION-1)) * (CAMERA_X_END - CAMERA_X_START) + CAMERA_X_START;
        double yPosition = (y * 1.0 / (Y_RESOLUTION-1)) * (CAMERA_Y_END - CAMERA_Y_START) + CAMERA_Y_START;
        Vector position = eye + (cameraDirection * CAMERA_DISTANCE) + (up * yPosition) + (right * xPosition);
        Vector direction = (position - eye).unit();
        image[i] = lightCast(position, direction, 2, randomState, index);
    }
}

__global__ void setup_kernel(curandState *state){
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  curand_init(RANDOM_SEED, index, 0, &state[index]);
}

int main(int numberArguments, char* argumentValues[]) {
    if (numberArguments < 2) return 1;
    std::string imageName(argumentValues[1]);
    float *image;
    
    int blockSize = 128;
    int numBlocks = 10;
    
    // Use that good good unified memory
    cudaMallocManaged(&image, X_RESOLUTION * Y_RESOLUTION * sizeof(float));
    
    curandState *randomState;
    cudaMalloc(&randomState, numBlocks * blockSize * sizeof(curandState));
    setup_kernel<<<numBlocks,blockSize>>>(randomState);
    
    checkCudaErrors(cudaDeviceSynchronize());
    
    // Grid stride implementation from https://devblogs.nvidia.com/cuda-pro-tip-write-flexible-kernels-grid-stride-loops/
    render<<<numBlocks, blockSize>>>(image, randomState);
    
    // Make it so
    checkCudaErrors(cudaDeviceSynchronize());
    
    writeImageOut(imageName, image);
    
    cudaFree(image);
    
    return 0;
}
