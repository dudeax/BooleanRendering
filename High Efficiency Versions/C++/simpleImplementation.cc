#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include<stdlib.h>

#define PI 3.141592653589793238463

// Camera Constants
// TODO: Turn these bad boys into variables someday
#define X_RESOLUTION 256
#define Y_RESOLUTION 256

#define CAMERA_X_START 1.0
#define CAMERA_Y_START 1.0
#define CAMERA_X_END -1.0
#define CAMERA_Y_END -1.0
#define CAMERA_DISTANCE 2.0

#define RAY_LENGTH 50.0
#define RAY_DETAIL 400.0 // Checks per unit
#define BINARY_SEARCH_DETAIL 0.0000001
#define NORMAL_BRANCH_DISTANCE 0.001

#define LIGHTING_SAMPLES 100

class Vector;

class Vector { // TODO: Pull this out
    public:
    double x, y, z;
    Vector() {}
    Vector(double value) {
        this->x = value;
        this->y = value;
        this->z = value;
    }
    Vector(double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    Vector operator+(const Vector& other) const {
        Vector result;
        result.x = this->x + other.x;
        result.y = this->y + other.y;
        result.z = this->z + other.z;
        return result;
    }
    Vector operator-(const Vector& other) const {
        Vector result;
        result.x = this->x - other.x;
        result.y = this->y - other.y;
        result.z = this->z - other.z;
        return result;
    }
    Vector operator*(const Vector& other) const {
        Vector result;
        result.x = this->x * other.x;
        result.y = this->y * other.y;
        result.z = this->z * other.z;
        return result;
    }
    Vector operator*(const double& value) const {
        Vector result;
        result.x = this->x * value;
        result.y = this->y * value;
        result.z = this->z * value;
        return result;
    }
    Vector operator/(double value) const {
        Vector result;
        result.x = this->x / value;
        result.y = this->y / value;
        result.z = this->z / value;
        return result;
    }
    Vector operator%(const double& value) const {
        Vector result;
        result.x = std::fmod(this->x, value);
        result.y = std::fmod(this->y, value);
        result.z = std::fmod(this->z, value);
        return result;
    }
    Vector cross(const Vector& other) const {
        Vector result;
        result.x = (this->y * other.z) - (other.y * this->z);
        result.y = (this->z * other.x) - (other.z * this->x);
        result.z = (this->x * other.y) - (other.x * this->y);
        return result;
    }
    double dot(const Vector& other) const {
        return this->x * other.x + this->y * other.y + this->z * other.z;
    }
    double magnitude() const {
        return std::sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
    }
    Vector unit() const {
        return (*this)/this->magnitude();
    }
    double& operator[] (size_t i){
        switch (i) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw "Vector Index Error";
        }
    }

    double operator[] (size_t i) const {
        return (*const_cast<Vector*>(this))[i];
    }
};

void writeImageOut(std::string fileName, std::vector<double> imageData) {
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
            outputFile << (int)(pixelValue * 255) << " ";
            outputFile << (int)(pixelValue * 255) << " ";
            outputFile << (int)(pixelValue * 255) << "  ";
        }
        outputFile << '\n';
    }
    outputFile.close();
}


bool function(const Vector& position) {
    float scale = 0.86;
    if (position.y < -10) return true;
    if (position.x * position.x / 2 + position.y * position.y / 10 + position.z * position.x > 5) return false;
    float value = std::sin((position.y + position.x + position.z) * scale) * std::sin((position.y * position.x * position.z) * scale);
    if (value > 0.75) return true;
    return false;
}


Vector getNormal(const Vector& position, const Vector& direction) {
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

double rayCast(const Vector& position, const Vector& direction, double length) {
    for (double distance = 0; distance <= length; distance += (1.0/RAY_DETAIL)) {
        if (function(position + direction * distance)) {
            for (double step = 1.0 / RAY_DETAIL; step > BINARY_SEARCH_DETAIL; step *= 0.5) {
                if (function(position + direction * distance)) {distance -= step;}
                else {distance += step;}
            }
            return distance;
        }
    }
    return -1;
}

double randomDouble() {
    // TODO: Switch this out to a real generator
    return (1.0 * rand()) / (1.0 * RAND_MAX);
}

Vector getVectorInUnitSphere() {
    float x = randomDouble() * 2 - 1;
    float y = randomDouble() * 2 - 1;
    float z = randomDouble() * 2 - 1;
    while (x * x + y * y + z * z > 1) {
        x = randomDouble() * 2 - 1;
        y = randomDouble() * 2 - 1;
        z = randomDouble() * 2 - 1;
    }
    return Vector(x, y, z);
}

double calculatePointLight(Vector position, Vector normal) {
    double lightValue = 0;
    for (int sample = 0; sample < LIGHTING_SAMPLES; sample++) {
        Vector lightDirection = Vector(0, 0, 0) + getVectorInUnitSphere() * 2 - position;
        float lightDistance = lightDirection.magnitude();
        lightDirection = lightDirection.unit();

        double distance = rayCast(position + lightDirection * 0.01, lightDirection, lightDistance);
        if (distance < 0) lightValue += std::abs(normal.dot(lightDirection));
    }
    return lightValue / LIGHTING_SAMPLES;
}

double lightCast(Vector position, Vector direction) {
    double distance = rayCast(position, direction, RAY_LENGTH);
    
    if (distance > 0) {
        
        Vector hitPosition = position + direction * distance;
        Vector normal = getNormal(hitPosition, direction);
        
        double lightValue = calculatePointLight(hitPosition, normal);
        
        return lightValue;
    }
    return 0;
}

std::vector<double> render(const Vector& look, const Vector& eye, const Vector& up) {
    std::vector<double> imageData = std::vector<double>();
    
    Vector lookDirection = (look - eye).unit();
    Vector right = lookDirection.cross(up).unit();
    Vector correctedUp = right.cross(lookDirection).unit();
    
    for (int y = 0; y < Y_RESOLUTION; y++){
        
       std::cout << ("Rendering: " + std::to_string((y / (1.0 * Y_RESOLUTION)) * 100.0) + "%\n");
       
        for (int x = 0; x < X_RESOLUTION; x++){
            
            double xPosition = (x * 1.0 / (X_RESOLUTION-1)) * (CAMERA_X_END - CAMERA_X_START) + CAMERA_X_START;
            double yPosition = (y * 1.0 / (Y_RESOLUTION-1)) * (CAMERA_Y_END - CAMERA_Y_START) + CAMERA_Y_START;
            
            Vector position = eye + (lookDirection * CAMERA_DISTANCE) + (correctedUp * yPosition) + (right * xPosition);
            Vector direction = (position - eye).unit();
            
            imageData.push_back(lightCast(position, direction));
        }
    }
    return imageData;
}

int main(int numberArguments, char* argumentValues[]) {
    if (numberArguments < 2) return 1;
    std::string imageName(argumentValues[1]);
    std::vector<double> image = render(Vector(0, 0, 0), Vector(20, 10, 16), Vector(0, 1, 0));
    writeImageOut(imageName, image);
    return 0;
}
