#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdlib.h>

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

#define NUMBER_OF_CELLS 100
#define SLICE_AREA 50.0

#define LIGHTING_SAMPLES 100

class Vector;
class RunLength;

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

class RunLength {
    public:
    int start, end;
};

bool cellExists(int x, int y, int z) {
    if (x < 0 || x >= NUMBER_OF_CELLS || y < 0 || y >= NUMBER_OF_CELLS || z < 0 || z >= NUMBER_OF_CELLS) return false;
    return true;
}

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

double simpleRayCast(const Vector& position, const Vector& direction, double length) {
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

double rayIntersectsBounds(const Vector& position, const Vector& direction) {
    // Implementation from https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
    double bound = SLICE_AREA / 2.0;
    double tmin = (bound - position.x) / direction.x;
    double tmax = (-bound - position.x) / direction.x;
    if (tmin > tmax) std::swap(tmin, tmax);
    double tymin = (bound - position.y) / direction.y;
    double tymax = (-bound - position.y) / direction.y;
    if (tymin > tymax) std::swap(tymin, tymax);
    if ((tmin > tymax) || (tymin > tmax)) return -1;
    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;
    double tzmin = (bound - position.z) / direction.z;
    double tzmax = (-bound - position.z) / direction.z;
    if (tzmin > tzmax) std::swap(tzmin, tzmax);
    if ((tmin > tzmax) || (tzmin > tmax)) return -1;
    if (tzmin > tmin) tmin = tzmin;
    return tmin;
}

double rayCast(const Vector& position, const Vector& direction, double rayDistance, const bool* cells) {
    // Check if it's in the cells, if so find the starting cell, if not find the starting cell
    // If the cell is empty, find the intersection with the next cell and repeat
    // if the cell is full then do a simple ray cast between this cell and the next
    double bound = SLICE_AREA / 2.0;
    int xCell,yCell, zCell;
    double xT, yT, zT;
    double totalDistance = 0;
    Vector currentPosition;
    
    // Get entry point
    if (position.x > bound || position.x < -bound || position.y > bound || position.y < -bound || position.z > bound || position.z < -bound) {
        double distance = rayIntersectsBounds(position, direction);
        if (distance == -1) return -1;
        currentPosition = position + direction * distance;
    } else currentPosition = Vector(position.x, position.y, position.z);
    
    // Find cells
    xCell = std::min(std::floor(((currentPosition.x / SLICE_AREA) + 0.5) * NUMBER_OF_CELLS), NUMBER_OF_CELLS - 1.0);
    yCell = std::min(std::floor(((currentPosition.y / SLICE_AREA) + 0.5) * NUMBER_OF_CELLS), NUMBER_OF_CELLS - 1.0);
    zCell = std::min(std::floor(((currentPosition.z / SLICE_AREA) + 0.5) * NUMBER_OF_CELLS), NUMBER_OF_CELLS - 1.0);
    
    while (true) {
        double bestDistance; // TODO: For the love of god switch vectors to index
        double offset = 0.00000000001;
        
        xT = (direction.x > 0) ? std::ceil(((currentPosition.x / SLICE_AREA) + 0.5) * NUMBER_OF_CELLS + offset) : std::floor(((currentPosition.x / SLICE_AREA) + 0.5) * NUMBER_OF_CELLS - offset);
        xT = std::abs((xT / NUMBER_OF_CELLS - 0.5) * SLICE_AREA - currentPosition.x) / std::abs(direction.x);
        yT = (direction.y > 0) ? std::ceil(((currentPosition.y / SLICE_AREA) + 0.5) * NUMBER_OF_CELLS + offset) : std::floor(((currentPosition.y / SLICE_AREA) + 0.5) * NUMBER_OF_CELLS - offset);
        yT = std::abs((yT / NUMBER_OF_CELLS - 0.5) * SLICE_AREA - currentPosition.y) / std::abs(direction.y);
        zT = (direction.z > 0) ? std::ceil(((currentPosition.z / SLICE_AREA) + 0.5) * NUMBER_OF_CELLS + offset) : std::floor(((currentPosition.z / SLICE_AREA) + 0.5) * NUMBER_OF_CELLS - offset);
        zT = std::abs((zT / NUMBER_OF_CELLS - 0.5) * SLICE_AREA - currentPosition.z) / std::abs(direction.z);
        
        if (xT < yT && xT < zT) {
            bestDistance = xT;
            xCell += (direction.x > 0)? 1 : -1;
        } else if (yT < xT && yT < zT) {
            bestDistance = yT;
            yCell += (direction.y > 0)? 1 : -1;
        } else {
            bestDistance = zT;
            zCell += (direction.z > 0)? 1 : -1;
        }

        if (!cellExists(xCell, yCell, zCell)) break;
        
        int cellIndex = xCell + yCell * NUMBER_OF_CELLS + zCell * NUMBER_OF_CELLS * NUMBER_OF_CELLS;
        if (cells[cellIndex] == true) {
            double distance = simpleRayCast(currentPosition, direction, bestDistance);
            if (distance != -1) {
                if (totalDistance + distance > rayDistance) return -1;
                return totalDistance + distance;
            }
        }
        totalDistance += bestDistance;
        if (totalDistance > rayDistance) return -1;
        currentPosition = currentPosition + direction * bestDistance;
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

double calculatePointLight(Vector position, Vector normal, const bool* cells) {
    double lightValue = 0;
    for (int sample = 0; sample < LIGHTING_SAMPLES; sample++) {
        Vector lightDirection = Vector(0, 0, 0) + getVectorInUnitSphere() * 2 - position;
        float lightDistance = lightDirection.magnitude();
        lightDirection = lightDirection.unit();

        double distance = rayCast(position + lightDirection * 0.01, lightDirection, lightDistance, cells);
        if (distance < 0) lightValue += std::abs(normal.dot(lightDirection));
    }
    return lightValue / LIGHTING_SAMPLES;
}

double lightCast(Vector position, Vector direction, const bool* cells) {
    double distance = rayCast(position, direction, RAY_LENGTH, cells);
    
    if (distance > 0) {
        
        Vector hitPosition = position + direction * distance;
        Vector normal = getNormal(hitPosition, direction);
        
        double lightValue = calculatePointLight(hitPosition, normal, cells);
        
        return lightValue;
    }
    return 0;
}

std::vector<double> render(const Vector& look, const Vector& eye, const Vector& up, const bool* cells) {
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
            
            imageData.push_back(lightCast(position, direction, cells));
        }
    }
    return imageData;
}

bool* createRunLengths() {
   bool* cells = new bool[NUMBER_OF_CELLS * NUMBER_OF_CELLS * NUMBER_OF_CELLS];
   float cellSize = SLICE_AREA * 1.0 / NUMBER_OF_CELLS;
   int totalFilled = 0;
   
   // Base cell population
   std::cout << ("Building Spacial Map... \n");
   for (int z = 0; z < NUMBER_OF_CELLS; z++) { // TODO: make this more clever
      for (int y = 0; y < NUMBER_OF_CELLS; y++) {
         for (int x = 0; x < NUMBER_OF_CELLS; x++) {
            int cellNumber = x + y * NUMBER_OF_CELLS + z * NUMBER_OF_CELLS * NUMBER_OF_CELLS;
            Vector cellPosition = Vector(cellSize * x - SLICE_AREA / 2 + cellSize / 2, cellSize * y - SLICE_AREA / 2 + cellSize / 2, cellSize * z - SLICE_AREA / 2 + cellSize/2);
            
            bool filled = false;
            
            for (int i = -5; i <= 5; i++) { // TODO: could probably be rolled into one set of loops
               for (int j = -5; j <= 5; j++) {
                  for (int k = -5; k <= 5; k++) {
                     Vector offset = Vector(i / 5.0, j / 5.0, k / 5.0);
                     filled = filled || function(cellPosition + (offset * 0.5));
                  }
               }
            }
            
            cells[cellNumber] = filled;
            if (filled) totalFilled++;
         }
      }
   }
   
   std::cout << "Total Cells Filled: " << totalFilled << " out of " << NUMBER_OF_CELLS * NUMBER_OF_CELLS * NUMBER_OF_CELLS << "\n";
   
   // Check the edges to make sure no values are missed
   int newCellsFound = 1;
   while (newCellsFound > 0) {
      std::cout << ("Expanding Spacial Map... \n");
      newCellsFound = 0;
      for (int z = 0; z < NUMBER_OF_CELLS; z++) {
         for (int y = 0; y < NUMBER_OF_CELLS; y++) {
            for (int x = 0; x < NUMBER_OF_CELLS; x++) {
               int cellNumber = x + y * NUMBER_OF_CELLS + z * NUMBER_OF_CELLS * NUMBER_OF_CELLS;
               if (!cells[cellNumber]) {
                  for (int i = 0; i <= 6; i++) { // TODO: make this code readable...
                     int negative = ((1 & i) == 1) ? -1: 0;
                     int xCheck = ((i < 2) ? 1: 0) * negative;
                     int yCheck = ((i & 2) >> 1) * negative;
                     int zCheck = (i >> 2) * negative;
                     
                     if (cellExists(xCheck + x, yCheck + y, zCheck + z)) {
                        int checkCell = xCheck + x + (yCheck + y) * NUMBER_OF_CELLS + (zCheck + z) * NUMBER_OF_CELLS * NUMBER_OF_CELLS;
                        if (cells[checkCell]) {
                           bool filled = false;
                           for (int j = 0; j < 20; j++) {
                              for (int k = 0; k < 20; k++) {
                                 float xGrid, yGrid, zGrid;
                                 xGrid = (xCheck == 0) ? (j - 10) * cellSize / 20 : xCheck / 2;
                                 int yValue = (xCheck == 0) ? k : j;
                                 yGrid = (yCheck == 0) ? (yValue - 10) * cellSize / 20 : yCheck / 2;
                                 zGrid = (zCheck == 0) ? (z - 10) * cellSize / 20 : zCheck / 2;
                                 
                                 Vector cellPosition = Vector(x - SLICE_AREA / 2 + xGrid + cellSize / 2, y - SLICE_AREA / 2 + yGrid + cellSize / 2, z - SLICE_AREA / 2 + zGrid + cellSize / 2);
                                 filled = filled || function(cellPosition);
                              }
                           }
                           if (filled) {
                              newCellsFound++;
                              cells[cellNumber] = true;
                              break;
                           }
                        }
                     }
                  }
               }
            }
         }
      }
      std::cout << ("Found " + std::to_string(newCellsFound) + " new cells\n");
   }

   bool* finalCells = new bool[NUMBER_OF_CELLS * NUMBER_OF_CELLS * NUMBER_OF_CELLS];
   
   // Add some padding just to be safe...
   std::cout << ("Adding a layer of padding... \n");
        for (int z = 0; z < NUMBER_OF_CELLS; z++) {
             for (int y = 0; y < NUMBER_OF_CELLS; y++) {
                for (int x = 0; x < NUMBER_OF_CELLS; x++) {
                   int cellNumber = x + y * NUMBER_OF_CELLS + z * NUMBER_OF_CELLS * NUMBER_OF_CELLS;
                   if (!cells[cellNumber]) {
                        for (int i = -1; i <= 1; i++) {
                            for (int j = -1; j <= 1; j++) {
                                for (int k = -1; k <= 1; k++) {
                                    if (cellExists(x + i, y + j, z + k)) {
                                        int checkNumber = (x + i) + (y + j) * NUMBER_OF_CELLS + (z + k) * NUMBER_OF_CELLS * NUMBER_OF_CELLS;
                                        if (cells[checkNumber]) finalCells[cellNumber] = true;
                                    }
                                }
                            }
                        }
                   } else {
                       finalCells[cellNumber] = true;
                   }
                }
         }
    }
    delete cells;
    return finalCells;
}

int main(int numberArguments, char* argumentValues[]) {
    if (numberArguments < 2) return 1;
    bool* cells = createSpacialMap();
    std::string imageName(argumentValues[1]);
    std::vector<double> image = render(Vector(0, 0, 0), Vector(20, 10, 16), Vector(0, 1, 0), cells);
    writeImageOut(imageName, image);
    return 0;
}
