// OpenGLHelper.cpp

#include "EngineHelper.h"
glm::vec3 OpenGLHelper::vecProj(glm::vec3 vec_a, glm::vec3 vec_b) {
    //Calculate director vector of vector a 
    glm::vec3 aHat = glm::normalize(vec_a);
    //Use dot product to calculate length of vector b on vector a's direction
    //Then vector multiplication with a director vector
    return glm::dot(aHat, vec_b) * aHat;// (ahat * b) * ahat
}
std::pair<GLfloat, glm::vec3> OpenGLHelper::PntLineRelation(glm::vec3 pnt, OpenGLHelper::Line line) {
    //Create a line b from a random point on line and the given pnt
    glm::vec3 b = line.point - pnt;
    //Project b onto direction of line
    glm::vec3 ba = OpenGLHelper::vecProj(line.dir, b);
    //Calculate the perpendicular vector between point b and the line
    glm::vec3 bperp = b - ba;
    //Distance between point and line is the length of the perpendicular vector
    GLfloat distance = glm::length(bperp);
    return std::make_pair(distance, bperp);
}

glm::vec3 OpenGLHelper::vecReflection(glm::vec3 in_vec, glm::vec3 norm) {
    //get -b by projecting in vector onto reflection normal vector
    norm = glm::normlize(norm);
    glm::vec3 in_vec_proj_norm = OpenGLHelper::vecProj(norm, in_vec); // -b
    glm::vec3 result = in_vec - 2.0f * in_vec_proj_norm; // out_vec = in_vec - 2 * b
    return result;
}

glm::vec3 OpenGLHelper::findSurfaceNorm(glm::vec3 vec_a, glm::vec3 vec_b) {
    //cross product of two vector is the new vector that perpenmdicular to these two vector
    glm::vec3 norm = glm::normalize(glm::cross(vec_a, vec_b));
    return norm;
}

GLfloat OpenGLHelper::getTriangleArea(glm::vec3 vec_a, glm::vec3 vec_b) {
    //0.5 * Magnitude of two vector cross product is the area of the triangle bween these two
    GLfloat area = 0.5f * glm::length(glm::cross(vec_a, vec_b));
    return area;
}

GLfloat OpenGLHelper::getTriangleArea(glm::vec3 pnt_1, glm::vec3 pnt_2, glm::vec3 pnt_3) {
    //create two vector out of three points, they need to be starting from the same point
    glm::vec3 vec_a = pnt_1 - pnt_2;
    glm::vec3 vec_b = pnt_2 - pnt_3;
    GLfloat area = OpenGLHelper::getTriangleArea( vec_a, vec_b);
    return area;
}

GLfloat OpenGLHelper::getTriangleArea(OpenGLHelper::Triangle triangle) {

    return OpenGLHelper::getTriangleArea(triangle.pnt_a, triangle.pnt_b, triangle.pnt_c);
}

glm::vec3 OpenGLHelper::GetPntBarCord(glm::vec3 point, Triangle triangle) {
    GLfloat A = OpenGLHelper::getTriangleArea(triangle);//area of the whole triangle
    //Area that is opposite to point a is the barcentric cord of point A
    GLfloat Au = OpenGLHelper::getTriangleArea(point, triangle.pnt_b, triangle.pnt_c) / A; 
    GLfloat Av = OpenGLHelper::getTriangleArea(point, triangle.pnt_a, triangle.pnt_c) / A;
    //The reset of the area is barcentric cord of point C
    GLfloat Aw =  1 - Au - Av;
    return glm::vec3(Au, Av, Aw);
}

GLfloat OpenGLHelper::getCos(glm::vec3 vec_a, glm::vec3 vec_b) {
    GLfloat result = glm::dot(vec_a, vec_b) / (glm::length(vec_a) * glm::length(vec_b)); //dot(A, B)/||A||||B||
    result = glm::clamp(result, -1.0f, 1.0f);//eliminate numerical error
    return result;
}

GLfloat OpenGLHelper::getSin(glm::vec3 vec_a, glm::vec3 vec_b) {
    GLfloat result = glm::length(glm::cross(vec_a, vec_b)) / (glm::length(vec_a) * glm::length(vec_b)); // cross(A, B)/ ||A||||B||
    result = glm::clamp(result, -1.0f, 1.0f);
    return result;
}

std::pair<GLfloat, GLfloat> OpenGLHelper::getDegree(glm::vec3 vec_a, glm::vec3 vec_b) {
    GLfloat cos = getCos(vec_a, vec_b); // get the cos between a and b
    GLfloat radian = acos(cos);         // get the gradiant by arccos
    GLfloat degree = glm::degrees(radian); //convert to degree
    if (degree > 180.0f) {  // we only want the acute angle
        degree = 360.0f - degree;
    }
    return std::make_pair(radian, degree); //return both radian and degree
}

bool OpenGLHelper::pntInPlane(glm::vec3 pnt, OpenGLHelper::Triangle tri) {
    glm::vec3 bar_cord = OpenGLHelper::GetPntBarCord(pnt, tri); // get the barcentric cord of the point that we are testing
    // if any of the barcord is smaller than zero then it is out of the plain
    if (bar_cord.x < EPSILON || bar_cord.y < EPSILON || bar_cord.z < EPSILON) {
        return false;
    }
    else {
        return true;
    }

}

// Function to compute a point on the plane Ax + By + Cz + D = 0
glm::vec3 OpenGLHelper::getPntOnPln(OpenGLHelper::Plane pln) {
    float A = pln.normal.x;
    float B = pln.normal.y;
    float C = pln.normal.z;
    float D = pln.d;
    glm::vec3 point;

    // Check if C is non-zero to solve for z
    if (C != 0.0f) {
        float x = 1.0f;  // Arbitrarily set x
        float y = 1.0f;  // Arbitrarily set y
        float z = (-(A * x + B * y + D)) / C;
        point = glm::vec3(x, y, z);
        return point;
    }
    // If C == 0, solve for y instead (assuming B != 0)
    else if (B != 0.0f) {
        float x = 1.0f;  // Arbitrarily set x
        float z = 1.0f;  // Arbitrarily set z
        float y = (-(A * x + C * z + D)) / B;
        point = glm::vec3(x, y, z);
        return point;
    }
    // If both B and C are 0, solve for x (assuming A != 0)
    else if (A != 0.0f) {
        float y = 1.0f;  // Arbitrarily set y
        float z = 1.0f;  // Arbitrarily set z
        float x = (-(B * y + C * z + D)) / A;
        point = glm::vec3(x, y, z);
        return point;
    }
    else {
        std::cerr << "Error when getting point for plane" << std::endl;
    }

}

GLfloat OpenGLHelper::distance(glm::vec3 pnt, Plane pln) {
    //need a vector from the point to the plane
    glm::vec3 pntOnPlane = OpenGLHelper::getPntOnPln(pln);  // grab a random point on plane
    glm::vec3 vec = pnt - pntOnPlane;                       //create a vector from the point to the point on the plane
    // the magniture of the vector that is projected by that vector onto the norm of the plane
    GLfloat distance = abs(glm::dot(vec, pln.normal));
    return distance;
}

// Function to solve for one of the coordinates and return a glm::vec3
glm::vec3 OpenGLHelper::findMissingCoordinate(const Sphere& sphere,
    std::optional<float> x,
    std::optional<float> y,
    std::optional<float> z) {
    glm::vec3 result = sphere.center; // Initialize result to the center of the sphere

    // Calculate the radius squared once for efficiency
    float r2 = sphere.radius * sphere.radius;

    if (x.has_value() && y.has_value() && z.has_value()) {
        std::cout << "All coordinates are provided!" << std::endl;
        return result; // No missing coordinate
    }

    if (x.has_value() && y.has_value()) {
        // Solve for z
        float value = r2 - ((x.value() - sphere.center.x) * (x.value() - sphere.center.x) +
            (y.value() - sphere.center.y) * (y.value() - sphere.center.y));
        if (value < 0) {
            std::cout << "Cannot calculate z: negative value under square root." << std::endl;
            return result; // Return center if calculation fails
        }
        result.z = std::sqrt(value) + sphere.center.z; // Set calculated z value
        result.x = x.value(); // Assign x value
        result.y = y.value(); // Assign y value
        return result;
    }
    else if (x.has_value() && z.has_value()) {
        // Solve for y
        float value = r2 - ((x.value() - sphere.center.x) * (x.value() - sphere.center.x) +
            (z.value() - sphere.center.z) * (z.value() - sphere.center.z));
        if (value < 0) {
            std::cout << "Cannot calculate y: negative value under square root." << std::endl;
            return result; // Return center if calculation fails
        }
        result.y = std::sqrt(value) + sphere.center.y; // Set calculated y value
        result.x = x.value(); // Assign x value
        result.z = z.value(); // Assign z value
        return result;
    }
    else if (y.has_value() && z.has_value()) {
        // Solve for x
        float value = r2 - ((y.value() - sphere.center.y) * (y.value() - sphere.center.y) +
            (z.value() - sphere.center.z) * (z.value() - sphere.center.z));
        if (value < 0) {
            std::cout << "Cannot calculate x: negative value under square root." << std::endl;
            return result; // Return center if calculation fails
        }
        result.x = std::sqrt(value) + sphere.center.x; // Set calculated x value
        result.y = y.value(); // Assign y value
        result.z = z.value(); // Assign z value
        return result;
    }
    else {
        std::cout << "At least two coordinates must be provided." << std::endl;
        return result; // Return center if calculation cannot proceed
    }
}
OpenGLHelper::FaceDirection OpenGLHelper::faceDirection(glm::vec3 vec_a, glm::vec3 vec_b) {
    //checking the dot product between two vector
    if (glm::dot(vec_a, vec_b) < 0) { //opposite when dot is negative (Angle is obtuse)
        return FaceDirection::oppose;
    }else if (glm::dot(vec_a, vec_b) == 0) {//perpendicular when dot is zero (Angle is 90 degree)
        return FaceDirection::perpendicular;
    }
    else{
        return FaceDirection::same;//same direction when dot is zero (Angle is acute)
    }
}