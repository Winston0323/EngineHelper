//This file strictly follows the book Foundations of Physically Based Modeling & Animation
//Here I sincerely express my appreciate to Dr. John C. Keyser and Dr. Donald H.House

#ifndef OPENGLHELPER_H
#define OPENGLHELPER_H

#define EPSILON 1e-4f
#include "core.h"
#include <iostream>
#include <cmath>
#include <optional>

class OpenGLHelper {
public:
    enum FaceDirection {
        same = 1,
        oppose = 2,
        perpendicular = 3
    };
    /// <summary>
    /// Struct that create a Line
    /// </summary>
    /// <param name = "point"> is a point on the line</param>
    /// <param name = "dir">is the direction vector of the line </param>
    struct Line {
        glm::vec3 point;
        glm::vec3 dir;
    };
    /// <summary>
    /// Struct that create a Plane
    /// </summary>
    /// <param name = "point"> is a point on the line</param>
    /// <param name = "dir">is the direction vector of the line </param>
    struct Plane {
        glm::vec3 normal; // The normal vector of the plane
        float d;          // The plane equation is: Ax + By + Cz + D = 0, where D = - (normal.x * point.x + normal.y * point.y + normal.z * point.z)

        // Constructor using point and normal vector
        Plane(const glm::vec3& point, const glm::vec3& normalVec) {
            normal = glm::normalize(normalVec);
            d = -glm::dot(normal, point);
        }

        // Constructor using three non-collinear points
        Plane(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3) {
            glm::vec3 v1 = p2 - p1;
            glm::vec3 v2 = p3 - p1;
            normal = glm::normalize(glm::cross(v1, v2));
            d = -glm::dot(normal, p1);
        }

        // Constructor using two intersecting lines (points + direction vectors)
        Plane(const glm::vec3& p1, const glm::vec3& dir1, const glm::vec3& p2, const glm::vec3& dir2) {
            normal = glm::normalize(glm::cross(dir1, dir2));
            d = -glm::dot(normal, p1); // Use one of the points
        }

        // Constructor using a line and a point not on the line
        Plane(const glm::vec3& pointOnLine, const glm::vec3& lineDirection, const glm::vec3& pointNotOnLine) {
            glm::vec3 v = pointNotOnLine - pointOnLine;
            normal = glm::normalize(glm::cross(lineDirection, v));
            d = -glm::dot(normal, pointOnLine); // Use the point on the line
        }

        // Method to display the plane equation
        void printPlaneEquation() const {
            std::cout << "Plane equation: " << normal.x << "x + " << normal.y << "y + " << normal.z << "z + " << d << " = 0" << std::endl;
        }
    };
    /// <summary>
    /// Struct that create an triangle, point is defined in counter clockwise
    /// </summary>
    /// <param name="pnt_a"> is a point of the triangle</param>
    /// <param name="pnt_b"> is a point of the triangle</param>
    /// <param name="pnt_c"> is a point of the triangle</param>
    struct Triangle {
        glm::vec3 pnt_a;
        glm::vec3 pnt_b;
        glm::vec3 pnt_c;
    };
    /// <summary>
    /// Struct that create a sphere
    /// </summary>
    /// <param name="center"> is the center of sphere </param>
    /// <param name="radius"> is the radius of sphere </param>

    struct Sphere {
        glm::vec3 center;
        GLfloat radius;
    };

    /// <summary>
    /// project from vector b on to a
    /// </summary>
    /// <param name="vec_a">being projected onto</param>
    /// <param name="vec_b">being projected</param>
    /// <returns> projected result </returns>
    static glm::vec3 vecProj(glm::vec3 vec_a, glm::vec3 vec_b);
    /// <summary>
    /// return the distance and perpendicular between the point and the line
    /// </summary>
    /// <param name="pnt"> point that is being project</param>
    /// <param name="vec"> vector that is being project</param>
    /// <returns> An std::pair, first value is a GLfloat which is distance, 
    /// second one is the perpendicular norm between point and line</returns>
    static std::pair<GLfloat, glm::vec3> PntLineRelation (glm::vec3 pnt, OpenGLHelper::Line line);
    /// <summary>
    /// Get the reflection direction of the in coming vector and base on normal
    /// </summary>
    /// <param name="in_vec">In coming vector</param>
    /// <param name="norm">Out going direction</param>
    /// <returns></returns>
    static glm::vec3 vecReflection(glm::vec3 in_vec, glm::vec3 norm);
    /// <summary>
    /// Find surface norm of the norm, according to right hand rules
    /// </summary>
    /// <param name="vec_a">First vector in counter-clockwise</param>
    /// <param name="vec_b">Second vector in counter-clockwise</param>
    /// <returns>Area value</returns>
    static glm::vec3 findSurfaceNorm(glm::vec3 vec_a, glm::vec3 vec_b);
    /// <summary>
    /// Find triangle area of between acute angle
    /// Vector order doesnt matter
    /// </summary>
    /// <param name="vec_a"> first vector</param>
    /// <param name="vec_b"> second vector</param>
    /// <returns>Area value</returns>
    static GLfloat getTriangleArea(glm::vec3 vec_a, glm::vec3 vec_b);
    /// <summary>
    /// Find triangle area of three point
    /// </summary>
    /// <param name="pnt_1">First point</param>
    /// <param name="pnt_2">Second point</param>
    /// <param name="pnt_3">Third point</param>
    /// <returns>Area value</returns>
    static GLfloat getTriangleArea(glm::vec3 pnt_1, glm::vec3 pnt_2, glm::vec3 pnt_3);
    /// <summary>
    /// Find triangle area of struct Triangle
    /// </summary>
    /// <param name="triangle"></param>
    /// <returns>Area value</returns>
    static GLfloat getTriangleArea(OpenGLHelper::Triangle triangle);
    /// <summary>
    /// Get the cos between vec_a and vec_b
    /// </summary>
    /// <param name="vec_a"></param>
    /// <param name="vec_b"></param>
    /// <returns></returns>
    static GLfloat getCos(glm::vec3 vec_a, glm::vec3 vec_b);
    /// <summary>
    /// Get the sin between vec_a and vec_b
    /// </summary>
    /// <param name="vec_a"></param>
    /// <param name="vec_b"></param>
    /// <returns></returns>
    static GLfloat getSin(glm::vec3 vec_a, glm::vec3 vec_b);
    /// <summary>
    /// Get the acute angle radian and degree between two vector
    /// </summary>
    /// <param name="vec_a"></param>
    /// <param name="vec_b"></param>
    /// <returns>std::pair, first is radian second is degree</returns>
    static std::pair<GLfloat, GLfloat> getDegree(glm::vec3 vec_a, glm::vec3 vec_b);
    /// <summary>
    /// Get the barcentric coordinate of a point according to a triangle
    /// Order is following the order of point inside triangle
    /// </summary>
    /// <param name="point">point that is being check</param>
    /// <param name="triangle">triangle that is checking on</param>
    /// <returns>barcentric corrdinate of the point</returns>
    static glm::vec3 GetPntBarCord(glm::vec3 point, OpenGLHelper::Triangle triangle);
    /// <summary>
    /// Check if a point is inside a triangle
    /// </summary>
    /// <param name="pnt">point that is being check</param>
    /// <param name="tri">triangle that is checking on</param>
    /// <returns>return if the point is inside the triangle</returns>
    static bool pntInPlane(glm::vec3 pnt, OpenGLHelper::Triangle tri);
    /// <summary>
    /// get a point on the plane
    /// </summary>
    /// <param name="pln"></param>
    /// <returns></returns>
    static glm::vec3 getPntOnPln(Plane pln);
    /// <summary>
    /// Get distance from point to a plane
    /// </summary>
    /// <param name="pnt"></param>
    /// <param name="pln"></param>
    /// <returns></returns>
    static GLfloat distance(glm::vec3 pnt, Plane pln);
    /// <summary>
    /// Find missing coordinate of a point that is on the sphere
    /// </summary>
    /// <param name="sphere">Sphere that the point is on</param>
    /// <param name="x"> x value of the point </param>
    /// <param name="y"> y value of the point </param>
    /// <param name="z"> z value of the point </param>
    /// <returns></returns>
    static glm::vec3 findMissingCoordinate(const Sphere& sphere,
        std::optional<float> x,
        std::optional<float> y,
        std::optional<float> z);

    static FaceDirection faceDirection(glm::vec3 vec_a, glm::vec3 vec_b);
};

#endif // OPENGLHELPER_H
