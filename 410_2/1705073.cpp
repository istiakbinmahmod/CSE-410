#include <bits/stdc++.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include "bitmap_image.hpp"
using namespace std;

class Point
{
public:
    double x, y, z, w;
    Point()
    {
        x = y = z = 0.0;
        w = 1.0;
    }
    Point(double x, double y, double z, double w)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
    }
    void scale()
    {
        x /= w;
        y /= w;
        z /= w;
        w = 1.0;
    }
    void normalize()
    {
        double length = sqrt(x * x + y * y + z * z);
        x /= length;
        y /= length;
        z /= length;
    }
};

Point subPoint(Point a, Point b)
{
    Point c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    c.z = a.z - b.z;
    return c;
}

Point addPoint(Point a, Point b)
{
    Point c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    c.z = a.z + b.z;
    return c;
}

Point gradientProduct(Point a, double b)
{
    Point c;
    c.x = a.x * b;
    c.y = a.y * b;
    c.z = a.z * b;
    return c;
}

Point crossProduct(Point a, Point b)
{
    Point c;
    c.x = a.y * b.z - a.z * b.y;
    c.y = a.z * b.x - a.x * b.z;
    c.z = a.x * b.y - a.y * b.x;
    return c;
}

Point dotProduct(Point a, Point b)
{
    Point c;
    c.x = a.x * b.x;
    c.y = a.y * b.y;
    c.z = a.z * b.z;
    return c;
}

class Transformation
{
public:
    double matrix[4][4];
    Transformation()
    {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                matrix[i][j] = 0.0;
        matrix[0][0] = 1.0;
        matrix[1][1] = 1.0;
        matrix[2][2] = 1.0;
        matrix[3][3] = 1.0;
    }
};

Transformation translationMatrix(double tx, double ty, double tz)
{
    Transformation t;
    t.matrix[0][3] = tx;
    t.matrix[1][3] = ty;
    t.matrix[2][3] = tz;
    return t;
}

Transformation scalingMatrix(double sx, double sy, double sz)
{
    Transformation t;
    t.matrix[0][0] = sx;
    t.matrix[1][1] = sy;
    t.matrix[2][2] = sz;
    return t;
}

Point rodriguesFormula(Point x, double angle, Point a)
{
    Point rx(0.0, 0.0, 0.0, 0.0);
    rx = addPoint(rx, gradientProduct(x, cos(angle)));
    rx = addPoint(rx, gradientProduct(dotProduct(a, dotProduct(a, x)), (1.0 - cos(angle))));
    rx = addPoint(rx, gradientProduct(crossProduct(a, x), sin(angle)));
    return rx;
}

Transformation rotationMatrix(double angle, double ax, double ay, double az)
{
    angle = angle * M_PI / 180.0;
    Transformation t;
    Point a(ax, ay, az, 1.0);
    a.normalize();
    Point c1(1.0, 0.0, 0.0, 1.0);
    Point c2(0.0, 1.0, 0.0, 1.0);
    Point c3(0.0, 0.0, 1.0, 1.0);
    c1 = rodriguesFormula(c1, angle, a);
    c2 = rodriguesFormula(c2, angle, a);
    c3 = rodriguesFormula(c3, angle, a);
    t.matrix[0][0] = c1.x;
    t.matrix[0][1] = c2.x;
    t.matrix[0][2] = c3.x;
    t.matrix[1][0] = c1.y;
    t.matrix[1][1] = c2.y;
    t.matrix[1][2] = c3.y;
    t.matrix[2][0] = c1.z;
    t.matrix[2][1] = c2.z;
    t.matrix[2][2] = c3.z;
    return t;
}

Transformation multiplyTransformation(Transformation a, Transformation b)
{
    Transformation c;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
        {
            c.matrix[i][j] = 0.0;
            for (int k = 0; k < 4; k++)
                c.matrix[i][j] += a.matrix[i][k] * b.matrix[k][j];
        }
    return c;
}

Point multiplyTransformationWithPoint(Transformation a, Point b)
{
    Point c(0.0, 0.0, 0.0, 0.0);
    c.x = a.matrix[0][0] * b.x + a.matrix[0][1] * b.y + a.matrix[0][2] * b.z + a.matrix[0][3] * b.w;
    c.y = a.matrix[1][0] * b.x + a.matrix[1][1] * b.y + a.matrix[1][2] * b.z + a.matrix[1][3] * b.w;
    c.z = a.matrix[2][0] * b.x + a.matrix[2][1] * b.y + a.matrix[2][2] * b.z + a.matrix[2][3] * b.w;
    c.w = a.matrix[3][0] * b.x + a.matrix[3][1] * b.y + a.matrix[3][2] * b.z + a.matrix[3][3] * b.w;
    return c;
}

Transformation viewMatrix(Point eye, Point look, Point up)
{
    Transformation ans, tAns;
    Point l = subPoint(look, eye);
    l.normalize();
    Point r = crossProduct(l, up);
    r.normalize();
    Point u = crossProduct(r, l);
    u.normalize();
    tAns = translationMatrix(-eye.x, -eye.y, -eye.z);
    ans.matrix[0][0] = r.x;
    ans.matrix[0][1] = r.y;
    ans.matrix[0][2] = r.z;
    ans.matrix[0][3] = 0.0;
    ans.matrix[1][0] = u.x;
    ans.matrix[1][1] = u.y;
    ans.matrix[1][2] = u.z;
    ans.matrix[1][3] = 0.0;
    ans.matrix[2][0] = -l.x;
    ans.matrix[2][1] = -l.y;
    ans.matrix[2][2] = -l.z;
    ans.matrix[2][3] = 0.0;
    ans.matrix[3][0] = 0.0;
    ans.matrix[3][1] = 0.0;
    ans.matrix[3][2] = 0.0;
    ans.matrix[3][3] = 1.0;
    ans = multiplyTransformation(ans, tAns);
    return ans;
}

Transformation projectionMatrix(double fovY, double aspectRatio, double near, double far)
{
    Transformation ans;
    double fovX = fovY * aspectRatio;
    double t = near * tan(M_PI * fovY / 360.0);
    double r = near * tan(M_PI * fovX / 360.0);
    ans.matrix[0][0] = near / r;
    ans.matrix[0][1] = 0.0;
    ans.matrix[0][2] = 0.0;
    ans.matrix[0][3] = 0.0;
    ans.matrix[1][0] = 0.0;
    ans.matrix[1][1] = near / t;
    ans.matrix[1][2] = 0.0;
    ans.matrix[1][3] = 0.0;
    ans.matrix[2][0] = 0.0;
    ans.matrix[2][1] = 0.0;
    ans.matrix[2][2] = -(far + near) / (far - near);
    ans.matrix[2][3] = -2.0 * far * near / (far - near);
    ans.matrix[3][0] = 0.0;
    ans.matrix[3][1] = 0.0;
    ans.matrix[3][2] = -1.0;
    ans.matrix[3][3] = 0.0;
    return ans;
}

void printMatrix(Transformation t)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
            cout << t.matrix[i][j] << " ";
        cout << endl;
    }
    cout << endl;
}

int main()
{
    ifstream input;
    ofstream output;
    string inputFile = "./testcase/1/";

    // stage 1
    string inp, outp;
    inp = inputFile + "scene.txt";
    outp = inputFile + "my_stage1.txt";
    input.open(inp);
    output.open(outp);
    double eyeX, eyeY, eyeZ,
        lookX, lookY, lookZ,
        upX, upY, upZ,
        fovY, aspectRatio, near, far;
    input >> eyeX >> eyeY >> eyeZ >>
        lookX >> lookY >> lookZ >>
        upX >> upY >> upZ >>
        fovY >> aspectRatio >> near >> far;
    stack<Transformation> transformationStack;
    transformationStack.push(Transformation());
    string command;
    int triangleCount = 0;
    while (true)
    {
        input >> command;
        if (command == "triangle")
        {
            triangleCount++;
            Point a, b, c;
            input >> a.x >> a.y >> a.z >>
                b.x >> b.y >> b.z >>
                c.x >> c.y >> c.z;
            a = multiplyTransformationWithPoint(transformationStack.top(), a);
            a.scale();
            b = multiplyTransformationWithPoint(transformationStack.top(), b);
            b.scale();
            c = multiplyTransformationWithPoint(transformationStack.top(), c);
            c.scale();
            output << std::fixed << std::setprecision(7);
            output << a.x << " " << a.y << " " << a.z << endl
                   << b.x << " " << b.y << " " << b.z << endl
                   << c.x << " " << c.y << " " << c.z << endl
                   << endl;
        }
        else if (command == "translate")
        {
            double tx, ty, tz;
            input >> tx >> ty >> tz;
            Transformation t = multiplyTransformation(transformationStack.top(), translationMatrix(tx, ty, tz));
            transformationStack.pop();
            transformationStack.push(t);
            printMatrix(t);
        }
        else if (command == "scale")
        {
            double sx, sy, sz;
            input >> sx >> sy >> sz;
            Transformation t = multiplyTransformation(transformationStack.top(), scalingMatrix(sx, sy, sz));
            transformationStack.pop();
            transformationStack.push(t);
            printMatrix(t);
        }
        else if (command == "rotate")
        {
            double angle, x, y, z;
            input >> angle >> x >> y >> z;
            Transformation t = multiplyTransformation(transformationStack.top(), rotationMatrix(angle, x, y, z));
            transformationStack.pop();
            transformationStack.push(t);
            printMatrix(t);
        }
        else if (command == "push")
        {
            transformationStack.push(transformationStack.top());
        }
        else if (command == "pop")
        {
            transformationStack.pop();
        }
        else if (command == "end")
        {
            break;
        }
    }
    input.close();
    output.close();

    // stage 2
    inp = inputFile + "my_stage1.txt";
    outp = inputFile + "my_stage2.txt";
    input.open(inp);
    output.open(outp);
    for (int i = 0; i < triangleCount; i++)
    {
        Point p1, p2, p3;
        input >> p1.x >> p1.y >> p1.z >>
            p2.x >> p2.y >> p2.z >>
            p3.x >> p3.y >> p3.z;
        Transformation t = transformationStack.top();
        t = multiplyTransformation(t, viewMatrix(Point(eyeX, eyeY, eyeZ, 1.0), Point(lookX, lookY, lookZ, 1.0), Point(upX, upY, upZ, 1.0)));
        p1 = multiplyTransformationWithPoint(t, p1);
        p2 = multiplyTransformationWithPoint(t, p2);
        p3 = multiplyTransformationWithPoint(t, p3);
        p1.scale();
        p2.scale();
        p3.scale();
        output << std::fixed << std::setprecision(7);
        output << p1.x << " " << p1.y << " " << p1.z << endl
               << p2.x << " " << p2.y << " " << p2.z << endl
               << p3.x << " " << p3.y << " " << p3.z << endl
               << endl;
    }

    input.close();
    output.close();

    // stage 3
    inp = inputFile + "my_stage2.txt";
    outp = inputFile + "my_stage3.txt";
    input.open(inp);
    output.open(outp);
    for (int i = 0; i < triangleCount; i++)
    {
        Point p1, p2, p3;
        input >> p1.x >> p1.y >> p1.z >>
            p2.x >> p2.y >> p2.z >>
            p3.x >> p3.y >> p3.z;
        Transformation t = transformationStack.top();
        t = multiplyTransformation(t, projectionMatrix(fovY, aspectRatio, near, far));
        p1 = multiplyTransformationWithPoint(t, p1);
        p2 = multiplyTransformationWithPoint(t, p2);
        p3 = multiplyTransformationWithPoint(t, p3);
        p1.scale();
        p2.scale();
        p3.scale();
        output << std::fixed << std::setprecision(7);
        output << p1.x << " " << p1.y << " " << p1.z << endl
               << p2.x << " " << p2.y << " " << p2.z << endl
               << p3.x << " " << p3.y << " " << p3.z << endl
               << endl;
    }
}