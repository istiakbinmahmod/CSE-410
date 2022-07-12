#include <bits/stdc++.h>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include "../bitmap_image.hpp"

using namespace std;

// std::cout << std::fixed;

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
    Point &operator=(const Point other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }
    void normalize()
    {
        double length = sqrt(x * x + y * y + z * z);
        x /= length;
        y /= length;
        z /= length;
    }
    void scale()
    {
        x /= w;
        y /= w;
        z /= w;
        w = 1.0;
    }
};

class Color
{
public:
    double redValue;
    double greenValue;
    double blueValue;
    Color()
    {
        redValue = 0.0;
        greenValue = 0.0;
        blueValue = 0.0;
    }
    Color(double redValue, double greenValue, double blueValue)
    {
        this->redValue = redValue;
        this->greenValue = greenValue;
        this->blueValue = blueValue;
    }
};

class Triangle
{
public:
    Point points[3];
    Color color;
};

Point addPoint(Point a, Point b)
{
    Point c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    c.z = a.z + b.z;
    return c;
}

Point subPoint(Point a, Point b)
{
    Point c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    c.z = a.z - b.z;
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

Point crossProduct(Point a, Point b)
{
    Point c;
    c.x = a.y * b.z - a.z * b.y;
    c.y = a.z * b.x - a.x * b.z;
    c.z = a.x * b.y - a.y * b.x;
    return c;
}

Point scalarProduct(double a, Point b)
{
    Point c;
    c.x = a * b.x;
    c.y = a * b.y;
    c.z = a * b.z;
    return c;
}

Point rodriguezFormula(Point x, double angle, Point a)
{
    angle = angle * (M_PI / 180.0) * 1.0;
    Point temp;
    temp = scalarProduct(cos(angle), x);
    temp = addPoint(temp, scalarProduct((1 - cos(angle)), dotProduct(a, dotProduct(a, x))));
    temp = addPoint(temp, scalarProduct(sin(angle), crossProduct(a, x)));
    return temp;
}

class Transformation
{
public:
    double matrix[4][4];
    Transformation()
    {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                if (i == j)
                {
                    matrix[i][j] = 1.0;
                }
                else
                {
                    matrix[i][j] = 0.0;
                }
            }
        }
    }
    Transformation(const Transformation &t)
    {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                matrix[i][j] = t.matrix[i][j];
            }
        }
    }
    Point multPoint(Point newP)
    {
        double ans[4];
        for (int i = 0; i < 4; i++)
        {
            ans[i] = 0.0;
            for (int j = 0; j < 4; j++)
            {
                if (j == 0)
                    ans[i] += matrix[i][j] * newP.x;
                else if (j == 1)
                    ans[i] += matrix[i][j] * newP.y;
                else if (j == 2)
                    ans[i] += matrix[i][j] * newP.z;
                else if (j == 3)
                    ans[i] += matrix[i][j] * newP.w;
            }
        }
        return Point(ans[0], ans[1], ans[2], ans[3]);
    }
    Transformation multTransformation(Transformation newT)
    {
        Transformation temp;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                temp.matrix[i][j] = 0.0;
                for (int k = 0; k < 4; k++)
                {
                    temp.matrix[i][j] += matrix[i][k] * newT.matrix[k][j];
                }
            }
        }
        return temp;
    }
    void translateMatrix(double tx, double ty, double tz)
    {
        matrix[0][3] = tx;
        matrix[1][3] = ty;
        matrix[2][3] = tz;
    }
    void scaleMatrix(double sx, double sy, double sz)
    {
        matrix[0][0] = sx;
        matrix[1][1] = sy;
        matrix[2][2] = sz;
    }
    void rotateMatrix(double angle, double ax, double ay, double az)
    {
        Point a(ax, ay, az, 1.0);
        a.normalize();
        Point c1(1.0, 0.0, 0.0, 1.0);
        Point c2(0.0, 1.0, 0.0, 1.0);
        Point c3(0.0, 0.0, 1.0, 1.0);
        c1 = rodriguezFormula(c1, angle, a);
        c2 = rodriguezFormula(c2, angle, a);
        c3 = rodriguezFormula(c3, angle, a);
        matrix[0][0] = c1.x;
        matrix[0][1] = c2.x;
        matrix[0][2] = c3.x;
        matrix[1][0] = c1.y;
        matrix[1][1] = c2.y;
        matrix[1][2] = c3.y;
        matrix[2][0] = c1.z;
        matrix[2][1] = c2.z;
        matrix[2][2] = c3.z;
    }
    void viewMatrix(Point eye, Point look, Point up)
    {
        Point l = subPoint(look, eye);
        l.normalize();
        Point r = crossProduct(l, up);
        r.normalize();
        Point u = crossProduct(r, l);

        Transformation temp;
        temp.translateMatrix(-eye.x, -eye.y, -eye.z);

        matrix[0][0] = r.x;
        matrix[0][1] = r.y;
        matrix[0][2] = r.z;

        matrix[1][0] = u.x;
        matrix[1][1] = u.y;
        matrix[1][2] = u.z;

        matrix[2][0] = -l.x;
        matrix[2][1] = -l.y;
        matrix[2][2] = -l.z;

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                matrix[i][j] = multTransformation(temp).matrix[i][j];
            }
        }
    }
    void projectMatrix(double fovY, double aspectRatio, double near, double far)
    {
        double fovX = fovY * aspectRatio;
        double r = near * tan(fovX * (M_PI / 180.0) / 2.0);
        double t = near * tan(fovY * (M_PI / 180.0) / 2.0);

        matrix[0][0] = near / r;
        matrix[1][1] = near / t;
        matrix[2][2] = -(far + near) / (far - near);
        matrix[2][3] = -(2.0 * far * near) / (far - near);
        matrix[3][2] = -1.0;
        matrix[3][3] = 0.0;
    }
};

int main()
{
    std::cout << std::setprecision(2) << std::fixed;
    ifstream input;
    ofstream output;
    input.open("../1/scene.txt");
    if (!input.is_open())
    {
        cout << "Error opening file" << endl;
        return 0;
    }

    double eyeX, eyeY, eyeZ, lookX, lookY, lookZ, upX, upY, upZ, fovY, aspectRatio, near, far;

    input >> eyeX >> eyeY >> eyeZ >> lookX >> lookY >> lookZ >> upX >> upY >> upZ >> fovY >> aspectRatio >> near >> far;

    // stage 1
    string command;
    stack<Transformation> transformations;
    transformations.push(Transformation());

    output.open("../1/output1.txt");
    if (!output.is_open())
    {
        cout << "Error opening file" << endl;
        return 0;
    }

    bool bullshitCommand = false;
    bool bullshitPop = false;

    int Triangle_Count = 0;
    int pushPopCount = 0;

    while (true)
    {
        input >> command;
        if (command == "triangle")
        {
            Point p1, p2, p3;
            input >> p1.x >> p1.y >> p1.z >> p2.x >> p2.y >> p2.z >> p3.x >> p3.y >> p3.z;
            p1 = transformations.top().multPoint(p1);
            p1.scale();
            p2 = transformations.top().multPoint(p2);
            p2.scale();
            p3 = transformations.top().multPoint(p3);
            p3.scale();
            // output.precision(6);
            output << std::setprecision(7) << std::fixed;
            output << p1.x << " " << p1.y << " " << p1.z << endl
                   << p2.x << " " << p2.y << " " << p2.z << endl
                   << p3.x << " " << p3.y << " " << p3.z << endl
                   << endl;
            Triangle_Count++;
        }
        else if (command == "translate")
        {
            double tx, ty, tz;
            input >> tx >> ty >> tz;
            Transformation temp;
            temp.translateMatrix(tx, ty, tz);
            temp = transformations.top().multTransformation(temp);
            transformations.pop();
            transformations.push(temp);
        }
        else if (command == "scale")
        {
            double sx, sy, sz;
            input >> sx >> sy >> sz;
            Transformation temp;
            temp.scaleMatrix(sx, sy, sz);
            temp = transformations.top().multTransformation(temp);
            transformations.pop();
            transformations.push(temp);
        }
        else if (command == "rotate")
        {
            double angle, ax, ay, az;
            input >> angle >> ax >> ay >> az;
            Transformation temp;
            temp.rotateMatrix(angle, ax, ay, az);
            temp = transformations.top().multTransformation(temp);
            transformations.pop();
            transformations.push(temp);
        }
        else if (command == "push")
        {
            pushPopCount++;
            transformations.push(transformations.top());
        }
        else if (command == "pop")
        {
            if (pushPopCount == 0)
            {
                bullshitPop = true;
            }
            else
            {
                transformations.pop();
                pushPopCount--;
            }
        }
        else if (command == "end")
        {
            break;
        }
        else if (command == "end")
        {
            break;
        }
    }

    input.close();
    output.close();

    // stage 2
    Transformation viewT;
    viewT.viewMatrix(Point(eyeX, eyeY, eyeZ, 1.0),
                     Point(lookX, lookY, lookZ, 1.0),
                     Point(upX, upY, upZ, 1.0));
    input.open("../1/output1.txt");
    output.open("../1/output2.txt");
    for (int i = 0; i < Triangle_Count; i++)
    {
        Point p1, p2, p3;
        input >> p1.x >> p1.y >> p1.z >> p2.x >> p2.y >> p2.z >> p3.x >> p3.y >> p3.z;
        p1 = viewT.multPoint(p1);
        p1.scale();
        p2 = viewT.multPoint(p2);
        p2.scale();
        p3 = viewT.multPoint(p3);
        p3.scale();

        // output.precision(6);
        output << std::setprecision(7) << std::fixed;
        output << p1.x << " " << p1.y << " " << p1.z << endl
               << p2.x << " " << p2.y << " " << p2.z << endl
               << p3.x << " " << p3.y << " " << p3.z << endl
               << endl;
    }

    input.close();
    output.close();

    // stage 3
    Transformation projectionT;
    projectionT.projectMatrix(fovY, aspectRatio, near, far);
    input.open("../1/output2.txt");
    output.open("../1/output3.txt");
    for (int i = 0; i < Triangle_Count; i++)
    {
        Point p1, p2, p3;
        input >> p1.x >> p1.y >> p1.z >> p2.x >> p2.y >> p2.z >> p3.x >> p3.y >> p3.z;
        p1 = projectionT.multPoint(p1);
        p1.scale();
        p2 = projectionT.multPoint(p2);
        p2.scale();
        p3 = projectionT.multPoint(p3);
        p3.scale();

        // output.precision(6);
        output << std::setprecision(7) << std::fixed;
        output << p1.x << " " << p1.y << " " << p1.z << endl
               << p2.x << " " << p2.y << " " << p2.z << endl
               << p3.x << " " << p3.y << " " << p3.z << endl
               << endl;
    }
    input.close();
    output.close();

    // stage 4
    input.open("../1/config.txt");
    output.open("../1/output4.txt");

    int Screen_Width, Screen_Height;
    double Left_Limit_X, Right_Limit_X;
    double Bottom_Limit_Y, Top_Limit_Y;
    double Front_Limit_Z, Rear_Limit_Z;

    input >> Screen_Width >> Screen_Height;
    input >> Left_Limit_X >> Bottom_Limit_Y;
    input >> Front_Limit_Z >> Rear_Limit_Z;

    Right_Limit_X = -Left_Limit_X;
    Top_Limit_Y = -Right_Limit_X;

    double dx = (Right_Limit_X - Left_Limit_X) / Screen_Width;
    double dy = (Top_Limit_Y - Bottom_Limit_Y) / Screen_Height;
    double Top_Y = Top_Limit_Y - dy / 2.0;
    double Left_X = Left_Limit_X + dx / 2.0;
    double Bottom_Y = Bottom_Limit_Y + dy / 2.0;
    double Right_X = Right_Limit_X - dx / 2.0;

    input.close();
    input.open("../1/output3.txt");

    Triangle triangles[Triangle_Count];
    srand(time(0));

    for (int i = 0; i < Triangle_Count; i++)
    {
        Point p1, p2, p3;
        input >> p1.x >> p1.y >> p1.z >> p2.x >> p2.y >> p2.z >> p3.x >> p3.y >> p3.z;
        triangles[i].points[0].x = p1.x;
        triangles[i].points[0].y = p1.y;
        triangles[i].points[0].z = p1.z;
        triangles[i].points[0].scale();
        triangles[i].points[1].x = p2.x;
        triangles[i].points[1].y = p2.y;
        triangles[i].points[1].z = p2.z;
        triangles[i].points[1].scale();
        triangles[i].points[2].x = p3.x;
        triangles[i].points[2].y = p3.y;
        triangles[i].points[2].z = p3.z;
        triangles[i].points[2].scale();
        // triangles[i].points[1] = p2;
        // triangles[i].points[2] = p3;
        triangles[i].color.redValue = rand() % 255;
        triangles[i].color.greenValue = rand() % 255;
        triangles[i].color.blueValue = rand() % 255;
    }
    input.close();

    // double Z_Buffer[Screen_Width][Screen_Height];
    // Color intensity_Buffer[Screen_Width][Screen_Height];

    double **Z_Buffer = new double *[Screen_Height];
    Color **intensity_Buffer = new Color *[Screen_Height];
    for (int i = 0; i < Screen_Width; i++)
    {
        Z_Buffer[i] = new double[Screen_Width];
        intensity_Buffer[i] = new Color[Screen_Width];
    }

    for (int i = 0; i < Screen_Width; i++)
    {
        for (int j = 0; j < Screen_Height; j++)
        {
            Z_Buffer[i][j] = Rear_Limit_Z;
            intensity_Buffer[i][j] = Color(0, 0, 0);
        }
    }

    // for (int i = 0; i < Screen_Width; i++)
    // {
    //     for (int j = 0; j < Screen_Height; j++)
    //     {
    //         Z_Buffer[i][j] = 0.0;
    //         intensity_Buffer[i][j].redValue = 0;
    //         intensity_Buffer[i][j].greenValue = 0;
    //         intensity_Buffer[i][j].blueValue = 0;
    //     }
    // }

    for (int i = 0; i < Triangle_Count; i++)
    {
        int Top_Scanline, Bottom_Scanline;
        double maxY_From_Points = max(triangles[i].points[0].y, max(triangles[i].points[1].y, triangles[i].points[2].y));
        double minY_From_Points = min(triangles[i].points[0].y, min(triangles[i].points[1].y, triangles[i].points[2].y));
        // double midY_From_Points = triangles[i].points[0].y + triangles[i].points[1].y + triangles[i].points[2].y - maxY_From_Points - minY_From_Points;
        // int maxX_Pos_Point_Index, midX_Pos_Point_Index, minX_Pos_Point_Index;
        // for (int j = 0; j < 3; j++)
        // {
        //     if (triangles[i].points[j].y == maxY_From_Points)
        //     {
        //         maxX_Pos_Point_Index = j;
        //     }
        //     if (triangles[i].points[j].y == midY_From_Points)
        //     {
        //         midX_Pos_Point_Index = j;
        //     }
        //     if (triangles[i].points[j].y == minY_From_Points)
        //     {
        //         minX_Pos_Point_Index = j;
        //     }
        // }

        if (maxY_From_Points >= Top_Y)
        {
            Top_Scanline = 0;
        }
        else
        {
            Top_Scanline = (int)round((Top_Y - maxY_From_Points) / dy);
        }
        if (minY_From_Points <= Bottom_Y)
        {
            Bottom_Scanline = Screen_Height - 1;
        }
        else
        {
            Bottom_Scanline = Screen_Height - 1 - (int)round((minY_From_Points - Bottom_Y) / dy);
        }
        for (int row_no = Top_Scanline; row_no <= Bottom_Scanline; row_no++)
        {
            double ys = Top_Y - row_no * dy;
            Point p1, p2, p3;
            if (ys <= triangles[i].points[0].y && ys >= triangles[i].points[1].y && ys >= triangles[i].points[2].y)
            {
                p1.x = triangles[i].points[0].x;
                p1.y = triangles[i].points[0].y;
                p1.z = triangles[i].points[0].z;
                p2.x = triangles[i].points[1].x;
                p2.y = triangles[i].points[1].y;
                p2.z = triangles[i].points[1].z;
                p3.x = triangles[i].points[2].x;
                p3.y = triangles[i].points[2].y;
                p3.z = triangles[i].points[2].z;
            }
            else if (ys >= triangles[i].points[0].y && ys <= triangles[i].points[1].y && ys <= triangles[i].points[2].y)
            {
                p1.x = triangles[i].points[0].x;
                p1.y = triangles[i].points[0].y;
                p1.z = triangles[i].points[0].z;
                p2.x = triangles[i].points[1].x;
                p2.y = triangles[i].points[1].y;
                p2.z = triangles[i].points[1].z;
                p3.x = triangles[i].points[2].x;
                p3.y = triangles[i].points[2].y;
                p3.z = triangles[i].points[2].z;
            }
            else if (ys <= triangles[i].points[1].y && ys >= triangles[i].points[0].y && ys >= triangles[i].points[2].y)
            {
                p1.x = triangles[i].points[1].x;
                p1.y = triangles[i].points[1].y;
                p1.z = triangles[i].points[1].z;
                p2.x = triangles[i].points[0].x;
                p2.y = triangles[i].points[0].y;
                p2.z = triangles[i].points[0].z;
                p3.x = triangles[i].points[2].x;
                p3.y = triangles[i].points[2].y;
                p3.z = triangles[i].points[2].z;
            }
            else if (ys >= triangles[i].points[1].y && ys <= triangles[i].points[0].y && ys <= triangles[i].points[2].y)
            {
                p1.x = triangles[i].points[1].x;
                p1.y = triangles[i].points[1].y;
                p1.z = triangles[i].points[1].z;
                p2.x = triangles[i].points[0].x;
                p2.y = triangles[i].points[0].y;
                p2.z = triangles[i].points[0].z;
                p3.x = triangles[i].points[2].x;
                p3.y = triangles[i].points[2].y;
                p3.z = triangles[i].points[2].z;
            }
            else if (ys <= triangles[i].points[2].y && ys >= triangles[i].points[0].y && ys >= triangles[i].points[1].y)
            {
                p1.x = triangles[i].points[2].x;
                p1.y = triangles[i].points[2].y;
                p1.z = triangles[i].points[2].z;
                p2.x = triangles[i].points[0].x;
                p2.y = triangles[i].points[0].y;
                p2.z = triangles[i].points[0].z;
                p3.x = triangles[i].points[1].x;
                p3.y = triangles[i].points[1].y;
                p3.z = triangles[i].points[1].z;
            }
            else if (ys >= triangles[i].points[2].y && ys <= triangles[i].points[0].y && ys <= triangles[i].points[1].y)
            {
                p1.x = triangles[i].points[2].x;
                p1.y = triangles[i].points[2].y;
                p1.z = triangles[i].points[2].z;
                p2.x = triangles[i].points[0].x;
                p2.y = triangles[i].points[0].y;
                p2.z = triangles[i].points[0].z;
                p3.x = triangles[i].points[1].x;
                p3.y = triangles[i].points[1].y;
                p3.z = triangles[i].points[1].z;
            }

            // cout << "p1: " << p1.x << " " << p1.y << endl;
            // cout << "p2: " << p2.x << " " << p2.y << endl;
            // cout << "p3: " << p3.x << " " << p3.y << endl;

            double xa = p1.x;
            double xb = p1.x;
            if (p2.y != p1.y)
            {
                xa += (p2.x - p1.x) * (ys - p1.y) / (p2.y - p1.y);
            }
            if (p3.y != p1.y)
            {
                xb += (p3.x - p1.x) * (ys - p1.y) / (p3.y - p1.y);
            }
            if (xa > xb)
                swap(xa, xb);

            // cout << "xa :" << xa << " and xb : " << xb << endl;
            double za = p1.z;
            double zb = p1.z;

            if (p2.y != p1.y)
            {
                za += (p2.z - p1.z) * (ys - p1.y) / (p2.y - p1.y);
            }
            if (p3.y != p1.y)
            {
                zb += (p3.z - p1.z) * (ys - p1.y) / (p3.y - p1.y);
            }
            // cout << za << " " << zb << endl;
            double Left_Intersecting_Column, Right_Intersecting_Column;
            if (xa <= Left_X)
            {
                Left_Intersecting_Column = 0;
            }
            else
            {
                Left_Intersecting_Column = (int)round((xa - Left_X) / dx);
            }
            if (xb >= Right_X)
            {
                Right_Intersecting_Column = Screen_Width - 1;
            }
            else
            {
                Right_Intersecting_Column = Screen_Width - 1 - (int)round((Right_X - xb) / dx);
            }

            double Constant_Term = 0.0;
            if (xb != xa)
            {
                Constant_Term = (dx * (zb - za)) / (xb - xa);
            }

            // cout << "Constant Term is : " << Constant_Term << endl;

            double zp;
            for (int col_no = Left_Intersecting_Column; col_no <= Right_Intersecting_Column; col_no++)
            {
                double xp = Left_X + col_no * dx;
                if (col_no == Left_Intersecting_Column)
                {
                    zp = za;
                    if (xb != xa)
                    {
                        zp += (zb - za) * (xp - xa) / (xb - xa);
                    }
                }
                else
                {
                    zp += Constant_Term;
                }
                if (zp > Front_Limit_Z && zp < Z_Buffer[row_no][col_no])
                {
                    Z_Buffer[row_no][col_no] = zp;
                    intensity_Buffer[row_no][col_no].redValue = triangles[i].color.redValue;
                    intensity_Buffer[row_no][col_no].greenValue = triangles[i].color.greenValue;
                    intensity_Buffer[row_no][col_no].blueValue = triangles[i].color.blueValue;
                }
            }
        }
    }

    for (int i = 0; i < Screen_Width; i++)
    {
        for (int j = 0; j < Screen_Height; j++)
        {
            output << Z_Buffer[i][j] << " ";
        }
        output << endl;
    }
    output << endl;

    /* saving outputs */
    bitmap_image bitmapImage(Screen_Width, Screen_Height);

    for (int row = 0; row < Screen_Height; row++)
    {
        for (int column = 0; column < Screen_Width; column++)
        {
            bitmapImage.set_pixel(column, row, intensity_Buffer[row][column].redValue, intensity_Buffer[row][column].greenValue, intensity_Buffer[row][column].blueValue);
        }
    }
    bitmapImage.save_image("../1/iistiakl.bmp");

    for (int i = 0; i < Screen_Height; i++)
    {
        delete[] Z_Buffer[i];
        delete[] intensity_Buffer[i];
    }
    delete[] Z_Buffer;
    delete[] intensity_Buffer;
    return 0;
}