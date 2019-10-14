#include "pch.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <iomanip>
using namespace std;
const double PI = 3.141592653589793238463;

class Vector3 {
private:
	float x;
	float y;
	float z;
public:
	Vector3() : x{ 0.0f }, y{ 0.0f }, z{ 0.0f } { };
	~Vector3() { };

	void SetVector3(float _x, float _y, float _z) { x = _x; y = _y; z = _z; };
	void GetVector3() { cout << x << " " << y << " " << z << " "; }
	float length(float x, float y, float z) const { return (x * x + y * y + z * z); }
	float d(float x, float y, float z) const { return sqrt(x * x + y * y + z * z); }
	float GetX() const { return x; }
	float GetY() const { return y; }
	float GetZ() const { return z; }

	void add(const Vector3& v, const Vector3& w, Vector3& sum) {
		sum.SetVector3(v.GetX() + w.GetX(), v.GetY() + w.GetY(), v.GetZ() + w.GetZ());
	}

	void subtract(const Vector3& v, const Vector3& w, Vector3& diff) {
		diff.SetVector3(v.GetX() - w.GetX(), v.GetY() - w.GetY(), v.GetZ() - w.GetZ());
	}

	float dot(const Vector3& v, const Vector3& w) {
		return (v.GetX()*w.GetX() + v.GetY()*w.GetY() + v.GetZ()* w.GetZ());
	}

	//// векторное произведение
	void cross(const Vector3& v, const Vector3& w, Vector3& crossOut) {
		crossOut.SetVector3((v.GetY()*w.GetZ() - v.GetZ()*w.GetY()), (v.GetZ()*w.GetX() - v.GetX()*w.GetZ()), (v.GetX()* w.GetY() - v.GetY()*w.GetX()));
	}

	void normalize(Vector3& v) { v.SetVector3(v.GetX() / v.length(x, y, z), v.GetY() / v.length(x, y, z), v.GetX() / v.length(x, y, z)); };
};

enum Direction
{
	REJECTED = 0,
	RIGHT = 1,
	LEFT = 2
};

float** newMatrix(const int n, const int m)
{
	float** arr = new float*[n];
	for (auto i{ 0 }; i < n; ++i)
		arr[i] = new float[m];
	return arr;
}

void deleteMatrix(float** arr, const int m)
{
	for (auto i{ 0 }; i < m; ++i)
		delete[] arr[i];
	delete[] arr;
}

float** TransposeMatrix(float** matrix, const int n, const int m)
{
	float** transform = newMatrix(n, m);

	for (auto i{ 0 }; i < n; ++i)
		for (auto j{ 0 }; j < m; ++j)
			transform[j][i] = matrix[i][j];

	return transform;
}

Direction testWayPoint(const Vector3& dronePosition, const Vector3& droneForward, const Vector3& targetWaypoint, const Vector3& worldUp) {

	Vector3 VectorFoward;
	Vector3 VectorTarget;
	Vector3 Direct;
	Vector3 Right;
	Vector3 Up;
	VectorFoward.subtract(dronePosition, droneForward, VectorFoward);
	VectorTarget.subtract(targetWaypoint, dronePosition, VectorTarget);
	VectorFoward.subtract(dronePosition, targetWaypoint, Direct);
	VectorFoward.normalize(VectorFoward);
	VectorFoward.cross(worldUp, Direct, Right);
	VectorFoward.cross(Direct, Right, Up);
	Right.normalize(Right);
	Up.normalize(Up);

	float **MatrixTransfer = newMatrix(4, 4);
	MatrixTransfer[0][0] = Right.GetX();	MatrixTransfer[0][1] = Right.GetY();		MatrixTransfer[0][2] = Right.GetZ();	MatrixTransfer[0][3] = 0;
	MatrixTransfer[1][0] = Up.GetX();		MatrixTransfer[1][1] = Up.GetY();			MatrixTransfer[1][2] = Up.GetZ();		MatrixTransfer[1][3] = 0;
	MatrixTransfer[2][0] = Direct.GetX();	MatrixTransfer[2][1] = Direct.GetY();		MatrixTransfer[2][2] = Direct.GetZ();	MatrixTransfer[2][3] = 0;
	MatrixTransfer[3][0] = 0;				MatrixTransfer[3][1] = 0;					MatrixTransfer[3][2] = 0;				MatrixTransfer[3][3] = 1;

	vector<float> Vector4;
	Vector4.reserve(4);

	Vector4.push_back(VectorFoward.GetX());
	Vector4.push_back(VectorFoward.GetY());
	Vector4.push_back(VectorFoward.GetZ());
	//Vector4.push_back(1);

	float len = VectorFoward.length(VectorFoward.GetX(), VectorFoward.GetY(), VectorFoward.GetZ());


	float d = VectorFoward.d(VectorFoward.GetX(), VectorFoward.GetY(), VectorFoward.GetZ());

	float alpha = acos((targetWaypoint.GetY() - dronePosition.GetY()) / sqrt(pow(targetWaypoint.GetY() - dronePosition.GetY(), 2) + pow(targetWaypoint.GetX() - dronePosition.GetX(), 2))) * 180 / PI;

	float **MatrixTransformX = newMatrix(4, 4);
	MatrixTransformX[0][0] = 1;		MatrixTransformX[1][0] = 0;				MatrixTransformX[2][0] = 0;				MatrixTransformX[3][0] = 0;
	MatrixTransformX[0][1] = 0;		MatrixTransformX[1][1] = cos(alpha);	MatrixTransformX[2][1] = sin(alpha);	MatrixTransformX[3][1] = 0;
	MatrixTransformX[0][2] = 0;		MatrixTransformX[1][2] = -sin(alpha);	MatrixTransformX[2][2] = cos(alpha);	MatrixTransformX[3][2] = 0;
	MatrixTransformX[0][3] = 0;		MatrixTransformX[1][3] = 0;				MatrixTransformX[2][3] = 0;				MatrixTransformX[3][3] = 1;

	float **MatrixTransformY = newMatrix(4, 4);
	MatrixTransformY[0][0] = cos(alpha);		MatrixTransformY[1][0] = 0;					MatrixTransformY[2][0] = -sin(alpha);	MatrixTransformY[3][0] = 0;
	MatrixTransformY[0][1] = 0;					MatrixTransformY[1][1] = 1;					MatrixTransformY[2][1] = 0;				MatrixTransformY[3][1] = 0;
	MatrixTransformY[0][2] = sin(alpha);		MatrixTransformY[1][2] = 0;					MatrixTransformY[2][2] = cos(alpha);	MatrixTransformY[3][2] = 0;
	MatrixTransformY[0][3] = 0;					MatrixTransformY[1][3] = 0;					MatrixTransformY[2][3] = 0;				MatrixTransformY[3][3] = 1;

	float **MatrixTransformZ = newMatrix(4, 4);
	MatrixTransformZ[0][0] = cos(alpha);			MatrixTransformZ[1][0] = sin(alpha);			MatrixTransformZ[2][0] = 0;				MatrixTransformZ[3][0] = 0;
	MatrixTransformZ[0][1] = -sin(alpha);			MatrixTransformZ[1][1] = cos(alpha);			MatrixTransformZ[2][1] = 0;				MatrixTransformZ[3][1] = 0;
	MatrixTransformZ[0][2] = 0;						MatrixTransformZ[1][2] = 0;						MatrixTransformZ[2][2] = 1;				MatrixTransformZ[3][2] = 0;
	MatrixTransformZ[0][3] = 0;						MatrixTransformZ[1][3] = 0;						MatrixTransformZ[2][3] = 0;				MatrixTransformZ[3][3] = 1;

	int n = 3;
	float sum = 0.0f;
	float **MatrixTransferTr = TransposeMatrix(MatrixTransfer, 4, 4);
	float **MatrixTransformXTr = TransposeMatrix(MatrixTransformX, 4, 4);
	float **MatrixTransformYTr = TransposeMatrix(MatrixTransformY, 4, 4);
	float **MatrixTransformZTr = TransposeMatrix(MatrixTransformZ, 4, 4);

	for (auto i{ 0 }; i < n; ++i) {
		sum = 0.0f;
		for (auto j{ 0 }; j < n; ++j) {
			sum += MatrixTransfer[i][j] * Vector4[j];
		}
		Vector4[i] = sum;
	}

	for (auto i{ 0 }; i < n; ++i) {
		sum = 0.0f;
		for (auto j{ 0 }; j < n; ++j) {
			sum += MatrixTransformY[i][j] * Vector4[j];
		}
		Vector4[i] = sum;
	}
	for (auto i{ 0 }; i < n; ++i) {
		sum = 0.0f;
		for (auto j{ 0 }; j < n; ++j) {
			sum += MatrixTransformZ[i][j] * Vector4[j];
		}
		Vector4[i] = sum;
	}
	/*
	alpha = acos(VectorFoward.GetX() / len);
	for (auto i{ 0 }; i < n; ++i) {
		sum = 0.0f;
		for (auto j{ 0 }; j < n; ++j) {
			sum += MatrixTransformZ[i][j] * Vector4[j];
		}
		Vector4[i] = sum;
	}

	for (auto i{ 0 }; i < n; ++i) {
		sum = 0.0f;
		for (auto j{ 0 }; j < n; ++j) {
			sum += MatrixTransformYTr[i][j] * Vector4[j];
		}
		Vector4[i] = sum;
	}

	for (auto i{ 0 }; i < n; ++i) {
		sum = 0.0f;
		for (auto j{ 0 }; j < n; ++j) {
			sum += MatrixTransformZTr[i][j] * Vector4[j];
		}
		Vector4[i] = sum;
	}*/

	cout << endl;
	for (auto i{ 0 }; i < n; ++i) {
		cout << setw(10) << setprecision(2) << Vector4[i] << setw(10);
	}
	deleteMatrix(MatrixTransformZTr, 4);
	deleteMatrix(MatrixTransformYTr, 4);
	//	deleteMatrix(MatrixTransformXTr, 4);
	deleteMatrix(MatrixTransferTr, 4);
	deleteMatrix(MatrixTransformY, 4);
	//	deleteMatrix(MatrixTransformX, 4);
	deleteMatrix(MatrixTransfer, 4);
	deleteMatrix(MatrixTransformZ, 4);



	float Radius = fabs(dronePosition.GetX() - targetWaypoint.GetX()) * static_cast<float>(tan(PI / 6.0f));
	cout << Radius;

	if ((0.0f < (targetWaypoint.GetX() - dronePosition.GetX()) <= 10.0f) && (dronePosition.GetY() - Radius <= targetWaypoint.GetY() <= dronePosition.GetY() + Radius) && (sqrt(pow(targetWaypoint.GetY(), 2) + pow(targetWaypoint.GetX(), 2)) < Radius)) {
		if (targetWaypoint.GetZ() > dronePosition.GetZ()) return LEFT;
		if (targetWaypoint.GetZ() < dronePosition.GetZ()) return RIGHT;
	}
	else
		return REJECTED;
}

int main() {
	Vector3 dronePosition;
	dronePosition.SetVector3(0.0, 0.0, 0.0);
	Vector3 droneForward;
	droneForward.SetVector3(10.0, 0.0, 0.0);
	Vector3 targetWaypoint;
	targetWaypoint.SetVector3(5, 0, 0.5);
	Vector3 worldUp;
	worldUp.SetVector3(0.0f, 1.0f, 0.0f);
	cout << " Res= " << testWayPoint(dronePosition, droneForward, targetWaypoint, worldUp);
}