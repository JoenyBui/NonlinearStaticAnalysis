#include "StdAfx.h"
#include "UniformLoad2D.h"

using namespace std;
using namespace arma;

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void UniformLoad2D::Initialize(double TransverseMagnitude, 
							   double TransverseLeftLength, 
							   double TransverseRightLength,
							   double ParallelMagnitude,
							   double ParallelLeftLength,
							   double ParallelRightLength)
{
	// Transverse Loading.
	MyTransverseMagnitude = TransverseMagnitude;
	MyTransverseLeftLength = TransverseLeftLength;
	MyTransverseRightLength = TransverseRightLength;
	
	// Parallel Loading.
	MyParallelMagnitude = ParallelMagnitude;
	MyParallelLeftLength = ParallelLeftLength;
	MyParallelRightLength = ParallelRightLength;
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void UniformLoad2D::Clone(const UniformLoad2D &Load)
{
	// Save all attribute.
	MyBeam = Load.MyBeam;

	// Transverse Load
	MyTransverseMagnitude = Load.MyTransverseMagnitude;
	MyTransverseLeftLength = Load.MyTransverseLeftLength;
	MyTransverseRightLength = Load.MyTransverseRightLength;

	MyParallelMagnitude = Load.MyParallelMagnitude;
	MyParallelLeftLength = Load.MyParallelLeftLength;
	MyParallelRightLength = Load.MyParallelRightLength;
};

///<summary></summary>
///<param name="&Beam"></param>
///<param name="TransverseMagnitude"></param>
///<param name="AddPointerToBeam"></param>
///<remarks>description</remarks>
UniformLoad2D::UniformLoad2D(Beam2D &Beam, 
							 double TransverseMagnitude,
							 bool AddPointerToBeam)
{
	// Set the Base Class.
	MyBeam = &Beam;
	
	// Initialize Variable.
	this -> Initialize(TransverseMagnitude, 0.0, 0.0, 0.0, 0.0, 0.0);

	// Set the Pointer of Load to Beam.
	this -> SetLoadPointerToBeam(AddPointerToBeam);
};

/// <summary>Constructor: </summary>
UniformLoad2D::UniformLoad2D(Beam2D& Beam, 
							 double TransverseMagnitude, 
							 double TransverseLeftLength, 
							 double TransverseRightLength, 
							 double ParallelMagnitude, 
							 double ParallelLeftLength,
							 double ParallelRightLength, 
							 bool AddPointerToBeam)
{
	// Set the Base Class.
	MyBeam = &Beam;
	
	// Initialize Variable.
	this -> Initialize(TransverseMagnitude, 
					   TransverseLeftLength, 
					   TransverseRightLength, 
					   ParallelMagnitude, 
					   ParallelLeftLength, 
					   ParallelRightLength);

	// Set the Pointer of Load to Beam.
	this -> SetLoadPointerToBeam(AddPointerToBeam);
};

///<summary></summary>
///<param name="Beam"></param>
///<param name="UniformMagnitude"></param>
///<param name="LeftLength"></param>
///<param name="RightLength"></param>
///<param name="Axis"></param>
///<param name="AddPointerToBeam"></param>
///<remarks>description</remarks>
UniformLoad2D::UniformLoad2D(Beam2D &Beam, 
							 double UniformMagnitude, 
							 double LeftLength,
							 double RightLength,
							 char Axis, 
							 bool AddPointerToBeam)
{
	// Set the Base Class.
	MyBeam = &Beam;

	// Get the Angle Between the Beam and the Load.
	double Angle = this -> GetProjectedAngleofForceToBeam(Axis);

	// Initialize Varialbes.
	this -> Initialize(UniformMagnitude * sin(Angle), 
			           LeftLength, 
					   RightLength, 
					   UniformMagnitude * cos(Angle), 
					   LeftLength, 
					   RightLength);
};

/// <summary>Default Constructor: No Magnitude.</summary>
UniformLoad2D::UniformLoad2D(void)
{
	// Initialize Variable.
	this -> Initialize();
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
UniformLoad2D::~UniformLoad2D(void)
{
};

///<summary></summary>
///<param name="Load"></param>
///<returns>description</returns>
///<remarks>description</remarks>
UniformLoad2D& UniformLoad2D::operator =(const UniformLoad2D &Load)
{
	// Clone the Variable.
	this -> Clone(Load);

	// Return a reference to this instance.
	return *this;
};

///<summary></summary>
///<param name="Factor"></param>
///<returns>description</returns>
///<remarks>description</remarks>
UniformLoad2D& UniformLoad2D::operator *(double Factor)
{
	// 
	MyTransverseMagnitude = Factor * MyTransverseMagnitude;
	MyParallelMagnitude = Factor * MyParallelMagnitude;

	// Return the Reference to this Instance.
	return *this;
};

/// <summary>Return the Local Load Vector for a Uniform Load Distribution.</summary>
mat UniformLoad2D::GetLocalLoadVector()
{
	mat Vector(6, 1);		// Load Vector.
	
	double L = MyBeam -> GetLength();

	// Initialize the Load Vector.
	Vector.fill(0.0);		
	
	Vector(0, 0) = this -> GetLeftFixedEndParallelForce(L);				//	A :: Parallel Force
	Vector(1, 0) = this -> GetLeftFixedEndTransverseForce(L);			//	A :: Transverse Force
	Vector(2, 0) = this -> GetLeftFixedEndMoment(L);					//	A :: Moment
	Vector(3, 0) = this -> GetRightFixedEndParallelForce(L);			//	B :: Parallel Force
	Vector(4, 0) = this -> GetRightFixedEndTransverseForce(L);			//	B :: Transverse Force
	Vector(5, 0) = this -> GetRightFixedEndMoment(L);					//	B :: Moment

	return Vector;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double UniformLoad2D::GetLeftFixedEndParallelForce(double Length)
{
	// Equations from "Matrix Analysis of Structures" Aslam Kassimali - Book Cover Page
	// FA_b = (w / 2L) (L - l_1 - l_2) (L - l_1 + l_2)
	
	double w = MyParallelMagnitude;
	double L = Length;
	double l_1 = MyParallelLeftLength;
	double l_2 = MyParallelRightLength;

	// Solve:
	double FA_b = (w / 2*L) * (L - l_1 - l_2) * (L - l_1 + l_2);

	return FA_b;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double UniformLoad2D::GetLeftFixedEndTransverseForce(double Length)
{
	// Equations from "Matrix Analysis of Structures" Aslam Kassimali - Book Cover Page
	// FS_b = (wL / 2) [ 1 - (l_1 / L^4) (2L^3 - 2l_1^2 * L + l_1^3) - l_2^3 / L^4 * (2*L - l_2)]
	
	double w = MyTransverseMagnitude;
	double L = Length;
	double l_1 = MyTransverseLeftLength;
	double l_2 = MyTransverseRightLength;
	
	double FS_b = (w*L/2) 
					*(1 - (l_1/pow(L, 4)) * (2*pow(L, 3) - 2*pow(l_1, 2)*L + pow(l_1, 3)) 
					        - pow(l_2, 3) / pow(L, 4) * (2*L - l_2));

	return FS_b;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double UniformLoad2D::GetLeftFixedEndMoment(double Length)
{
	// Equations from "Matrix Analysis of Structures" Aslam Kassimali - Book Cover Page
	// FM_b = (w*L^2/12)[1-l_1^2/L^4*(6*L^2 - 8*l_1*L + 3*l_1^2) - l_2^3 / L^4 * (4*L - 3*l_2)]
	
	double w = MyTransverseMagnitude;
	double L = Length;
	double l_1 = MyTransverseLeftLength;
	double l_2 = MyTransverseRightLength;

	double FM_b = (w*pow(L, 2) / 12) 
					* (1 - pow(l_1, 2)/pow(L, 4) * (6*pow(L, 2) - 8*l_1*L + 3*pow(l_1, 2)) 
						 - pow(l_2, 3)/pow(L, 4) * (4*L - 3*l_2));

	return FM_b;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double UniformLoad2D::GetRightFixedEndParallelForce(double Length)
{
	// Equations from "Matrix Analysis of Structures" Aslam Kassimali - Book Cover Page
	// FA_e = (w / 2L) (L - l_1 - l_2) (L + l_1 - l_2)
	
	double w = MyParallelMagnitude;
	double L = Length;
	double l_1 = MyParallelLeftLength;
	double l_2 = MyParallelRightLength;

	double FA_e = (w / 2 * L) * (L - l_1 - l_2) * (L + l_1 - l_2);

	return FA_e;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double UniformLoad2D::GetRightFixedEndTransverseForce(double Length)
{
	// Equations from "Matrix Analysis of Structures" Aslam Kassimali - Book Cover Page
	// FS_e = (wL / 2) [ 1 - (l_1^3 / L^4) (2L - l_1) - l_2 / L^4 * (2*L^3 - 2*l_2^2*L + l_2^3)]
	
	double w = MyTransverseMagnitude;
	double L = Length;
	double l_1 = MyTransverseLeftLength;
	double l_2 = MyTransverseRightLength;
	
	double FS_e = (w * L / 2) 
					* (1 - (pow(l_1, 3) / pow(L, 4)) * (2 * L - l_1) 
						- l_2 / pow(L, 4) * (2 * pow(L, 3) - 2 * pow(l_2, 2) * L + pow(l_2, 3)));
	
	return FS_e;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double UniformLoad2D::GetRightFixedEndMoment(double Length)
{
	// Equations from "Matrix Analysis of Structures" Aslam Kassimali - Book Cover Page
	// FM_e = (-w*L^2/12) [1 - l_1^3/L^4(4L - 3l_1) - l_2^2 / L^4 * (6*L^2 - 8*l_2*L + 3*l_2^2)
	
	double w = MyTransverseMagnitude;
	double L = Length;
	double l_1 = MyTransverseLeftLength;
	double l_2 = MyTransverseRightLength;

	double FM_e = (-w * pow(L, 2) / 12)
					* (1 - pow(l_1, 3) / pow(L, 4) *(4 * L - 3 * l_1) 
					   - pow(l_2, 2) / pow(L, 4) * (6 * pow(L, 2) - 8 * l_2 * L + 3 * pow(l_2, 2)));

	return FM_e;
};