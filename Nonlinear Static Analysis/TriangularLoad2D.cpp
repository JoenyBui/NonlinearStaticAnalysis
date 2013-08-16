#include "StdAfx.h"
#include "TriangularLoad2D.h"

using namespace std;
using namespace arma;

///<summary>Initialize all the private variables of the Load.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void TriangularLoad2D::Initialize(double TransverseLeftMagnitude, 
							      double TransverseRightMagnitude, 
							      double TransverseLeftLength, 
							      double TransverseRightLength, 
							      double ParallelLeftMagnitude, 
							      double ParallelRightMagnitude)
{
	MyTransverseLeftMagnitude = TransverseLeftMagnitude;
	MyTransverseLeftLength = TransverseLeftLength;

	MyTransverseRightMagnitude = TransverseRightMagnitude;
	MyTransverseRightLength = TransverseRightLength;

	MyParallelLeftMagnitude = ParallelLeftMagnitude;
	MyParallelRightMagnitude = ParallelRightMagnitude;
};

///<summary>Save all the variables fromt he reference load.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void TriangularLoad2D::Clone(const TriangularLoad2D &Load)
{
	MyTransverseLeftMagnitude = Load.MyTransverseLeftMagnitude;
	MyTransverseLeftLength = Load.MyTransverseLeftLength;
	MyTransverseRightMagnitude = Load.MyTransverseRightMagnitude;
	MyTransverseRightLength = Load.MyTransverseRightLength;

	MyParallelLeftMagnitude = Load.MyParallelLeftMagnitude;
	MyParallelRightMagnitude = Load.MyParallelRightMagnitude;
};

///<summary>Constructor for the Transverse Load that is throughout the beam.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
TriangularLoad2D::TriangularLoad2D(Beam2D &Beam, 
								   double LeftMagnitude, 
								   double RightMagnitude, 
								   bool AddPointerToBeam)
{
	// Set the Base Class.
	MyBeam = &Beam;

	// Initialize Varialbes.
	this ->Initialize(LeftMagnitude, RightMagnitude);

	// Set the Pointer of Load to Beam.
	this -> SetLoadPointerToBeam(AddPointerToBeam);
};
		
///<summary>Constructor for the Transverse Load that stops within the beam.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
TriangularLoad2D::TriangularLoad2D(Beam2D &Beam, 
								   double LeftMagnitude, 
								   double RightMagnitude, 
								   double LeftLength, 
								   double RightLength,
								   bool AddPointerToBeam)
{
	// Set the Base Class.
	MyBeam = &Beam;

	// Initialize Varialbes.
	this -> Initialize(LeftMagnitude, RightMagnitude, LeftLength, RightLength);

	// Set the Pointer of Load to Beam.
	this -> SetLoadPointerToBeam(AddPointerToBeam);
};

///<summary>Constructor for the Transverse Load that stops within the beam and the Parallel Load that extends to the edge.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
TriangularLoad2D::TriangularLoad2D(Beam2D &Beam, 
								   double TransverseLeftMagnitude, 
								   double TransverseRightMagnitude, 
								   double TransverseLeftLength, 
								   double TransverseRightLength, 
								   double ParallelLeftMagnitude, 
								   double ParallelRightMagnitude, 
								   bool AddPointerToBeam)
{
	// Set the Base Class.
	MyBeam = &Beam;

	// Initialize Varialbes.
	this -> Initialize(TransverseLeftMagnitude, 
			           TransverseRightMagnitude, 
					   TransverseLeftLength, 
					   TransverseRightLength, 
					   ParallelLeftMagnitude, 
					   ParallelRightMagnitude);

	// Set the Pointer of Load to Beam.
	this -> SetLoadPointerToBeam(AddPointerToBeam);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
TriangularLoad2D::TriangularLoad2D(Beam2D &Beam,
								   double LinearLeftMagnitude,
								   double LinearRightMagnitude,
								   char Axis,
								   bool AddPointerToBeam)
{
	// Set the Base Class.
	MyBeam = &Beam;

	double Angle = this -> GetProjectedAngleofForceToBeam(Axis);

	// Initialize Varialbes.
	this -> Initialize(LinearLeftMagnitude * sin(Angle), 
			           LinearRightMagnitude * sin(Angle), 
					   0.0, 
					   0.0, 
					   LinearLeftMagnitude * cos(Angle), 
					   LinearRightMagnitude * cos(Angle));

	// Set the Pointer of Load to Beam.
	this -> SetLoadPointerToBeam(AddPointerToBeam);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
TriangularLoad2D::TriangularLoad2D(void)
{
	// Initialize Variables.
	this ->Initialize();
};

/// <summary></summary>
TriangularLoad2D::~TriangularLoad2D(void)
{
};

/// <summary>Return an address to the copy of this instance.</summary>
///<param name="Load"></param>
///<returns>description</returns>
///<remarks>description</remarks>
TriangularLoad2D& TriangularLoad2D::operator =(const TriangularLoad2D &Load)
{
	// Set the Value.
	this -> Clone(Load);
	
	// Return a reference to this instance.
	return *this;
};

///<summary>Assignment Operator: Modify the Magnitude.</summary>
///<param name="Factor"></param>
///<remarks>description</remarks>
TriangularLoad2D& TriangularLoad2D::operator *(double Factor)
{
	// Modify the Magnitude.
	MyTransverseLeftMagnitude = Factor * MyTransverseLeftMagnitude;
	MyTransverseRightMagnitude = Factor * MyTransverseRightMagnitude;
	MyParallelLeftMagnitude = Factor * MyParallelLeftMagnitude;
	MyParallelRightMagnitude = Factor * MyParallelRightMagnitude;

	// Return the reference address of this object.
	return *this;
};

/// <summary></summary>
mat TriangularLoad2D::GetLocalLoadVector()
{
	mat Vector(6, 1);		// Load Vector.
	
	double L = MyBeam -> GetLength();

	// Initialize the Load Vector.
	Vector.fill(0.0);		

	//	A :: Parallel Force	
	Vector(0, 0) = GetLowerFixedEndParallelForce(L, MyParallelLeftMagnitude, MyParallelRightMagnitude);		

	//	B :: Parallel Force
	Vector(3, 0) = GetHigherFixedEndParallelForce(L, MyParallelLeftMagnitude, MyParallelRightMagnitude);		

	if (abs(MyTransverseLeftMagnitude) <= abs(MyTransverseRightMagnitude))
	{
										
		Vector(1, 0) = GetLowerFixedEndTransverseForce(L, 
													   MyTransverseLeftMagnitude,
													   MyTransverseLeftLength,
													   MyTransverseRightMagnitude,
													   MyTransverseRightLength);			//	A :: Transverse Force
		Vector(2, 0) = GetLowerFixedEndMoment(L, 
											  MyTransverseLeftMagnitude,
											  MyTransverseLeftLength,
											  MyTransverseRightMagnitude,
											  MyTransverseRightLength);						//	A :: Moment

											
		Vector(4, 0) = GetHigherFixedEndTransverseForce(L, 
													   MyTransverseLeftMagnitude,
													   MyTransverseLeftLength,
													   MyTransverseRightMagnitude,
													   MyTransverseRightLength);			//	B :: Transverse Force
		Vector(5, 0) = GetHigherFixedEndMoment(L, 
											   MyTransverseLeftMagnitude,
											   MyTransverseLeftLength,
											   MyTransverseRightMagnitude,
											   MyTransverseRightLength);					//	B :: Moment
	} 
	else
	{

		Vector(1, 0) = GetHigherFixedEndTransverseForce(L, 
													    MyTransverseRightMagnitude,
													    MyTransverseRightLength,
													    MyTransverseLeftMagnitude,
													    MyTransverseLeftLength);			//	A :: Transverse Force
		Vector(2, 0) = -GetHigherFixedEndMoment(L, 
											   MyTransverseRightMagnitude,
											   MyTransverseRightLength,
											   MyTransverseLeftMagnitude,
											   MyTransverseLeftLength);						//	A :: Moment

		Vector(4, 0) = GetLowerFixedEndTransverseForce(L, 
													   MyTransverseRightMagnitude,
													   MyTransverseRightLength,
													   MyTransverseLeftMagnitude,
													   MyTransverseLeftLength);				//	B :: Transverse Force
		Vector(5, 0) = -GetLowerFixedEndMoment(L, 
											  MyTransverseRightMagnitude,
											  MyTransverseRightLength,
											  MyTransverseLeftMagnitude,
											  MyTransverseLeftLength);						//	B :: Moment
	}
	return Vector;
};

///<summary></summary>
///<param name="Length"></param>
///<param name = "LowerMagnitude"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double TriangularLoad2D::GetLowerFixedEndParallelForce(double Length,
														double LowerMagnitude, 
														double HigherMagnitude)
{
	return (2 * LowerMagnitude + HigherMagnitude) * Length / 6.0;
};

/// <summary></summary>
double TriangularLoad2D::GetLowerFixedEndTransverseForce(double Length, 
									   double LowerMagnitude, 
									   double LowerLength, 
									   double HigherMagnitude, 
									   double HigherLength)
{
	// Equations from "Matrix Analysis of Structures" Aslam Kassimali - Book Cover Page
	// FSb = A{B - C*D + E} + F{G*H - I*J} 
	
	double L = Length;
	double w_1 = LowerMagnitude;
	double l_1 = LowerLength;
	double w_2 = HigherMagnitude;
	double l_2 = HigherLength;

	// Constants
	double A = (w_1 * pow(L - l_1, 3)) / (20 * pow(L, 3));
	double B = 7 * L + 8 * l_1;
	double C = l_2 * (3*L + 2*l_1) / (L - l_1);
	double D = 1 + l_2/(L - l_1) + pow(l_2, 2) / pow(L - l_1, 2);
	double E = 2 * pow(l_2, 4) / pow(L - l_1, 2);
	
	double F = (w_2 * pow(L - l_1, 3)) / (20 * pow(L, 3));
	double G = 3 * L + 2 * l_1;
	double H = 1 + l_2 / (L - l_1) + pow(l_2, 2)/pow(L - l_1, 2);
	double I = pow(l_2, 3) / pow(L - l_1, 2);
	double J = 2 + (15 * L - 8 * l_2) / (L - l_1);
	
	// Shear Force;
	double FSb = A*(B - C*D + E) + F*(G*H - I*J);

	return FSb;
};

/// <summary></summary>
double TriangularLoad2D::GetLowerFixedEndMoment(double Length, 
							  double LowerMagnitude, 
							  double LowerLength, 
							  double HigherMagnitude, 
							  double HigherLength)
{
	// Equations from "Matrix Analysis of Structures" Aslam Kassimali - Book Cover Page
	// FMb = A[B - C*D + E] + F[G*H - I*J]

	double L = Length;
	double w_1 = LowerMagnitude;
	double l_1 = LowerLength;
	double w_2 = HigherMagnitude;
	double l_2 = HigherLength;
	
	double A = (w_1 * pow(L - l_1, 3)) / (60 * pow(L, 2));
	double B = 3*(L + 4*l_1);
	double C = l_2 * (2*L + 3*l_1) / (L - l_1);
	double D = 1 + l_2*(L - l_1) + pow(l_2, 2)/pow(L - l_1, 2);
	double E = (3 * pow(l_2, 4)) / pow(L - l_1, 3); 

	double F = (w_2 * pow(L - l_1, 3)) / (60 * pow(L, 2));
	double G = 2*L + 3*l_1;
	double H = 1 + l_2 / (L - l_1) + pow(l_2, 2)/pow(L - l_1, 2);
	double I = 3 * pow(l_2, 3) / pow(L - l_1, 2);
	double J = 1 + (5*L - 4*l_2)/ (L - l_1);

	double FMb = A*(B - C*D + E) + F*(G*H - I*J);

	return FMb;
};

///<summary>Return the Hight Fixed End Parallel Force.</summary>
///<param name="Length"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double TriangularLoad2D::GetHigherFixedEndParallelForce(double Length,
														double LowerMagnitude, 
														double HigherMagnitude)
{
	return (LowerMagnitude + 2 * HigherMagnitude) * Length / 6.0;
};

/// <summary></summary>
double TriangularLoad2D::GetHigherFixedEndTransverseForce(double Length, 
														  double LowerMagnitude, 
														  double LowerLength, 
														  double HigherMagnitude, 
														  double HigherLength)
{
	// Equations from "Matrix Analysis of Structures" Aslam Kassimali - Book Cover Page
	// FSe = (w_1 + w_2)/2 * (L - l_1 - l_2) - FSb;
	double L = Length;
	double w_1 = LowerMagnitude;
	double l_1 = LowerLength;
	double w_2 = HigherMagnitude;
	double l_2 = HigherLength;
	
	double FSb = GetLowerFixedEndTransverseForce(L, w_1, l_1, w_2, l_2);

	double FSe = ((w_1 + w_2) / 2) * (L - l_1 - l_2) - FSb;

	return FSe;
};

/// <summary></summary>
double TriangularLoad2D::GetHigherFixedEndMoment(double Length, 
												 double LowerMagnitude, 
												 double LowerLength, 
												 double HigherMagnitude, 
												 double HigherLength)
{
	// Equations from "Matrix Analysis of Structures" Aslam Kassimali - Book Cover Page
	// FMe = A*(B - C) * FSe*L - FMb

	double L = Length;
	double w_1 = LowerMagnitude;
	double l_1 = LowerLength;
	double w_2 = HigherMagnitude;
	double l_2 = HigherLength;
	
	double FSe = GetLowerFixedEndTransverseForce(L, w_1, l_1, w_2, l_2);
	double FMb = GetLowerFixedEndMoment(L, w_1, l_1, w_2, l_2);

	double A = (L - l_1 - l_2) / 6;
	double B = w_1 * (-2*L + 2*l_1 - l_2);
	double C = w_2 * (L - l_1 - 2*l_2);

	double FMe =  A * (B - C) + FSe*L - FMb;

	return FMe;
};