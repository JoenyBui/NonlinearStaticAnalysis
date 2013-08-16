#include "StdAfx.h"
#include "PyramidLoad2D.h"

using namespace std;
using namespace arma;

void PyramidLoad2D::Initialize(double TransversePeakMagnitude, 
							   double TransverseLeftMagnitude,
							   double TransverseRightMagnitude,
							   double TransversePeakLength,
							   double TransverseLeftLength,
							   double TransverseRightLength)
{
	// Store the Transverse Information.
	MyTransversePeakMagnitude = TransversePeakMagnitude;
	MyTransverseLeftMagnitude = TransverseLeftMagnitude;
	MyTransverseRightMagnitude = TransverseRightMagnitude;

	MyTransversePeakLength = TransversePeakLength;
	MyTransverseLeftLength = TransversePeakLength;
	MyTransverseRightLength = TransverseRightLength;
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void PyramidLoad2D::Clone(const PyramidLoad2D &Load)
{
	// Save all attribute.
	MyBeam = Load.MyBeam;
	
	// Store the Transverse Information.
	MyTransversePeakMagnitude = Load.MyTransversePeakMagnitude;
	MyTransverseLeftMagnitude = Load.MyTransverseLeftMagnitude;
	MyTransverseRightMagnitude = Load.MyTransverseRightMagnitude;

	MyTransversePeakLength = Load.MyTransversePeakLength;
	MyTransverseLeftLength = Load.MyTransverseLeftLength;
	MyTransverseRightLength = Load.MyTransverseRightLength;
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
PyramidLoad2D::PyramidLoad2D(Beam2D &Beam, 
							 double PeakMagnitude, 
							 double LeftMagnitude, 
							 double RightMagnitude, 
							 double PeakLength,
							 bool AddPointerToBeam)
{
	// Store the Beam.
	MyBeam = &Beam;

	// Initialize the Object.
	this -> Initialize(PeakMagnitude, LeftMagnitude, RightMagnitude, PeakLength);

	SetTriangularLoads();

	// Set the Pointer of Load to Beam.
	this -> SetLoadPointerToBeam(AddPointerToBeam);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
PyramidLoad2D::PyramidLoad2D(Beam2D &Beam, 
							 double PeakMagnitude, 
							 double LeftMagnitude, 
							 double RightMagnitude, 
							 double PeakLength, 
							 double LeftLength, 
							 double RightLength,
							 bool AddPointerToBeam)
{
	// Store the Beam.
	MyBeam = &Beam;

	// Initialize the Object.
	this -> Initialize(PeakMagnitude, LeftMagnitude, RightMagnitude, PeakLength, LeftLength, RightLength);

	// Set the Pointer of Load to Beam.
	this -> SetLoadPointerToBeam(AddPointerToBeam);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
PyramidLoad2D::PyramidLoad2D(void)
{
	// Store the Transverse Information.
	// Initialize the Object.
	this -> Initialize();
}

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
PyramidLoad2D::~PyramidLoad2D(void)
{
}

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void PyramidLoad2D::SetTriangularLoads()
{
	// Length of the Beam
	double Length = MyBeam -> GetLength();

	MyLeftTriangularLoad = TriangularLoad2D(*MyBeam, 
											MyTransverseLeftMagnitude, 
											MyTransversePeakMagnitude, 
											MyTransverseLeftLength, 
											Length - MyTransversePeakLength, 
											false);

	MyRightTriangularLoad = TriangularLoad2D(*MyBeam,
											 MyTransversePeakMagnitude,
											 MyTransverseRightMagnitude,
											 MyTransversePeakLength,
											 MyTransverseRightLength,
											 false);
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
mat PyramidLoad2D::GetLocalLoadVector()
{	
	// Set the Two Trianglur Load to create a Pyramid Loading.
	this -> SetTriangularLoads();

	// Intitialize the Load Vector.
	mat Vector(6, 1);
	
	// Initialize the Vector.
	Vector.fill(0.0);
	
	mat LeftVector = MyLeftTriangularLoad.GetLocalLoadVector();
	mat RightVector = MyRightTriangularLoad.GetGlobalLoadVector();

	Vector = LeftVector + RightVector;

	return Vector;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
PyramidLoad2D& PyramidLoad2D::operator= (const PyramidLoad2D &Load)
{
	// Set the Value.
	this -> Clone(Load);
	
	// Return a reference to this instance.
	return *this;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
PyramidLoad2D& PyramidLoad2D::operator* (double Factor)
{
	// Modify the Magnitude.
	MyTransversePeakMagnitude = Factor * MyTransversePeakMagnitude;
	MyTransverseLeftMagnitude = Factor * MyTransverseLeftMagnitude;
	MyTransverseRightMagnitude = Factor * MyTransverseRightMagnitude;

	// Return the reference address of this object.
	return *this;
};

