#include "StdAfx.h"
#include "MomentLoad2D.h"

/// <summary></summary>
void MomentLoad2D::Initialize(double MomentMagnitude = 0.0,
							  double MomentLengthLeft = 0.0)
{
	// Concentrated Moment.
	MyMomentMagnitude = MomentMagnitude;
	MyMomentLengthLeft = MomentLengthLeft;

};

/// <summary></summary>
void MomentLoad2D::Clone(const MomentLoad2D &Load)
{
	// Save all the attribute.
	MyBeam = Load.MyBeam;

	// Store the Length.
	this -> Initialize(Load.MyMomentMagnitude, Load.MyMomentLengthLeft);
};

/// <summary></summary>
MomentLoad2D::MomentLoad2D(void)
{
	this -> Initialize();
}

/// <summary></summary>
MomentLoad2D::MomentLoad2D(Beam2D &Beam, 
						   double MomentMagnitude, 
						   double MomentLengthLeft, 
						   bool AddPointerToBeam)
{
	// Set the Beam.
	MyBeam = &Beam;

	// Initialize to the Mid-Point Distance.
	this -> Initialize(MomentMagnitude, MomentLengthLeft);

	// Set the Pointer of Load to Beam.
	this -> SetLoadPointerToBeam(AddPointerToBeam);
};

/// <summary></summary>
MomentLoad2D::~MomentLoad2D(void)
{
}

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
MomentLoad2D& MomentLoad2D::operator =(const MomentLoad2D &Load)
{
	// Set the Value.
	this -> Clone(Load);

	// Return the reference.
	return *this;
};

///<summary>Assignment Operator: Multiple the magnitude with the corresponding factor.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
MomentLoad2D& MomentLoad2D::operator *(double Factor)
{
	// Modify the Moment Magnitude.
	MyMomentMagnitude = Factor * MyMomentMagnitude;

	// Return a Reference Address of this Object.
	return *this;
};

/// <summary>Return the Local Load Vector.</summary>
mat MomentLoad2D::GetLocalLoadVector()
{
	// Load Vector.
	mat Vector(6, 1);

	// Length of the Beam.
	double L = MyBeam -> GetLength();
	// Moment Magnitude.
	double M = MyMomentMagnitude;
	// Length to the Left of the Concentrated Moment.
	double l1 = MyMomentLengthLeft;
	// Length to the Right of the Concentrated Moment.
	double l2 = L - l1;
	
	// Initialize the Load Vector.
	Vector.fill(0.0);

	Vector(0, 0) = 0.0;											//	A :: Parallel Force.
	Vector(1, 0) = -6 * M * (l1 * l2) / pow(L, 3);				//	A :: Transverse Force.
	Vector(2, 0) = M * l2 * (l2 - 2*l1) / pow(L, 2);			//	A :: Moment.
	Vector(3, 0) = 0.0;											//	B :: Parallel Force.
	Vector(4, 0) = 6 * M * (l1 *l2) / pow(L, 3);				//	B :: Transverse Force.
	Vector(5, 0) = M * l1 * (l1 - 2*l2) / pow(L, 2);			//	B :: Moment.

	return Vector;
};
