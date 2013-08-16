#include "StdAfx.h"
#include "PointLoad2D.h"

///<summary></summary>
///<param name="TransverseMagnitude"></param>
///<param name="TransverseLengthLeft"></param>
///<param name="ParallelMagnitude"></param>
///<param name="ParallelLengthLeft"></param>
///<remarks>description</remarks>
void PointLoad2D::Initialize(double TransverseMagnitude, 
							 double TransverseLengthLeft, 
							 double ParallelMagnitude, 
							 double ParallelLengthLeft)
{
	// The Vertical Load.
	MyTransverseMagnitude = TransverseMagnitude;
	MyTransverseLengthLeft = TransverseLengthLeft;
	
	// The Transverse Load.
	MyParallelMagnitude = ParallelMagnitude;
	MyParallelLengthLeft = ParallelLengthLeft;
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void PointLoad2D::Clone(const PointLoad2D &Load)
{
	// Save all attribute.
	MyBeam = Load.MyBeam;
	
	MyTransverseMagnitude = Load.MyTransverseMagnitude;
	MyTransverseLengthLeft = Load.MyTransverseLengthLeft;

	MyParallelMagnitude = Load.MyParallelMagnitude;
	MyParallelLengthLeft = Load.MyParallelLengthLeft;
};

/// <summary></summary>
PointLoad2D::PointLoad2D(void)
{
	this -> Initialize();
}

/// <summary>Constuctor: Set Transverse Magnitude with MidPoint Distance.</summary>
PointLoad2D::PointLoad2D(Beam2D &Beam,
						 double TransverseMagnitude,
						 bool AddPointerToBeam)
{
	// Set Beam.
	MyBeam = &Beam;

	// Initialize to the Mid Point Distance.
	this -> Initialize(TransverseMagnitude, (MyBeam -> GetLength() / 2.0), 0.0, 0.0);

	// Set the Pointer of Load to Beam.
	this -> SetLoadPointerToBeam(AddPointerToBeam);
};

/// <summary>Constructor: Vertical Point Load.</summary>
PointLoad2D::PointLoad2D(Beam2D &Beam, 
						 double TransverseMagnitude, 
						 double TransverseLengthLeft,
						 bool AddPointerToBeam)
{
	// Beam.
	MyBeam = &Beam;
	
	// Initialize Value.
	this -> Initialize(TransverseMagnitude, TransverseLengthLeft, 0.0, 0.0);

	// Set the Pointer of Load to Beam.
	this -> SetLoadPointerToBeam(AddPointerToBeam);
};

/// <summary>Constructor: Vertical and Horizontal Point Load.</summary>
PointLoad2D::PointLoad2D(Beam2D &Beam, 
						 double TransverseMagnitude, 
						 double TransverseLengthLeft, 
						 double ParallelMagnitude, 
						 double ParallelLengthLeft, 
						 bool AddPointerToBeam)
{
	// Beam.
	MyBeam = &Beam;
	
	// Initialize Value.
	this ->Initialize(TransverseMagnitude, 
		              TransverseLengthLeft, 
					  ParallelMagnitude, 
					  ParallelLengthLeft);

	// Set the Pointer of Load to Beam.
	this -> SetLoadPointerToBeam(AddPointerToBeam);
};

///<summary></summary>
///<param name="Beam"></param>
///<remarks>description</remarks>
PointLoad2D::PointLoad2D(Beam2D &Beam, 
						 double Magnitude, 
						 double Length,
						 char Axis, 
						 bool AddPointerToBeam)
{
	// Set the Base Class.
	MyBeam = &Beam;

	// Get the Angle Between the Beam and the Load.
	double Angle = this -> GetProjectedAngleofForceToBeam(Axis);

	// Initialize Variables.
	this -> Initialize(Magnitude * sin(Angle),
					   Length,
					   Magnitude * cos(Angle),
					   Length);
};

/// <summary>Return an address to the copy of this instance.</summary>
///<param name="Load"></param>
///<returns>description</returns>
///<remarks>description</remarks>
PointLoad2D& PointLoad2D::operator =(const PointLoad2D &Load)
{
	// Set the Value.
	this -> Clone(Load);
	
	// Return a reference to this instance.
	return *this;
};

///<summary>Assignment Operator: Modify the Magnitude.</summary>
///<param name="Factor"></param>
///<remarks>description</remarks>
PointLoad2D& PointLoad2D::operator *(double Factor)
{
	// Modify the Magnitude.
	MyTransverseMagnitude = Factor * MyTransverseMagnitude;
	MyParallelMagnitude = Factor * MyParallelMagnitude;

	// Return the reference address of this object.
	return *this;
};

/// <summary></summary>
PointLoad2D::~PointLoad2D(void)
{
}

/// <summary>Return the Local Load Vector.</summary>
mat PointLoad2D::GetLocalLoadVector()
{
	mat Vector(6, 1);		// Load Vector.
	
	// Length of Beam.
	double L = MyBeam -> GetLength();
	
	double pV = MyTransverseMagnitude;		// Vertical Point Load.
	double L1V = MyTransverseLengthLeft;	// Length to the Left of the Vertical Point Load.
	double L2V = L - L1V;					// Length to the Right of the Vertical Point Load.

	double pH = MyParallelMagnitude;		// Horizontal Point Load
	double L1H = MyParallelLengthLeft;		// Length to the Left of the Horizontal Point Load
	double L2H = L - L1H;					// Length to the Right of the Horizontal Point Load

	// Initialize the Load Vector.
	Vector.fill(0.0);		
	
	Vector(0, 0) = pH * L2H / L;												//	A :: Parallel Force
	Vector(1, 0) = pV * pow(L2V, 2) * (3 * L1V + L2V) / pow(L, 3);				//	A :: Transverse Force
	Vector(2, 0) = pV * L1V * pow(L2V, 2) / pow(L, 2);							//	A :: Moment
	Vector(3, 0) = pH * L1H / L;												//	B :: Parallel Force
	Vector(4, 0) = pV * pow(L1V, 2) * (L1V + 3 * L2V) / pow(L, 3);				//	B :: Transverse Force
	Vector(5, 0) = -pV * pow(L1V, 2) * L2V / pow(L, 2);							//	B :: Moment

	return Vector;
};
