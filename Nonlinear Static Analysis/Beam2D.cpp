//#include "StdAfx.h"
//#include "Node2D.h"
#include "Beam2D.h"

using namespace std;
using namespace arma;

// Constructor
void Beam2D::Initialize(int ID = -1)
{
	// Default ID.
	MyID = ID;
	
	// No Releases.
	SetNodeARelease(false, false, false);
	SetNodeBRelease(false, false, false);
};

void Beam2D::Clone(const Beam2D &Beam)
{
	// Copy Node ID.
	MyID = Beam.MyID;

	// Copy Node A and Node B.
	MyNodeA = Beam.MyNodeA;
	MyNodeB = Beam.MyNodeB;
	
	// Copy the material property.
	MySection = Beam.MySection;

	// Copy the Releases.
	MyReleaseNodeA[0] = Beam.MyReleaseNodeA[0];
	MyReleaseNodeA[1] = Beam.MyReleaseNodeA[1];
	MyReleaseNodeA[2] = Beam.MyReleaseNodeA[2];

	MyReleaseNodeB[0] = Beam.MyReleaseNodeB[0];
	MyReleaseNodeB[1] = Beam.MyReleaseNodeB[1];
	MyReleaseNodeB[2] = Beam.MyReleaseNodeB[2];

	MyReleaseNodeAMagnitude[0] = Beam.MyReleaseNodeAMagnitude[0];
	MyReleaseNodeAMagnitude[1] = Beam.MyReleaseNodeAMagnitude[1];
	MyReleaseNodeAMagnitude[2] = Beam.MyReleaseNodeAMagnitude[2];

	MyReleaseNodeBMagnitude[0] = Beam.MyReleaseNodeBMagnitude[0];
	MyReleaseNodeBMagnitude[1] = Beam.MyReleaseNodeBMagnitude[1];
	MyReleaseNodeBMagnitude[2] = Beam.MyReleaseNodeBMagnitude[2];
};

// Default Constructor
Beam2D::Beam2D() //: MyNodeA(Node2D()), MyNodeB(Node2D())
{
	// Call the Intitialize Method.
	this -> Initialize();
};
		
// Copy Constructor.
Beam2D::Beam2D(const Beam2D &Beam)
{
	// Call the Clone Method.
	this -> Clone(Beam);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
Beam2D::Beam2D(int ID, 
			   Node2D &NodeA, 
			   Node2D &NodeB, 
			   Section2D &Section)
{
	// See default parameter.
	this -> Initialize(ID);

	// Set the Nodes.
	MyNodeA = &NodeA;
	MyNodeB = &NodeB;

	// Set the Section Reference.
	MySection = &Section;

	// Settle the Pointer to the Node.
	this -> SetBeamPointerToNode();
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
Beam2D::Beam2D(int ID, 
			   Node2D &NodeA, 
			   Node2D &NodeB, 
			   Section2D &Section, 
			   bool ReleaseNodeA[], 
			   bool ReleaseNodeB[])
{
	// See default parameter.
	this -> Initialize(ID);

	// Set the Nodes.
	MyNodeA = &NodeA;
	MyNodeB = &NodeB;

	// Set the Section Reference.
	MySection = &Section;

	// Set the Releases.
	this -> SetNodeARelease(ReleaseNodeA[0], ReleaseNodeA[1], ReleaseNodeA[2]);
	this -> SetNodeBRelease(ReleaseNodeB[0], ReleaseNodeB[1], ReleaseNodeB[2]);

	this -> SetBeamPointerToNode();
};

// Destructor
Beam2D::~Beam2D() {
};

/// <summary> Cloning Operator '=' </summary>
Beam2D& Beam2D::operator =(const Beam2D &Beam)
{
	// Copy Data.
	this -> Clone(Beam);

	return *this;
};

///<summary>Set the Nodes Address inside the Beam.</summary>
///<param name="NodeAPtr">Pass Node A Pointer.</param>
///<param name="NodeBPtr">Pass Node B Pointer.</param>
///<remarks>description</remarks>
void Beam2D::SetNode(Node2D* NodeAPtr, Node2D* NodeBPtr)
{
	// Set the Nodes.
	this -> SetNodeA(NodeAPtr);
	this -> SetNodeB(NodeBPtr);
};


///<summary>Set the Node A address inside the Beam.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void Beam2D::SetNodeA(Node2D *NodePtr)
{
	// Change Node A.
	MyNodeA = NodePtr;
};

///<summary>Set the Node B address inside the Beam.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void Beam2D::SetNodeB(Node2D *NodePtr)
{
	// Change Node B.
	MyNodeB = NodePtr;
};

///<summary>Set the Pointer of the Section to the Beam.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
void Beam2D::SetSection(Section2D* SectionPtr)
{
	// Set the Materail.
	MySection = SectionPtr;
};


///<summary>Set the Beam Pointers for Node A and Node B.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void Beam2D::SetBeamPointerToNode()
{
	// Assign the Beams to the Nodes.
	MyNodeA -> AddBeam(this);
	MyNodeB -> AddBeam(this);
};

/// <summary> Return Nodal ID. </summary>
int Beam2D::GetID() 
{ 
	return MyID; 
}


/// <summary></summary>
Node2D& Beam2D::GetNode(int ID)
{
	Node2D *Node;

	if (ID == GetNodeAID())
	{
		Node = MyNodeA;
	} 
	else
	{
		Node = MyNodeB;
	}

	return *Node;
};

/// <summary>Check the Release of all the DOFs.</summary>
bool Beam2D::IsAnyDofRelease()
{
	bool IsRelease = false;

	// Loop through the DOFs see if anything is release.
	for (int i = 0; i < DOF; i++)
	{
		// Check to see if Node A or Node B is release.
		if (MyReleaseNodeA[i] == true || MyReleaseNodeB[i] == true)
		{
			// Set to true.
			IsRelease = true;
		}
	};

	// Return if the Beam is release.
	return IsRelease;
};

/// <summary>Is Translation about the X Release.</summary>
bool Beam2D::IsTranslationXRelease(int ID)
{
	bool Release;
	
	// Check the which Node to return.
	if (ID == GetNodeAID())
	{
		// Return the Translation X.
		Release = MyReleaseNodeA[0];
	}
	else
	{
		// Return the Translation X.
		Release = MyReleaseNodeB[0];
	}

	return Release;
};

bool Beam2D::IsTranslationXReleaseNodeA()
{
	return MyReleaseNodeA[0];
};

bool Beam2D::IsTranslationXReleaseNodeB()
{
	return MyReleaseNodeB[0];
};


/// <summary>Is Translationa about the Y axis Release.</summary>
bool Beam2D::IsTranslationYRelease(int ID)
{
	bool Release;
	
	// Check the which Node to return.
	if (ID == GetNodeAID())
	{
		// Return the Translation X.
		Release = MyReleaseNodeA[1];
	}
	else
	{
		// Return the Translation X.
		Release = MyReleaseNodeB[1];
	}

	return Release;
};

bool Beam2D::IsTranslationYReleaseNodeA()
{
	return MyReleaseNodeA[1];
};

bool Beam2D::IsTranslationYReleaseNodeB()
{
	return MyReleaseNodeB[1];
};

/// <summary>Is Rotation about Z Release.</summary>
bool Beam2D::IsRotationZRelease(int ID)
{
	bool Release;
	
	// Check the which Node to return.
	if (ID == this -> GetNodeAID())
	{
		// Return the Translation X.
		Release = MyReleaseNodeA[2];
	}
	else
	{
		// Return the Translation X.
		Release = MyReleaseNodeB[2];
	}

	return Release;
};

bool Beam2D::IsRotationZReleaseNodeA()
{
	return MyReleaseNodeA[2];
};

bool Beam2D::IsRotationZReleaseNodeB()
{
	return MyReleaseNodeB[2];
};

/// <summary> Return Node A. </summary>
Node2D& Beam2D::GetNodeA() 
{ 
	return *MyNodeA; 
}

/// <summary> Return Node B. </summary>
Node2D& Beam2D::GetNodeB() 
{ 
	return *MyNodeB; 
}
		
/// <summary> Return Node A ID. </summary>
int Beam2D::GetNodeAID() 
{ 
	return MyNodeA -> GetID(); 
}

/// <summary> Return Node B ID. </summary>
int Beam2D::GetNodeBID() 
{ 
	return MyNodeB -> GetID(); 
}

///<summary>Return the Section ID for this Beam.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
int Beam2D::GetSectionID()
{
	return MySection -> GetID();
};

///<summary>Return Degrees of Freedom Index.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
vector <int> Beam2D::GetDOFIndexNodes()
{
	// Index Vector.
	vector <int> Index;
	
	// Get the Index vector for each Node.
	vector <int> IndexNodeA = MyNodeA -> GetBeamDOFIndexes(this -> MyID);
	vector <int> IndexNodeB = MyNodeB -> GetBeamDOFIndexes(this -> MyID);
	
	// Store them inside the Index.
	// Store the A.
	for (int i = 0; i < IndexNodeA.size(); i++)
	{
		Index.push_back(IndexNodeA[i]);
	};

	// Store the B.
	for (int j = 0; j < IndexNodeB.size(); j++)
	{
		Index.push_back(IndexNodeB[j]);
	};

	// Return the Index 6 DOF.
	return Index;
}

/// <summary></summary>
///<param name="ParallelRelease"></param>
///<param name="TransverseRelease"></param>
///<param name="MomentRelease"></param>
///<param name="ParallelMagnitude"></param>
///<param name="TransverseMagnitude"></param>
///<param name="MomentMagnitude"></param>
///<returns>description</returns>
///<remarks>description</remarks>
void Beam2D::SetNodeARelease(bool ParallelRelease, 
							 bool TransverseRelease, 
							 bool MomentRelease, 
							 double ParallelMagnitude, 
							 double TransverseMagnitude, 
							 double MomentMagnitude)
{
	// Set the Release for Degrees-of-Freedom.  First look at then and 
	// see if the node support. 

	MyReleaseNodeA[0] = ParallelRelease;
	MyReleaseNodeA[1] = TransverseRelease;
	MyReleaseNodeA[2] = MomentRelease;
	
	MyReleaseNodeAMagnitude[0] = ParallelMagnitude;
	MyReleaseNodeAMagnitude[1] = TransverseMagnitude;
	MyReleaseNodeAMagnitude[2] = MomentMagnitude;
};


/// <summary></summary>
void Beam2D::SetNodeBRelease(bool ParallelRelease, 
							 bool TransverseRelease, 
							 bool MomentRelease, 
							 double ParallelMagnitude, 
							 double TransverseMagnitude, 
							 double MomentMagnitude)
{
	MyReleaseNodeB[0] = ParallelRelease;
	MyReleaseNodeB[1] = TransverseRelease;
	MyReleaseNodeB[2] = MomentRelease;
	
	MyReleaseNodeBMagnitude[0] = ParallelMagnitude;
	MyReleaseNodeBMagnitude[1] = TransverseMagnitude;
	MyReleaseNodeBMagnitude[2] = MomentMagnitude;
};

///<summary>Release the Moment at A.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void Beam2D::SetNodeAReleaseMoment(bool MomentRelease, 
								   double MomentMagnitude)
{
	MyReleaseNodeA[2] = MomentRelease;
	MyReleaseNodeAMagnitude[2] = MomentMagnitude;
};

///<summary>Release the Moment at B.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void Beam2D::SetNodeBReleaseMoment(bool MomentRelease, 
								   double MomentMagnitude)
{
	MyReleaseNodeB[2] = MomentRelease;
	MyReleaseNodeBMagnitude[2] = MomentMagnitude;
};


///<summary>Return the Stiffness Matrix.</summary>
///<remarks></remarks>
mat Beam2D::GetLocalStiffnessMatrix() {
	// Stiffness matrix 
	mat K(6,6);
	
	// Initialize the Stiffness Matrix
	K.fill(0.0);
		
	double L = GetLength();																// Length
	double A = MySection -> GetArea();													// Area
	double E = MySection -> GetElasticModulus();										// Elastic Modulus
	double I = MySection -> GetMomentOfInertia();										// Moment of Inertia
	        	        
	// Setup the Stiffness Function
	        
    /*
	         Axial Load    Vertical        Moment of       Axial       Vertical    Moment of
	                       Load            Inertia         Load        Load        Inertia
	           (1)             (2)            (3)          (4)         (5)         (6)
	        _                                                                             _
	       |   A*L^2/I     0               0           -A*L^2/I    0           0           |   (1)
	       |                                                                               |
	       |   0           12              6*L         0           -12         6*L         |   (2)
	       |                                                                               |
	  E I  |   0           6*L             4*L^2       0           -6*L        2*L^2       |   (3)
	 ----- |                                                                               |
	  L^3  |   -A*L^2/I    0               0           A*L^2/I     0           0           |   (4)
	       |                                                                               |
	       |   0           -12             -6*L        0           12          -6*L        |   (5)
	       |                                                                               |
	       |   0           6*L             2*L^2       0           -6*L        4*L^2       |   (6)
	       |_                                                                             _|
	*/

	// Translation along the Axis at Point A w/ respect to the other Degree's of Freedom
	K(0, 0) = E * A / L;
	K(0, 1) = 0.0;
	K(0, 2) = 0.0;
	K(0, 3) = -E * A / L;
	K(0, 4) = 0.0;
	K(0, 5) = 0.0;
			
	// Translation along the Perpendicular Axis at Point A w/ respect to the other Degree's of Freedom
	K(1, 0) = 0.0;
	K(1, 1) = 12 * E * I / pow(L, 3);
	K(1, 2) = 6 * E * I / pow(L, 2);
	K(1, 3) = 0.0;
	K(1, 4) = -12 * E * I / pow(L, 3);
	K(1, 5) = 6 * E * I / pow(L, 2);
			
	// Rotational along the Out-Of-Plane Axis at Point A w/ respect to the other Degree's of Freedom
	K(2, 0) = 0.0;
	K(2, 1) = 6 * E * I / pow(L, 2);
	K(2, 2) = 4 * E * I / L;
	K(2, 3) = 0.0;
	K(2, 4) = -6 * E * I / pow(L, 2);
	K(2, 5) = 2 * E * I / L;
			
	// Translation along the Axis at Point B w/ respect to the other Degree's of Freedom
	K(3, 0) = -E * A / L;
	K(3, 1) = 0.0;
	K(3, 2) = 0.0;
	K(3, 3) = E * A / L;
	K(3, 4) = 0.0;
	K(3, 5) = 0.0;
			
	// Translation along the Perpendicular Axis at Point B w/ respect to the other Degree's of Freedom
	K(4, 0) = 0.0;
	K(4, 1) = -12 * E * I / pow(L, 3);
	K(4, 2) = -6 * E * I / pow(L, 2);
	K(4, 3) = 0.0;
	K(4, 4) = 12 * E * I / pow(L, 3);
	K(4, 5) = -6 * E * I / pow(L, 2);
			
	// Rotational along the Out-Of-Plane Axis at Point B w/ respect to the other Degree's of Freedom
	K(5, 0) = 0.0;
	K(5, 1) = 6 * E * I / pow(L, 2);
	K(5, 2) = 2 * E * I / L;
	K(5, 3) = 0.0;
	K(5, 4) = -6 * E * I / pow(L, 2);
	K(5, 5) = 4 * E * I / L;

	return K;
};

/// <summary>Include the effects of bending on the geometric stiffness matrix, the strain due to flexure.</summary>
mat Beam2D::GetLocalGeometricStiffnessMatrix(double AxialLoad)
{
	// See "Matrix Structural Analysis, 2nd Edition", McGuire, Gallagher, and Ziemian.
	// Define the Geometric Stiffness Matrix
	mat Kg(6, 6);
	
	// Set the Axial Load.
	double P = AxialLoad;

	// Get the Length of the Beam.
	double L = GetLength();

	// Initialize the Geometric Stiffness Matrix.
	Kg.fill(0.0);

	//			Axial		Vertical	Moment of		Axial		Vertical    Moment of
	//			Load        Load        Inertia         Load        Load        Inertia
	//           (1)        (2)         (3)				(4)         (5)         (6)
	//        _                                                                            _
	//       |   1			0			0				-1			0			0			|   (1)
	//       |                                                                              |
	//       |   0          6/5         L/10			0           -6/5        L/10        |   (2)
	//       |                                                                              |
	//  Fx2  |   0          L/10        2L^2/15			0           -L/10       -L^2/30     |   (3)
	// ----- |                                                                              |
	//  L    |   -1			0           0				1			0           0           |   (4)
	//       |                                                                              |
	//       |   0          -6/5        -L/10			0           6/5         -L/10       |   (5)
	//       |                                                                              |
	//       |   0          L/10        -L^2/30			0           -L/10       2L^2/15     |   (6)
	//       |_                                                                            _|
	        
	// Translation along the Axis at Point A w/ respect to the other Degree's of Freedom
	Kg(0, 0) = P / L;
	Kg(0, 1) = 0.0;
	Kg(0, 2) = 0.0;
	Kg(0, 3) = -P / L;
	Kg(0, 4) = 0.0;
	Kg(0, 5) = 0.0;
			
	// Translation along the Perpendicular Axis at Point A w/ respect to the other Degree's of Freedom
	Kg(1, 0) = 0.0;
	Kg(1, 1) = (6 / 5) * (P / L);
	Kg(1, 2) = (L / 10) * (P / L);
	Kg(1, 3) = 0.0;
	Kg(1, 4) = (-6 / 5) * (P / L);
	Kg(1, 5) = (L / 10) * (P / L);
			
	// Rotational along the Out-Of-Plane Axis at Point A w/ respect to the other Degree's of Freedom
	Kg(2, 0) = 0.0;
	Kg(2, 1) = (L / 10) * (P / L);
	Kg(2, 2) = (2 * pow(L, 2) / 15) * (P / L);
	Kg(2, 3) = 0.0;
	Kg(2, 4) = (-L / 10) * (P / L);
	Kg(2, 5) = (-pow(L, 2) / 30) * (P / L);
			
	// Translation along the Axis at Point B w/ respect to the other Degree's of Freedom
	Kg(3, 0) = -P / L;
	Kg(3, 1) = 0.0;
	Kg(3, 2) = 0.0;
	Kg(3, 3) = P / L;
	Kg(3, 4) = 0.0;
	Kg(3, 5) = 0.0;
			
	// Translation along the Perpendicular Axis at Point B w/ respect to the other Degree's of Freedom
	Kg(4, 0) = 0.0;
	Kg(4, 1) = (-6 / 5) * (P / L);
	Kg(4, 2) = (-L / 10) * (P / L);
	Kg(4, 3) = 0.0;
	Kg(4, 4) = (6 / 5) * (P / L);
	Kg(4, 5) = (-L / 10) * (P / L);
			
	// Rotational along the Out-Of-Plane Axis at Point B w/ respect to the other Degree's of Freedom
	Kg(5, 0) = 0.0;
	Kg(5, 1) = (L / 10) * (P / L);
	Kg(5, 2) = (-pow(L, 2) / 30) * (P / L);
	Kg(5, 3) = 0.0;
	Kg(5, 4) = (-L / 10) * (P / L);
	Kg(5, 5) = (2 * pow(L, 2) / 15) * (P / L);

	return Kg;
};

/// <summary>Get the Elastic Stfffness Matrix of the release beam.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
mat Beam2D::GetReleaseElasticStiffnessMatrix()
{
	// Permutation Matrix.
	mat P = GetPermutationMatrix();
	
	P.print("Permutation Matrix");
	// Return the Local Stiffness Matrix
	mat k = GetLocalStiffnessMatrix();

	k.print("Original Matrix");

	// Get the Permutated Matrix.
	mat kP = P.t() * k * P;

	// Get the Release Index
	int TotalIndex = 6;
	int ReleaseIndex = this -> GetNumberOfRelease();
	int FixedIndex = TotalIndex - ReleaseIndex;

	// Developed the Release Matrix. (See Kassimali)
	if (ReleaseIndex != 0)
	{
		// Fixed-Fixed Matrix
		mat kP_ff = kP.submat(0, 
							  0, 
							  FixedIndex - 1, 
							  FixedIndex - 1);

		// Fixed-Released Matrix.
		mat kP_fr = kP.submat(0, 
							  FixedIndex, 
							  FixedIndex - 1, 
							  TotalIndex - 1);

		// Released-Fixed Matrix.
		mat kP_rf = kP.submat(FixedIndex, 
							  0, 
							  TotalIndex - 1, 
							  FixedIndex - 1);

		// Released-Released Matrix.
		mat kP_rr = kP.submat(FixedIndex, 
							  FixedIndex, 
							  TotalIndex - 1, 
							  TotalIndex - 1);

		

		// Declare the inverse of the release matrix.
		mat kP_rr_inv = arma::inv(kP_rr);

		// Create the Modified Release Stiffness Matrix.
		mat kMod = kP_ff - kP_fr * kP_rr_inv * kP_rf;

		// Add the rows back.
		kMod = arma::resize(kMod, 6, 6);

		// Return the Permutated Matrix.
		kP = P * kMod * P.t();

		if (false) 
		{
			P.print("Permutation Matrix");
			
			k.print("Original Matrix");

			kP_ff.print("Fixed-Fixed");
			kP_fr.print("Fixed-Released");
			kP_rf.print("Released-Fixed");
			kP_rr.print("Released-Released");
		
			kMod.print("Modified Stiffness");
			kP.print("NEW");
		};
	};

	return kP;
};

///<summary>Return the release magnitude in terms of nodal form.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
mat Beam2D::GetReleaseForceVector(mat TangentStiffnessMatrix, 
								  mat FreeDisplacements)
{
	mat ForceVector(6, 1);

	// Permutation Matrix.
	mat P = GetPermutationMatrix();
	
	// Return the Local Stiffness Matrix
	mat k = TangentStiffnessMatrix;

	// Get the Permutated Matrix.
	mat kP = P.t() * k * P;

	// Get the Release Index
	int TotalIndex = 6;
	int ReleaseIndex = this -> GetNumberOfRelease();
	int FixedIndex = TotalIndex - ReleaseIndex;

	// Developed the Release Matrix. (See Kassimali)
	if (ReleaseIndex != 0)
	{
		// Fixed-Fixed Matrix
		mat kP_ff = kP.submat(0, 
							  0, 
							  FixedIndex - 1, 
							  FixedIndex - 1);
		

		// Fixed-Released Matrix.
		mat kP_fr = kP.submat(0, 
							  FixedIndex, 
							  FixedIndex - 1, 
							  TotalIndex - 1);
		

		// Released-Fixed Matrix.
		mat kP_rf = kP.submat(FixedIndex, 
							  0, 
							  TotalIndex - 1, 
							  FixedIndex - 1);
		

		// Released-Released Matrix.
		mat kP_rr = kP.submat(FixedIndex, 
							  FixedIndex, 
							  TotalIndex - 1, 
							  TotalIndex - 1);
		
		// Declare the slow inverse form of the release matrix.
		mat kP_rr_inv = arma::inv(kP_rr);

		// Get the modified release matrix.
		mat kMod = kP_ff - kP_fr * kP_rr_inv * kP_rf;

		// Add the release row with zeros.
		kMod = arma::resize(kMod, 6, 6);

		// Re-Permutated the Matrix.
		mat k_Release = P * kMod * P.t();

		// Get the known release force vectors.
		mat f(6, 1);

		f(0, 0) = MyReleaseNodeAMagnitude[0];
		f(1, 0) = MyReleaseNodeAMagnitude[1];
		f(2, 0) = MyReleaseNodeAMagnitude[2];

		f(3, 0) = MyReleaseNodeBMagnitude[0];
		f(4, 0) = MyReleaseNodeBMagnitude[1];
		f(5, 0) = MyReleaseNodeBMagnitude[2];

		// Return the Permutated Matrix.
		mat fP = P.t() * f;

		// Get the Fixed Force Vector.
		mat fP_f = fP.submat(0, 
							 0, 
							 FixedIndex - 1,
							 0);
		
		// Get the Released Force Vector.
		mat fP_r = fP.submat(FixedIndex,
							 0, 
							 TotalIndex - 1,
							 0);

		// Get the Influence Force Vector from the release.
		mat fP_Mod = kP_fr * kP_rr_inv * fP_r;

		// Add the remaing rows.
		fP_Mod = arma::resize(fP_Mod, 6, 1);

		for (size_t i = 0; i < ReleaseIndex; i++)
		{
			// Store the release moment into the system.
			fP_Mod(i + FixedIndex, 0) = fP_r(i, 0);
		};

		// Get the release form back.
		mat f_Release = P * fP_Mod;

		// Get the Internal Forces.
		ForceVector = k_Release * FreeDisplacements + f_Release;

		if (true) 
		{
			P.print("Permutation Matrix");
			
			k.print("Original Matrix");

			kP_ff.print("Fixed-Fixed");
			kP_fr.print("Fixed-Released");
			kP_rf.print("Released-Fixed");
			kP_rr.print("Released-Released");

			kMod.print("Modified Stiffness");
			k_Release.print("New Release Stiffness Matrix");

			fP_f.print("Fixed Forces");
			fP_r.print("Release Forces");
		
			fP_Mod.print("Moified Load Vector");

			f_Release.print("Modified Release Load Vector");

			ForceVector.print("Force Vector");
		};
		
	};

	return ForceVector;
};

/// <summary>Return the Permutation Matrix of the beam element looking to see if any of the
/// degrees of freedom is released.</summary>
mat Beam2D::GetPermutationMatrix()
{
	// Define the Permutation Matrix.
	mat P(6, 6);
	
	// Initialize the Matrix.
	P.fill(0.0);

	int ConstrainIndex = 0;
	int ReleaseIndex = 6 - this -> GetNumberOfRelease();

	int Row = 0;

	// Look through Node A.
	for (int i = 0; i < 3; i++)
	{
		if(MyReleaseNodeA[i] == false) 
		{
			// Store the Permutation Index.
			P(Row, ConstrainIndex) = 1.0;
			
			// Increment the Constrain Index.
			ConstrainIndex++;
		}
		else
		{
			// Store the Permutation Index.
			P(Row, ReleaseIndex) = 1.0;
			
			// Increment the Release Index.
			ReleaseIndex++;
		}
		
		// Increment the Row Index.
		Row++;
	}

	// Look through Node B.
	for (int j = 0; j < 3; j++)
	{
		if(MyReleaseNodeB[j] == false) 
		{
			// Store the Permutation Index.
			P(Row, ConstrainIndex) = 1.0;
			
			// Increment the Constrain Index.
			ConstrainIndex++;
		}
		else
		{
			// Store the Permutation Index.
			P(Row, ReleaseIndex) = 1.0;

			// Increment the Row Index.
			ReleaseIndex++;
		}
	
		// Increment the Row Index.
		Row++;
	}
	return P;
};

/// <summary></summary>
int Beam2D::GetNumberOfRelease()
{
	int NumOfRelease = 0;
	
	// Count the Number of Release.
	for (int i = 0; i < 3; i++)
	{
		// Count Node A.
		if(MyReleaseNodeA[i] == true)
		{
			NumOfRelease++;
		}
		
		// Count Node B.
		if(MyReleaseNodeB[i] == true)
		{
			NumOfRelease++;
		}
	};

	return NumOfRelease;
};

/// <summary>Return the Constraint List.</summary>
vector <int> Beam2D::GetConstraintList()
{
	vector <int> Indexes;
	
	for (int i = 0; i < DOF; i++)
	{
		// Count Node A.
		if(MyReleaseNodeA[i] == false)
		{
			Indexes.push_back(i);
		}
	};

	for (int j = DOF; j < 2*DOF; j++)
	{
		// 
		if(MyReleaseNodeB[j] == false)
		{
			Indexes.push_back(j);
		};
	};

	return Indexes;
};

/// <summary>Return the Release List.</summary>
vector <int> Beam2D::GetReleaseList()
{
	vector <int> Indexes;
	
	for (int i = 0; i < DOF; i++)
	{
		// Count Node A.
		if(MyReleaseNodeA[i] == true)
		{
			Indexes.push_back(i);
		}
	};

	for (int j = DOF; j < 2*DOF; j++)
	{
		// 
		if(MyReleaseNodeB[j] == true)
		{
			Indexes.push_back(j);
		};
	};

	return Indexes;
};


mat Beam2D::GetTransformationMatrix()
{
	// Return the Transformation Matrix.
	return this -> GetTransformationMatrix(this -> GetAngleXAxis());
};

// Return the Transformation Matrix for a Beam. (Radians)
mat Beam2D::GetTransformationMatrix(double Angle) 
{
	// Define Transformation Matrix.
	mat T(6, 6);

	// Initialize the Matrix.
	T.fill(0.0);

	//         Axial Load    Vertical        Moment of       Axial       Vertical    Moment of
	//                       Load            Inertia         Load        Load        Inertia
	//           (1)             (2)            (3)          (4)         (5)         (6)
	//        _                                                                             _
	//       |   cos(theta)  sin(theta)          0            0           0           0       |   (1)
	//       |                                                                                |
	//       |   -sin(theta) cos(theta)          0            0           0           0       |   (2)
	//       |                                                                                |
	//       |   0           0                   1            0           0           0       |   (3)
	//       |                                                                                |
	//       |   0           0                   0            cos(theta)  sin(theta)  0       |   (4)
	//       |                                                                                |
	//       |   0           0                   0            -sin(theta) cos(theta)  0       |   (5)
	//       |                                                                                |
	//       |   0           0                   0            0           0           1       |   (6)
	//       |_                                                                              _|
			
	// 1st Row
	T(0,0) = cos(Angle);
	T(0,1) = sin(Angle);

	// 2nd Row
	T(1,0) = -sin(Angle);
	T(1,1) = cos(Angle);

	// 3rd Row
	T(2,2) = 1.0;

	// 4th Row
	T(3,3) = cos(Angle);
	T(3,4) = sin(Angle);

	// 5th Row
	T(4,3) = -sin(Angle);
	T(4,4) = cos(Angle);

	// 6th Row
	T(5,5) = 1.0;

	return T;
};

///<summary> Return the Global Stiffness Matrix </summary>
///<returns>Return in arma::mat type.</returns>
///<remarks>description</remarks>
mat Beam2D::GetGlobalStiffnessMatrix() 
{
	// Get the Transformation Matrix.
	mat T = GetTransformationMatrix(GetAngleXAxis());

	// Get the Local Stiffness Matix.
	mat k = GetLocalStiffnessMatrix();

	// Transform the Local Stiffness Matix to the Global Stiffness Matrix.
	mat K = T.t() * k * T;

	return K;
};

///<summary></summary>
///<returns>Return in arma::mat type.</returns>
///<remarks>description</remarks>
mat Beam2D::GetGlobalGeometricStiffnessMatrix()
{
	// Find the angle of the node load.
	double AngleX = this -> GetAngleXAxis();

	// Find the Point Load.
	double PointLoad = 0.0;

	// Get the Load A.
	mat LoadA = MyNodeA -> GetLocalForceVector();

	// Add the Point Load for Node A.
	PointLoad += LoadA(0, 0) * sin(AngleX) + LoadA(1, 0) * cos(AngleX);

	// Get the Load B.
	mat LoadB = MyNodeB -> GetLocalForceVector();

	// Add the Point Load for Node B.	
	PointLoad += LoadB(0, 0) * sin(AngleX) + LoadB(1, 0) * cos(AngleX);

	// Find the global geometric stiffness matrix.
	return this -> GetGlobalGeometricStiffnessMatrix(PointLoad);
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
mat Beam2D::GetGlobalGeometricStiffnessMatrix(mat LocalLoads)
{
	return this ->GetGlobalGeometricStiffnessMatrix(LocalLoads(3, 0));
};

///<summary>Return the global geometric stiffness matrix given the axial load.</summary>
///<param name="AxialLoad">Axial force applied on the beam.</param>
///<returns>description</returns>
///<remarks>description</remarks>
mat Beam2D::GetGlobalGeometricStiffnessMatrix(double AxialLoad)
{

	// Get the Transformation Matrix.
	mat T = this -> GetTransformationMatrix(GetAngleXAxis());

	// Get the Local GeometricStiffness Matix.
	mat k = this -> GetLocalGeometricStiffnessMatrix(AxialLoad);

	// Transform the Local Stiffness Matix to the Global Stiffness Matrix.
	mat K = T.t() * k * T;

	return K;
};

/// <summary> Return the Permutated Stiffness Matrix.
/*
	|					|	|		|		|		|		|		|
	|	Kcc		Kcr		|	|	Uc	|		|	Fc	|		|FEM_c	|
	|					|	|		|		|		|		|		|
	|					|	|		|	=	|		|	+	|		|
	|	Krc		Krr		|	|	Ur	|		|	Fr	|		|FEM_r	|
	|					|	|		|		|		|		|		|

	Systems of Equation:
	[Kcc][Uc] + [Kcr][Ur] = [Fc]
	[Krc][Uc] + [Krr][Ur] = [0]

	Uc = displacement at the constrain DOF. (0 for our scenario)
	Fc = fixed end forces at the constrain nodes.
	FEM-c = no release fixed end forces at the constrain.

	Ur = displacement at the release DOF. 
	Fr = fixed end forces at the release nodes. (a known value - 0 for pure release)
	FEM-r = no release fixed end forces at the release.
	
	Substitute Ur for the 1st equation with the 2nd equation to find Fc.
	1st	::	[Ur] = [Krr]^-1*[Kcr][Uc]
	2nd	::	[Kcc][Uc] + [Kcr]{[Krr]^-1*[Kcr][Uc]} = [Fc]

	[K_modified] = [Kcc] - [Kcr]*[Krr]^-1*[Krc]
*/
///</summary>
mat Beam2D::GetLocalReleaseStiffnessMatrix(const arma::mat StiffnessMatrix, int NumOfRelease)
{
	// Return the Permutation Matrix.
	mat P = this -> GetPermutationMatrix();

	// Get the New Permutated Matrix
	mat K_permutated = P.t() * StiffnessMatrix * P;
	
	// Number of Constrained DOF
	int NumOfConstrain = 2 * DOF - NumOfRelease;
	
	// Get the Submatrix from the permutated matrix.
	mat Kcc = K_permutated.submat(0, 0, NumOfConstrain - 1, NumOfConstrain - 1);
	mat Kcr = K_permutated.submat(0, NumOfConstrain, NumOfConstrain - 1, 2 * DOF - 1);
	mat Krc = K_permutated.submat(NumOfConstrain, 0, (2 * DOF) - 1, NumOfConstrain - 1);
	mat Krr = K_permutated.submat(NumOfConstrain, NumOfConstrain, (2 * DOF) - 1, (2 * DOF) - 1);

	// Get the Modified Version.
	mat K_modified = Kcc - (Kcr * inv(Krr) * Krc);

	// Declare the New Matrix with the Re-order.
	mat K_new(2*DOF, 2*DOF);
	
	// Fill with 0.0.
	K_new.fill(0.0);

	// Append the Submatrix into this new one.
	K_new.submat(0, 0, NumOfConstrain - 1, NumOfConstrain - 1) = K_modified;

	K_new.print();
	
	K_new = P * K_new * P.t();

	K_new.print();

	return K_new;
};

///<summary>Return the Length of the beam from the nodal axis.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double Beam2D::GetLength() 
{
	double X1 = GetXCoordinateNodeA();
	double Y1 = GetYCoordinateNodeA();
	double X2 = GetXCoordinateNodeB();
	double Y2 = GetYCoordinateNodeB();

	return sqrt(pow(X1 - X2, 2) + pow(Y1 - Y2, 2));
};

///<summary>Return the Angle of the Beam.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double Beam2D::GetAngleXAxis() {
	//              ---B              
    //           ---    \ LEG BC
    //        ---        \
    //     ---            \
    //  ---                \
    // A------------------- C 
    //       LEG AC
    // ___________________> X-Axis
	
	// Coordinates
	double Length = GetLength();

	double XA = GetXCoordinateNodeA();
	double YA = GetYCoordinateNodeA();
	double XB = GetXCoordinateNodeB();
	double YB = GetYCoordinateNodeB();
	double XC = Length + XA;
	double YC = YA;
	
	double AB[2];
	AB[0] = XB - XA;
	AB[1] = YB - YA;

	double AC[2];
	AC[0] = XC - XA;
	AC[1] = YC - YA;

	// Dot Product
	// AB * AC = |AB| |AC| COS(THETA)
	
	// AB * AC / |AB| |AC|
	double Product = (AB[0] * AC[0] + AB[1] * AC[1]) / pow(Length, 2);
	
	double Angle = 0.0;
	
	// If the Product is Negative, than the angle is taken bottom side.
	if (AB[1] >= AC[1]) {
		Angle = acos(Product);
	} else {
		Angle = 2*PI - acos(Product);
	};
	
	// Return in Radians;
	return Angle;
};

/// <summary>Get Node A X Coordinate.</summary>
double Beam2D::GetXCoordinateNodeA() {
	return MyNodeA -> GetX();
};

// Get Node B X Coordinate.
double Beam2D::GetXCoordinateNodeB() {
	return MyNodeB -> GetX();
};

// Get Node A Y Coordinate.
double Beam2D::GetYCoordinateNodeA() {
	return MyNodeA -> GetY();
};

// Get Node B Y Coordinate.
double Beam2D::GetYCoordinateNodeB() {
	return MyNodeB -> GetY();
};

// Print Beam Information
void Beam2D::Print(bool ShowMatrix) {
	cout << "Beam ID: \t" << MyID << "\t Address: \t" << this << endl;
	cout << "Node A: \t" << MyNodeA -> GetID() << "\t Address: \t" << MyNodeA <<endl;
	cout << "Node B: \t" << MyNodeB -> GetID() << "\t Address: \t" << MyNodeB <<endl;
	cout << endl;
	
	// Add the Beam Loads Address.
	for (size_t i = 0; i < MyVectorBeamLoads.size(); i++)
	{
		cout << "Beam Load Address: \t" << MyVectorBeamLoads[i] << endl;
	};

	// Check to see if we want to show the matrix.
	if (ShowMatrix)
	{
		mat T = GetTransformationMatrix(this -> GetAngleXAxis());
		mat k = GetLocalStiffnessMatrix();
		mat K = T.t() * k * T;

		T.print("T :");
		k.print("k :");
		K.print("K :");
	};
};

///<summary>Add the Pointer of the Beam Load inside the Beam.</summary>
///<param name="name">Pointer to the beam load.</param>
///<remarks>This is used when the local forces needs to be calculated and the beam
/// needs to know what forces are acting on it.</remarks>
void Beam2D::AddLoad(BeamLoad2D* Load)
{
	MyVectorBeamLoads.push_back(Load);
};

///<summary>Clear all the load pointers connected to the beam.</summary>
///<remarks>description</remarks>
void Beam2D::ClearLoads()
{
	MyVectorBeamLoads.clear();
};

///<summary>Return the total load force vector.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
mat Beam2D::GetTotalLocalForceVector()
{
	return this -> GetBeamLocalForceVector() + this -> GetNodeLocalForceVector();
};

///<summary>Return the beam load force vector.  Nodal forces are excluded.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
mat Beam2D::GetBeamLocalForceVector()
{
	// Declare the Force Vector.
	mat LocalForceVector(6, 1);

	// Initialize the Force Vector.
	LocalForceVector.fill(0.0);

	// Loop to find the forces.
	for (int i = 0; i < MyVectorBeamLoads.size(); i++)
	{
		// Add the Local Force Vector.
		LocalForceVector = LocalForceVector + MyVectorBeamLoads[i] -> GetLocalLoadVector();
	};

	// Return the Force Vector.
	return LocalForceVector;
};

///<summary>Return the Node load force vector.  Beam forces are excluded.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
mat Beam2D::GetNodeLocalForceVector()
{
	// Declare the Force Vector.
	mat LocalForceVector(6, 1);

	// Initialize the Force Vector.
	LocalForceVector.fill(0.0);

	// Get the Load at Node A.
	mat NodeALoad = MyNodeA -> GetLocalForceVector();
	mat NodeBLoad = MyNodeB -> GetLocalForceVector();

	LocalForceVector(0, 0) = NodeALoad(0, 0);
	LocalForceVector(1, 0) = NodeALoad(1, 0);
	LocalForceVector(2, 0) = NodeALoad(2, 0);
	LocalForceVector(3, 0) = NodeBLoad(0, 0);
	LocalForceVector(4, 0) = NodeBLoad(1, 0);
	LocalForceVector(5, 0) = NodeBLoad(2, 0);
	
	// Return the Transformation Matix.
	mat T = this -> GetTransformationMatrix();

	return T * LocalForceVector;
};

///<summary></summary>
///<returns>description</returns>
///<remarks>description</remarks>
mat Beam2D::GetBeamGlobalForceVector()
{
	mat P = this -> GetBeamLocalForceVector();
	mat T = this -> GetTransformationMatrix();

	return T * P;
};

///<summary>Return the corresponding plastic moment based off of the direction of the internal moment for Node A. 
/// For positive curvature requires a negative applied moment, with reverse order for negative curvature.</summary>
///<param name="InternalMoment"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double Beam2D::GetPlasticMomentCapacityNodeA(double InternalMoment)
{
	// Declare Plastic Moment Variable.
	double PlasticMoment;

	// Check if the internal moment is positive or negative.
	if (InternalMoment > 0)
	{
		// If the applied moment is negative, then the section is in negative bending
		// and return the retrospect negative plastic moment.

		PlasticMoment = MySection -> GetNegativePlasticMoment();
	}
	else
	{
		// If the applied moment is positive, then the section is in positive bending
		// and return the retrospect positve plastic moment.
		
		PlasticMoment = MySection -> GetPositivePlasticMoment();
	};

	return PlasticMoment;
};

///<summary>Return the corresponding plastic moment based off of the direction of the internal moment for Node B. 
/// For positive curvature requires a positive applied moment, with reverse order for negative curvature.</summary>
///<param name="InternalMoment"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double Beam2D::GetPlasticMomentCapacityNodeB(double InternalMoment)
{
	// Declare Plastic Moment Variable.
	double PlasticMoment;

	// Check if the internal moment is positive or negative.
	if (InternalMoment > 0)
	{
		// If the applied moment is positive, then the section is in positive bending
		// and return the retrospect positve plastic moment.
		
		PlasticMoment = MySection -> GetPositivePlasticMoment();
	}
	else
	{
		// If the applied moment is negative, then the section is in negative bending
		// and return the retrospect negative plastic moment.

		PlasticMoment = MySection -> GetNegativePlasticMoment();
	};

	return PlasticMoment;
};

///<summary>Return the effective mass from this element at Node A.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
double Beam2D::GetNodeAMass()
{
	// Declare the Mass of variables.
	// mass = density * Area * length
	double Mass = MySection -> GetDensity() * MySection -> GetArea() * this ->GetLength();

	return Mass / 2.0;
};

///<summary>Return the effective mass from this element at Node B.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
double Beam2D::GetNodeBMass()
{
	// Declare the Mass of variables.
	// mass = density * Area * length
	double Mass = MySection -> GetDensity() * MySection -> GetArea() * this ->GetLength();

	return Mass / 2.0;
};

///<summary>Find the mass vector for the x, y, and z degrees of freedom of the beam.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
mat Beam2D::GetMassVector()
{
	// Declare the Mass Vector.
	mat MassVector(6, 1);

	// Set the Mass Vector for Node A and Node B.

	// Node A :: (Translation X, Translation Y, Rotation Z)
	double NodeAMass = this -> GetNodeAMass();

	MassVector(0, 0) = NodeAMass;	// Mass in Node A X-Translation.
	MassVector(1, 0) = NodeAMass;	// Mass in Node A Y-Translation.
	MassVector(2, 0) = NodeAMass;	// TODO: This is possibly incorrect for the inertial mass. But at the time being this is okay.

	// Node B :: (Translation X, Translation Y, Rotation Z)
	double NodeBMass = this -> GetNodeBMass();

	MassVector(3, 0) = NodeBMass;	// Mass in Node B X-Translation.
	MassVector(4, 0) = NodeBMass;	// Mass in Node B Y-Translation.
	MassVector(5, 0) = NodeBMass;	// TODO: This is possibly incorrect for the inertial mass. But at the time being this is okay.

	return MassVector;
};

///<summary>Return mass self weight factor for a uniform load.</summary>
///<returns>Returns in mass without the onset of gravity.</returns>
///<remarks></remarks>
double Beam2D::GetSelfWeightFactor()
{
	return MySection -> GetMassPerUnitLength();
};

///<summary></summary>
///<param name="GlobalDisplacements"></param>
///<returns>description</returns>
///<remarks>description</remarks>
mat Beam2D::GetLocalInternalForces(mat GlobalDisplacements)
{
	// 
	mat ForceVector(6, 1);
	
	// Stiffness Matrix.
	mat StiffnessMatrix = this -> GetLocalStiffnessMatrix();

	// Return the Transformation Matix.
	mat TransformationMatrix  = this -> GetTransformationMatrix();

	// Get the force vector.
	ForceVector = StiffnessMatrix * TransformationMatrix * GlobalDisplacements;

	// Return the force vector.
	return ForceVector;
};
