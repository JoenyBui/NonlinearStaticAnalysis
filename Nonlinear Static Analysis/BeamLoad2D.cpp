#include "StdAfx.h"
#include "BeamLoad2D.h"

using namespace std;
using namespace arma;

/// <summary></summary>
BeamLoad2D::BeamLoad2D(void)
{
}

/// <summary></summary>
BeamLoad2D::~BeamLoad2D(void)
{
}

/// <summary>Assignment Operator.</summary>
BeamLoad2D& BeamLoad2D::operator =(const BeamLoad2D &Load)
{
	// Storing the Loads.
	MyBeam = Load.MyBeam;
	
	// Return a pointer to this instace.
	return *this;
}

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void BeamLoad2D::SetBeam(Beam2D* PtrBeam)
{
	// Change the Address to the New Beam.
	MyBeam = PtrBeam;
};

///<summary>Add the load pointer to the beam.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void BeamLoad2D::SetLoadPointerToBeam(bool AddPointerToBeam)
{
	if (AddPointerToBeam)
	{
		// Add the Pointers to the Beam.
		MyBeam -> AddLoad(this);
	};
};

/// <summary>Return the Beam ID that the load acts on.</summary>
int BeamLoad2D::GetBeamID()
{
	return MyBeam -> GetID();
}

/// <summary>Return the Global Load Vector.</summary>
mat BeamLoad2D::GetGlobalLoadVector()
{
	// Get the Transformation Matrix.
	mat T = MyBeam -> GetTransformationMatrix(MyBeam -> GetAngleXAxis());
	
	// Get Local Load Vector.
	mat f = this -> GetLocalLoadVector();
	
	// Check the Beam for releases. 
	int NumOfRelease = MyBeam -> GetNumberOfRelease();

	if (NumOfRelease != 0)
	{
		// Change the Force Vector to the Equivalent Force Vector.
		//f = this -> GetReleaseForceVector(f, NumOfRelease);
	};

	// Declare Global Force Vector.
	mat F(6, 1);
	
	// Initialize Global Force Function.
	F = T.t() * f;

	// Return the Force Vector.
	return F;
};

/// <summary> Return the Permutated Force Vector Back into Normal Form.
/*
	|					|	|		|		|		|		|		|
	|	Kcc		Kcr		|	|	Uc	|		|	Fc	|		|FEM_c	|
	|					|	|		|		|		|		|		|
	|					|	|		|	=	|		|	+	|		|
	|	Krc		Krr		|	|	Ur	|		|	Fr	|		|FEM_r	|
	|					|	|		|		|		|		|		|

	Systems of Equation:
	[Kcc][Uc] + [Kcr][Ur] = [Fc] + [FEM_c]
	[Krc][Uc] + [Krr][Ur] = [Fr] + [FEM_r]

	Uc = displacement at the constrain DOF. (0 for our scenario)
	Fc = fixed end forces at the constrain nodes.
	FEM-c = no release fixed end forces at the constrain.

	Ur = displacement at the release DOF. 
	Fr = fixed end forces at the release nodes. (a known value - 0 for pure release)
	FEM-r = no release fixed end forces at the release.
	
	Substitute Ur for the 1st equation with the 2nd equation to find Fc.
	1st	::	[Kcr][Ur] = [Fc] + [FEM_c]
	2nd	::	[Ur] = [Krr]^-1*([Fr] + [FEM_r])

	[Fc] = [Kcr]*[Krr]^-1*([Fr] + [FEM_r]) - [FEM_c]

	for pure release.
	[Fc] = [Kcr]*[Krr]^-1*[FEM_r] - [FEM_c}
*/
///</summary>
mat BeamLoad2D::GetReleaseForceVector(const mat &ForceVector, int NumOfRelease)
{
	// Return the Permutation Matrix.
	mat P = MyBeam -> GetPermutationMatrix();
	
	// Return the Stiffness Matrix.
	mat k_local = MyBeam -> GetLocalStiffnessMatrix();
	
	// Get the New Permutated Matrix
	mat k_permutated = P.t() * k_local * P;
	mat f_permutated = P.t() * ForceVector;

	int NumOfConstrain = 2 * DOF - NumOfRelease;
	
	// Get the Submatrix from the permutated matrix.
	mat Kcr = k_permutated.submat(0, NumOfConstrain, NumOfConstrain - 1, 2 * DOF - 1);
	mat Krr = k_permutated.submat(NumOfConstrain, NumOfConstrain, (2 * DOF) - 1, (2 * DOF) - 1);
	
	// Store the Submatrix from the Fixed End Forces.
	mat FEM_c = f_permutated.submat(0, 0, NumOfConstrain - 1, 0);
	mat FEM_r = f_permutated.submat(NumOfConstrain, 0, (2*DOF) - 1, 0);
	
	// Return the New Fixed End Forces at the constraint.
	mat Fc = Kcr * (inv(Krr)*FEM_r) - FEM_c;
	
	// Repermutate back to original order.
	// Add back in the 0.
	mat F_cs(2*DOF, 1);
	
	for (int i = 0; i < NumOfConstrain; i++)
	{
		if (i < NumOfConstrain)
		{
			// Add the New Force Vector.
			F_cs(i, 0) = Fc(i, 0);
		}
		else
		{
			F_cs(i, 0) = 0.0;
		}
	}
	
	mat F = P*F_cs;
	
	return F;
};

/// <summary></summary>
void BeamLoad2D::Print(bool ShowMatrix)
{
	cout << "Beam Load Address: \t" << this << endl;

	cout << "Beam ID: \t" << MyBeam -> GetID() << "\t Address: \t" << MyBeam << endl;
	

	if (ShowMatrix)
	{
		mat f = GetLocalLoadVector();
		mat F = GetGlobalLoadVector();

		cout << "Local Load Vector: " << endl;

		f.print("f :");
		 
		cout << "Global Load Vector: " << endl;

		F.print("F :");
	};

	cout << endl;
}

/// <summary></summary>
vector <NodeLoad2D*> BeamLoad2D::GetNodalLoad()
{
	vector <NodeLoad2D*> Load;
	
	// Get the Global Load Vector.
	mat F = GetGlobalLoadVector();
	
	// Set the Load for each Node.
	// Place into Array for return.
	Load.push_back(new NodeLoad2D(MyBeam -> GetNodeA(), 
										  F(0, 0), 
										  F(1, 0), 
										  F(2, 0)));
	Load.push_back(new NodeLoad2D(MyBeam -> GetNodeB(), 
									  F(3, 0), 
									  F(4, 0), 
									  F(5, 0)));
	return Load;
};

///<summary></summary>
///<param name="Axis"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double BeamLoad2D::GetProjectedAngleofForceToBeam(char Axis)
{
	double Angle = 0.0;

	switch (Axis)
	{
		case 'x':
		case 'X':
			Angle = this -> GetProjectedAngleofForceToBeam(0.0);

			break;
		case 'y':
		case 'Y':
			Angle = this -> GetProjectedAngleofForceToBeam(PI / 2.0);

			break;
		default:

			break;
	}

	return Angle;
};

///<summary></summary>
///<param name="ForceAngle"></param>
///<returns>description</returns>
///<remarks>description</remarks>
double BeamLoad2D::GetProjectedAngleofForceToBeam(double ForceAngle)
{
	// Get Beam Angle.
	double BeamAngle = MyBeam-> GetAngleXAxis();

	return ForceAngle - BeamAngle;
};
