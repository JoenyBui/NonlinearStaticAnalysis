#include "StdAfx.h"
#include "ElasticFrame2D.h"

using namespace std;
using namespace arma;

/// <summary></summary>
void ElasticFrame2D::Initialize(string Name = "Default Name", 
								int ID = -1)
{
	// Initialize the Parameters.
	MyName = Name;
	MyID = ID;

	// Unlock the inititalization.
	MyLock = false;
	IncludeGeometricNonLinearity = false;
	IncludeMaterialNonLinearity = false;

	ShowRatioTables = false;
	
	ShowNodes = false;
	ShowBeams = false;
	ShowNodesDisplacement = false;
	ShowNodeLoads = false;
	ShowBeamLoads = false;	
};

/// <summary>Constructor.</summary>
ElasticFrame2D::ElasticFrame2D(string Name)
{
	// Set the Name of the Frame
	this -> Initialize(Name);
};

/// <summary>Default Constructor.</summary>
ElasticFrame2D::ElasticFrame2D(void) 
{
	this -> Initialize();
};

///<summary>Destructor</summary>
///<returns>description</returns>
///<remarks>description</remarks>
ElasticFrame2D::~ElasticFrame2D(void)
{
	///MyVectorBeamLoads.clear();
};


/// <summary>Copy the Data of the Object.</summary>
void ElasticFrame2D::Clone(const ElasticFrame2D &Frame)
{
	// Copy the Name.
	MyName = Frame.MyName;
	MyID = Frame.MyID;

	// Add Node to the frame.
	this -> AddNode(Frame.MyVectorNodes);

	// Add Section to the Frame
	this -> AddSection(Frame.MyVectorSections);

	// Add Beam to the Frame.
	this -> AddBeam(Frame.MyVectorBeams);

	// Add Loads to the Frame.
	this -> AddNodeLoad(Frame.MyVectorNodeLoads);

	this -> AddBeamLoad(Frame.MyVectorMomentLoads);
	this -> AddBeamLoad(Frame.MyVectorPointLoads);
	this -> AddBeamLoad(Frame.MyVectorPyramidLoads);
	this -> AddBeamLoad(Frame.MyVectorTriangularLoads);
	this -> AddBeamLoad(Frame.MyVectorUniformLoads);

	// Save teh Results.	
	MyLock = Frame.MyLock;
	IncludeGeometricNonLinearity = Frame.IncludeGeometricNonLinearity;
	IncludeMaterialNonLinearity = Frame.IncludeMaterialNonLinearity;

	MyLocalBeamEndForces = Frame.MyLocalBeamEndForces;

	MyVectorDOF = Frame.MyVectorDOF;

	MyElasticStiffnessMatrix = Frame.MyElasticStiffnessMatrix;
	MyGeometricStiffnessMatrix = Frame.MyGeometricStiffnessMatrix;
	MyTangentStiffnessMatrix = Frame.MyTangentStiffnessMatrix;
	MyPermutationMatrix = Frame.MyPermutationMatrix;

	MyMemberForceVector = Frame.MyMemberForceVector;
	MyJointForceVector = Frame.MyJointForceVector;
	MyCombineForceVector = Frame.MyCombineForceVector;

	MyMassVector = Frame.MyMassVector;

	MyExternalForceVector = Frame.MyExternalForceVector;

	MyFreeDOF = Frame.MyFreeDOF;
	MyFixedDOF = Frame.MyFixedDOF;
	MyTotalDOF = Frame.MyTotalDOF;

	MyNodesDisplacements = Frame.MyNodesDisplacements;
};

/// <summary>Clone the entire frame.</summary>
ElasticFrame2D& ElasticFrame2D::operator=(const ElasticFrame2D &Frame)
{
	// Clone all the parameters.
	this -> Clone(Frame);

	// Return a pointer of this.
	return *this;
};

///<summary>Add Nodal Increment to the Frame.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::operator +=(vector <Node2D> &Nodes)
{
	// Verify that the two nodes are the same sizes.
	if (Nodes.size() == MyVectorNodes.size())
	{
		// Loop through the new deform and increment it.
		for (size_t i = 0; i < MyVectorNodes.size(); i++)
		{
			// Get the Nodal Indices.
			vector <int> Indexes = MyVectorNodes[i].GetNodeDOFIndexes();
			
			// Change the X-Location.
			if (Indexes[0] != -1)
			{
				MyVectorNodes[i].IncrementTranslationX(Nodes[i].GetX());
			};

			// Change the Y-Location.
			if (Indexes[1] != -1)
			{
				MyVectorNodes[i].IncrementTranslationY(Nodes[i].GetY());
			};

			// Change the Z-Rotation.
			if (Indexes[2] != -1)
			{
				MyVectorNodes[i].IncrementRotationZ(Nodes[i].GetRotationZ());
			};
		};
	};
};

/// <summary> Copy Constructor. </summary>
///<param name="Frame"></param>
///<remarks>description</remarks>
ElasticFrame2D::ElasticFrame2D(const ElasticFrame2D &Frame)
{
	this -> Clone(Frame);
};

///<summary>Run the static analysis.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
vector <mat> ElasticFrame2D::RunStaticAnalysis(bool CalculateDisplacement)
{
	vector <mat> Results;
	
	// Connect all the References for the Node and the Beams.
	this -> ConnectComponents();

	// Find the Global Stiffness Matrix.
	MyElasticStiffnessMatrix = this -> GetGlobalStiffnessMatrix();

	// Find the Global Joint Force Vector.
	MyJointForceVector = this -> GetGlobalJointForceVector();

	// Find the Global Member Force Vector.
	MyMemberForceVector = this -> GetGlobalMemberForceVector();
	
	// Find the Mass Vector.
	MyMassVector = this -> GetGlobalMassVector();

	// Find the Permutation Matrix.
	MyPermutationMatrix = GetPermutationMatrix();

	// Free Degrees of Freedom.
	MyFreeDOF = this -> GetNumberOfDOF();

	// Total Degrees of Freedom.
	MyTotalDOF = this -> GetDofIndex();

	// Fixed Degrees of Freedom.
	MyFixedDOF = MyTotalDOF - MyFreeDOF;

	// Set the Tangent Stiffness Matrix.
	MyTangentStiffnessMatrix = MyElasticStiffnessMatrix;

	// Set the Force Vector.
	MyCombineForceVector = MyJointForceVector + MyMemberForceVector;

	if (CalculateDisplacement)
	{
		// Clear Results.
		MyNodesDisplacements.clear();

		Results = this -> RunAnalysis();
	};

	// Return the Permutated Global Stiffness Matrix.
	return Results;
};

///<summary></summary>
///<param name="LocalBeamEndForces"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <mat> ElasticFrame2D::RunStaticAnalysis(vector <mat> LocalBeamEndForces)
{
	// Clear Results.
	MyNodesDisplacements.clear();

	// Connect all the References for the Node and the Beams.
	this -> ConnectComponents();

	// Set the Local Beam End Forces.
	MyLocalBeamEndForces = LocalBeamEndForces;

	// Find the Global Stiffness Matrix.
	MyElasticStiffnessMatrix = this -> GetGlobalStiffnessMatrix();

	// Find the Global Geometric Stiffness Matrix.
	MyGeometricStiffnessMatrix = this -> GetGlobalGeometricStiffnessMatrix(MyLocalBeamEndForces);

	// Switch on the Geometric NonLinearity.
	IncludeGeometricNonLinearity = true;

	// Find the Global Joint Force Vector.
	MyJointForceVector = this -> GetGlobalJointForceVector();

	// Find the Global Member Force Vector.
	MyMemberForceVector = this -> GetGlobalMemberForceVector();
	
	// Find the Mass Vector.
	MyMassVector = this -> GetGlobalMassVector();

	// Find the Permutation Matrix.
	MyPermutationMatrix = GetPermutationMatrix();

	// Free Degrees of Freedom.
	MyFreeDOF = this -> GetNumberOfDOF();

	// Total Degrees of Freedom.
	MyTotalDOF = this -> GetDofIndex();

	// Fixed Degrees of Freedom.
	MyFixedDOF = MyTotalDOF - MyFreeDOF;

	// Set the Tangent Stiffness Matrix.
	MyTangentStiffnessMatrix = MyElasticStiffnessMatrix + MyGeometricStiffnessMatrix;

	// Set the Force Vector.
	MyCombineForceVector = MyJointForceVector + MyMemberForceVector;

	// Run full static analysis.
	return this -> RunAnalysis();
};

///<summary></summary>
///<param name="LocalBeamEndForces"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <mat> ElasticFrame2D::RunStaticAnalysisOnResidualForce(std::vector<mat> LocalBeamEndForces, 
															  arma::mat ResidualForce)
{
	// Clear Results.
	MyNodesDisplacements.clear();

	// Connect all the References for the Node and the Beams.
	this -> ConnectComponents();
	
	// Set the Local Beam End Forces.
	MyLocalBeamEndForces = LocalBeamEndForces;

	// Switch on the Geometric NonLinearity.
	IncludeGeometricNonLinearity = true;

	// Find the Global Stiffness Matrix.
	MyElasticStiffnessMatrix = this -> GetGlobalStiffnessMatrix();

	// Find the Global Geometric Stiffness Matrix.
	MyGeometricStiffnessMatrix = this -> GetGlobalGeometricStiffnessMatrix(MyLocalBeamEndForces);

	// Find the Global Joint Force Vector.
	MyJointForceVector = ResidualForce;
	
	// Find the Mass Vector.
	MyMassVector = this -> GetGlobalMassVector();

	// Find the Permutation Matrix.
	MyPermutationMatrix = this -> GetPermutationMatrix();

	// Free Degrees of Freedom.
	MyFreeDOF = this -> GetNumberOfDOF();

	// Total Degrees of Freedom.
	MyTotalDOF = this -> GetDofIndex();

	// Fixed Degrees of Freedom.
	MyFixedDOF = MyTotalDOF - MyFreeDOF;

	// Set the Tangent Stiffness Matrix.
	MyTangentStiffnessMatrix = MyElasticStiffnessMatrix + MyGeometricStiffnessMatrix;

	// Set the Force Vector.
	MyCombineForceVector = MyJointForceVector;

	// Run full static analysis.
	return this -> RunAnalysis();
};

///  <summary> See "Matrix Analysis of Structures" Aslam Kassimali - page 496.
/*
  _			   _		 _			   _		 _							   _	 _			   _
|		Qj		|		|		Qm		|		|		Sff		|		Sfr		|	|		Df		|
|	NDOF * 1	|		|	 NDOF * 1	|		|	NDOF * NDOF	|	NOF * NR	|	|	 NDOF * 1	|
|---------------|	-	|---------------|	=	|---------------|---------------|	|---------------|
|		Rj		|		|		Rm		|		|		Srf		|		Srr		|	|		Dr		|
|	 NR * 1		|		|	  NR * 1	|		|	  NR * NDOF	|	  NR * NR	|	|	  NR * 1	|
|_			   _|		|_			   _|		|_				|			   _|	|_			   _|

Qj,m = free DOF forces
Rj,m = reaction DOF forces
Pf = denotes fixed-joint forces, due to member loads, temerature changes, and fabrication errors corresponding to the free nodes.
Rf = denotes the structure fixed-joint forces corresponding to the restrained coordinates
Dr = effects of support displacement vector corresponds to this.

S = Represents the force at the free vector.
Sfr = submatrix represents the force at a free coordinate caused by a unit displacement of the restrained coordinates.
Srf =
Srr =
*/
/// <summary>
///<param name="Stiffness"></param>
/// <returns> /*
//	0 = Stiffness Matrix for Free DOF contribution to Free DOF.
//	1 = Stiffness Matrix for Restrain DOF contribution to Free DOF.
//	2 = Stiffness Matrix for Free DOF contribution to Restrain DOF.	 
//	3 = Stiffness Matrix for Restrain DOF contribution to Restrain DOF.
//	4 = Member Force Vector for the Free DOF.
//	5 = Member Force Vector for the Restrain DOF.
//	6 = Nodal Force Vector for the Free DOF.
//	7 = Displacement Vector for Restrain DOF.
//	8 = Displacement Vector for Free DOF.
//	9 = Nodal Reactions Vector for Restrain DOF.
///*/</returns>
vector <mat> ElasticFrame2D::RunAnalysis()
{
	// Declare the Matrices.
	vector <mat> Matrices;
	
	// Permutated the Matrices.
	mat K = MyPermutationMatrix.t() * MyTangentStiffnessMatrix * MyPermutationMatrix;

	mat F = MyPermutationMatrix.t() * MyCombineForceVector;
	
	// Free-Free
	mat K_FreeFree = K.submat(0, 
							  0, 
							  MyFreeDOF - 1, 
							  MyFreeDOF - 1);					
	
	// Free-Fixed
	mat K_FreeFixed = K.submat(0, 
		               MyFreeDOF, 
					   MyFreeDOF - 1, 
					   MyTotalDOF - 1);				

	// Fixed - Free
	mat K_FixedFree = K.submat(MyFreeDOF, 
							   0, 
							   MyTotalDOF - 1, 
							   MyFreeDOF - 1);				
	
	// Restrict - Restrict
	mat K_FixedFixed = K.submat(MyFreeDOF, 
								MyFreeDOF, 
								MyTotalDOF - 1, 
								MyTotalDOF - 1);		
	
	// Get the Partition Joint and Member Load Vector.

	// Contribution from the Member Loads.
	mat F_Free = F.submat(0, 
						  0, 
					      MyFreeDOF - 1, 
						  0);
	// Reaction.
	mat F_Fixed = F.submat(MyFreeDOF, 
						   0, 
						   MyTotalDOF - 1, 
						   0);
	
	// Support Displacements.
	mat D_Initial = mat(MyFixedDOF, 1);

	// Initialize all Displacement Vectors to Zeroes.
	D_Initial.fill(0.0);

	// Solve.
	// [Qj + Qm] = Sff * Df + Sfr * Dr
	// |--> Df = Sff^-1 * [Qj + Qm - Sfr*Dr]
	
	// Nodal Displacement for the Free Degrees of Freedom.
	/*  arma::solve(A, B, slow = false)
		- solve a system of linear equations, ie., A*X = B, where X is unknown.
		- for a square matrix A, this function is coneptually the same as X = inv(A)*B, but is more efficient
		- similary functionality to the "\" (left division operator) operator in Matlab/Octave, ie. X = A \ B
		- the number of ro in A and B must be the same
		- if A is known to be a triangular matrix, the solution can be computed faster by explicitly
		  marking the matrix as triangular through trimatu() or trimatl()
		- if A is non-square (and hence also non-triangular), solve() will also try to provide approximate
		  solutions to under-determines as well as over-determined systems
	    - if no solution is found, X is reset and:
			- solve(A, B) throws a std::runtime_error exception
			- solve(X, A, B) returns a bool set to false
		- for matrix size <= 4x4, a fast algorithm is used by default. In rare instances, the fast algorithm
		  might be less precise than the standard algorithm.  To force the use of the standard algorithm, set
		  the slow argument to true
	*/
	mat D_Free = solve(K_FreeFree, F_Free - K_FreeFixed * D_Initial, false);

	// [Rj + Rm] = Srf * Df + Srr * Dr
	// |--> Rj = Srf * Df + Srr * Drr - Rm
	mat R_Joint = K_FixedFree * D_Free + K_FixedFixed * D_Initial - F_Fixed;

	/// TODO After analysis, check if the results are correct. If so lock the results and save the analysis.
	/// Lock the Frame from changes.
	MyLock = true;

	// D = Df + Dr;
	// Re-input the displacements back into a full vector.
	mat FullDisplacementVector = mat(MyTotalDOF, 1);
	
	// Re-input the forces into a full vector including the reactionary force.
	mat FullForceVector = mat(MyTotalDOF, 1);

	// Recombine the displacement vector and the force vector back into the vector cells
	// with nodal indices.
	for (size_t i = 0; i < MyTotalDOF; i++)
	{
		if (i < MyFreeDOF)
		{
			// Add the Free DOF Displacement.
			FullDisplacementVector(i, 0) = D_Free(i, 0);
			FullForceVector(i, 0) = 0.0;
		}
		else
		{
			// Add the Restrain DOF Displacement.
			FullDisplacementVector(i, 0) = D_Initial(i - MyFreeDOF, 0);
			FullForceVector(i, 0) = R_Joint(i - MyFreeDOF, 0);
		};
	};

	// Permutate the Matrix back into global index form and store it.
	MyNodesDisplacements = MyPermutationMatrix * FullDisplacementVector;
	
	// Return the external force vector on the system (including the reactions).
	MyExternalForceVector = MyPermutationMatrix * (FullForceVector) + MyCombineForceVector;

	// Check to see to show the displacements.
	if (ShowNodesDisplacement)
	{
		//MyNodesDisplacements.print("Nodal Displacement:");
	};

	Matrices.push_back(K_FreeFree);		//	0 = Stiffness Matrix for Free DOF contribution to Free DOF.
	Matrices.push_back(K_FreeFixed);	//	1 = Stiffness Matrix for Restrain DOF contribution to Free DOF.
	Matrices.push_back(K_FixedFree);	//	2 = Stiffness Matrix for Free DOF contribution to Restrain DOF.	 
	Matrices.push_back(K_FixedFixed);	//	3 = Stiffness Matrix for Restrain DOF contribution to Restrain DOF.
	
	// Return the Permutated Global Stiffness Matrix.
	return Matrices;
};

///<summary>Return the node external force vector.</summary>
///<remarks>description</remarks>
mat ElasticFrame2D::GetGlobalExternalForceVector()
{
	return MyExternalForceVector;
};

/// <summary>Return the Frame Geometry.</summary>
ElasticFrame2D ElasticFrame2D::GetElasticFrameGeometry()
{
	ElasticFrame2D Frame = ElasticFrame2D();

	// Store the Parameters. (Storing Pointers).
	for (size_t i = 0; i < MyVectorNodes.size(); i++)
	{
		Frame.AddNode(MyVectorNodes[i]);
	};
	
	// Store the Beams and re-reference the Nodes.
	for (size_t j = 0; j < MyVectorBeams.size(); j++)
	{
		Frame.AddBeam(MyVectorBeams[j]);
	};

	// Return Frame Geometry.
	return Frame;
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::SetID(int ID)
{
	// Change the ID of the Frame.
	MyID = ID;
};

/// <summary> Add a New Node to the Frame.  Check if the Node Exist First. </summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddNode(Node2D Node) 
{
	// TODO: There should be a check to make sure the nodes does not have the same label id.
	Node.ClearBeams();
	Node.ClearLoads();
	
	// Add Node to Vector.
	MyVectorNodes.push_back(Node);
};

///<summary></summary>
///<param name="Nodes"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddNode(vector <Node2D> Nodes)
{
	for (size_t i = 0; i < Nodes.size(); i++)
	{
		// Add Nodes one by one.
		this -> AddNode(Nodes[i]);
	};
};

/// <summary> Add a New Beam to the Frame. Must check if the Two Nodes Exist First.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeam(Beam2D Beam) 
{
	this -> AddBeam(Beam, Beam.GetNodeAID(), Beam.GetNodeBID(), Beam.GetSectionID());
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeam(Beam2D Beam, int NodeAID, int NodeBID, int SectionID)
{
	// Get the Indexes for both Node A and Node B.
	int IndexA = this -> GetNodeIndex(NodeAID);
	int IndexB = this -> GetNodeIndex(NodeBID);

	// Get the Indexes for the Material from the ID.
	int IndexSection = this -> GetSectionIndex(SectionID); 

	// Make sure the Nodal are valid.
	if (IndexA != -1 && IndexB != -1 && IndexSection != -1)
	{
		// Set the Node Pointers.
		Beam.SetNode(&MyVectorNodes[IndexA], &MyVectorNodes[IndexB]);
		
		// Set the Material Pointers.
		Beam.SetSection(&MyVectorSections[IndexSection]);

		// Clear all the attach nodes.
		Beam.ClearLoads();

		// Add The Beam.
		MyVectorBeams.push_back(Beam);
	};
};

/// TODO: DELETE FUNCTION
mat ElasticFrame2D::GetDisplacementVector(const ElasticFrame2D &Frame)
{
	mat DisplacementVector;

	//ElasticFrame2D DeformFrame = Frame.GetPlasticDeformFrameGeometry();
	

	return DisplacementVector;
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeam(vector<Beam2D> Beams)
{
	// Loop through all the Beams.
	for (size_t i = 0; i < Beams.size(); i++)
	{
		// Call the Add Beam funciton and check if the beam is valid.
		this -> AddBeam(Beams[i]);
	};
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
void ElasticFrame2D::AddSection(Section2D Section)
{
	MyVectorSections.push_back(Section);
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
void ElasticFrame2D::AddSection(vector <Section2D> Sections)
{
	for (size_t i = 0; i < Sections.size(); i++)
	{
		// Cal
		this ->AddSection(Sections[i]);
	};
};

/// <summary>Add a New Nodal Load to the Frame.</summary>
///<param name="Load"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddNodeLoad(NodeLoad2D Load)
{
	this -> AddNodeLoad(Load, Load.GetNodeID());
};

/// <summary>Add in a vector of Nodal Loads. </summary>
void ElasticFrame2D::AddNodeLoad(vector <NodeLoad2D> Load)
{
	// Loop and add all the nodes in the vector.
	for (size_t i = 0; i < Load.size(); i++)
	{
		this -> AddNodeLoad(Load[i], Load[i].GetNodeID());
	};
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddNodeLoad(NodeLoad2D Load, int NodeID)
{
	// Retreive the Node ID if it exist.
	int Index = this -> GetNodeIndex(NodeID);

	if (Index != -1)
	{
		// Update the Nodal Address.
		Load.SetNode(&MyVectorNodes[Index]);

		// Add Node Loads to the bottom of the vector.
		MyVectorNodeLoads.push_back(Load);
	};
};

/// <summary>Return the desire Pointer address for the Node.</summary>
Node2D& ElasticFrame2D::GetNodeAddress(int ID)
{
	int Index = -1;

	/// TODO Need a better search function.
	for (size_t i = 0; i < MyVectorNodes.size(); i++)
	{
		if (ID == MyVectorNodes[i].GetID())
		{
			// Store the Index of the vector.
			Index = i;

			// Exit Loop
			i = MyVectorNodes.size();
		};
	};

	return MyVectorNodes[Index];
};

/// <summary>Return the desire Pointer address for the Beam.</summary>
Beam2D& ElasticFrame2D::GetBeamAddress(int ID)
{
	int Index = -1;

	/// TODO Nee a better search algorithm.
	for (size_t i = 0; i < MyVectorBeams.size(); i++)
	{
		if (ID == MyVectorNodes[i].GetID())
		{
			// Store the Index of the vector.
			Index = i;
		};
	};

	return MyVectorBeams[Index];
};

/// <summary>Add a New Beam Load to the Frame.</summary>
void ElasticFrame2D::AddBeamLoad(BeamLoad2D &Load)
{
	MyVectorBeamLoads.push_back(&Load);
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(MomentLoad2D Load)
{
	this -> AddBeamLoad(Load, Load.GetBeamID());
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(MomentLoad2D Load, int BeamID)
{
	int Index = this-> GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyVectorBeams[Index]);

		// Save the Load.
		MyVectorMomentLoads.push_back(Load);
	};
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(vector <MomentLoad2D> Load)
{
	for (size_t i = 0; i < Load.size(); i++)
	{
		//Add Beam Load.
		AddBeamLoad(Load[i], Load[i].GetBeamID());
	};
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(PointLoad2D Load)
{
	this -> AddBeamLoad(Load, Load.GetBeamID());
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(PointLoad2D Load, int BeamID)
{
	// Find the Index.
	int Index = this-> GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyVectorBeams[Index]);

		// Save the Load.
		MyVectorPointLoads.push_back(Load);
	};
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(vector <PointLoad2D> Load)
{
	for (size_t i = 0; i < Load.size(); i++)
	{
		//Add Beam Load.
		AddBeamLoad(Load[i], Load[i].GetBeamID());
	};
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(PyramidLoad2D Load)
{
	this -> AddBeamLoad(Load, Load.GetBeamID());
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(PyramidLoad2D Load, int BeamID)
{
	// Find the Index.
	int Index = this-> GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyVectorBeams[Index]);

		// Save the Load.
		MyVectorPyramidLoads.push_back(Load);
	};
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(vector <PyramidLoad2D> Load)
{
	for (size_t i = 0; i < Load.size(); i++)
	{
		//Add Beam Load.
		AddBeamLoad(Load[i], Load[i].GetBeamID());
	};
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(TriangularLoad2D Load)
{
	this -> AddBeamLoad(Load, Load.GetBeamID());
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(TriangularLoad2D Load, int BeamID)
{
	// Find the Index.
	int Index = this-> GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyVectorBeams[Index]);

		// Save the Load.
		MyVectorTriangularLoads.push_back(Load);
	};
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(vector <TriangularLoad2D> Load)
{
	for (size_t i = 0; i < Load.size(); i++)
	{
		//Add Beam Load.
		AddBeamLoad(Load[i], Load[i].GetBeamID());
	};
};

///<summary>Add the Uniform Beam Load.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(UniformLoad2D Load)
{
	this -> AddBeamLoad(Load, Load.GetBeamID());
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(UniformLoad2D Load, int BeamID)
{
	// Find the Index.
	int Index = this-> GetBeamIndex(BeamID);

	// Add the Beam Load if there is a beam.
	if (Index != -1)
	{
		// Change the Beam Location.
		Load.SetBeam(&MyVectorBeams[Index]);

		// Save the Load.
		MyVectorUniformLoads.push_back(Load);
	};
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::AddBeamLoad(vector <UniformLoad2D> Load)
{
	for (size_t i = 0; i < Load.size(); i++)
	{
		//Add Beam Load.
		AddBeamLoad(Load[i], Load[i].GetBeamID());
	};
};

///<summary>This subroutine clear all existing connection between nodes, elements, and loads.  
/// Once this is done, everything is reconnected and then degrees of freedom are set.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::ConnectComponents()
{
	// Clear all the references if any.
	for (size_t e = 0; e < MyVectorNodes.size(); e++)
	{
		MyVectorNodes[e].ClearAll();
	};

	// Add the Beam Address to the Node.
	for (size_t g = 0; g < MyVectorBeams.size(); g++)
	{
		// Clear the Loads.
		MyVectorBeams[g].ClearLoads();

		MyVectorBeams[g].SetBeamPointerToNode();
	};

	// Add the Reference to the Node Load.
	for (size_t h = 0; h < MyVectorNodeLoads.size(); h++)
	{
		// Set Load Reference
		MyVectorNodeLoads[h].SetLoadPointerToNode();
	};
		
	// Add the Moment Load Address to the Vector of Beam Loads.
	for (size_t i = 0; i < MyVectorMomentLoads.size(); i++)
	{
		MyVectorBeamLoads.push_back(&MyVectorMomentLoads[i]);
	};

	// Add the Point Load Address to the Vector of Beam Loads.
	for (size_t j = 0; j < MyVectorPointLoads.size(); j++)
	{
		MyVectorBeamLoads.push_back(&MyVectorPointLoads[j]);
	};

	// Add the Pyramid Load Address to the Vector of Beam Loads.
	for (size_t k = 0; k < MyVectorPyramidLoads.size(); k++)
	{
		MyVectorBeamLoads.push_back(&MyVectorPyramidLoads[k]);
	};

	// Add the Triangular Load Address to the Vector of Beam Loads.
	for (size_t l = 0; l < MyVectorTriangularLoads.size(); l++)
	{
		MyVectorBeamLoads.push_back(&MyVectorTriangularLoads[l]);
	};

	// Add the Uniform Load Address to the Vector of Beam Loads.
	for (size_t m = 0; m < MyVectorUniformLoads.size(); m++)
	{
		MyVectorBeamLoads.push_back(&MyVectorUniformLoads[m]);
	};

	// Add the Reference to Beam Load.
	for (size_t n = 0; n < MyVectorBeamLoads.size(); n++)
	{
		MyVectorBeamLoads[n] -> SetLoadPointerToBeam();
	};
	
	// Set the Degree of Freedom for the Specific Beams inside of the Node.
	this -> SetDOFIndex();
};

/// <summary></summary>
vector <int> ElasticFrame2D::GetNodeList()
{
	// Set the Vector
    vector <int> List;
	
	// Find the Maximum Number of Nodes
    size_t NumNode = MyVectorNodes.size();
	
	// Loop to find the list.
	for (size_t i = 0; i < NumNode; i++) {
		List.push_back(MyVectorNodes[i].GetID());
	};

	return List;
}

/// <summary>Return the Node Indexes List.</summary>
vector <int> ElasticFrame2D::GetNodeIndexesList()
{
	// Declare the Vector.
	vector <int> List;

	// Get the Size of the Node Vector.
	size_t NumNode = MyVectorNodes.size();

	// Loop to return the list.
	for (size_t i = 0; i < NumNode; i++)
	{
		// Return the Node Index List.
		vector <int> NodeList = MyVectorNodes[i].GetNodeDOFIndexes();

		List.push_back(NodeList[0]);
		List.push_back(NodeList[1]);
		List.push_back(NodeList[2]);
	};
	
	// Return the List.
	return List;
};

/// <summary>Return the Node List and find if its restrained or not.</summary>
vector <bool> ElasticFrame2D::GetNodeFixityList() 
{
	// Set the Vector
    vector <bool> List;

	// Find the Maximum Number of Nodes
	size_t NumNode = MyVectorNodes.size();
	
	// Loop to find the list.
	for (size_t i = 0; i < NumNode; i++) {
		List.push_back(MyVectorNodes[i].GetDofTranslationX());
		List.push_back(MyVectorNodes[i].GetDofTranslationY());
		List.push_back(MyVectorNodes[i].GetDofRotationZ());
	};

	return List;
};


/// <summary>Return the Free Degree of Freedom Index.</summary>
int ElasticFrame2D::GetNumberOfFreeDOF(vector<bool> List) {
	/// Number of Vecotr
	int NumOfFree = 0;

	for (size_t i = 0; i < List.size(); i++) {
		if (List[i] == true) 
		{
			// Increment to the number of freedom.
			NumOfFree++;
		};
	};

	return NumOfFree;
};

/// <summary>Return the Global Stiffness Matrix with only the Degrees of Freedom 
/// that are able to translate.</summary>
mat ElasticFrame2D::GetFreeGlobalStiffnessMatrix() {
	// Get the List of the Free Nodes.
	vector <bool> List = GetNodeFixityList();
	
	// Get the Number of Degrees of Freedom.
	int FreeDOF = GetNumberOfFreeDOF(List);
	
	// Set the Size of the Degrees of Freedom.
	mat K_Free(FreeDOF, FreeDOF);

	// Set the Global Stiffness Matrix.
	mat K_Global = GetGlobalStiffnessMatrix();

	// Initialize the Matrix.
	K_Free.fill(0.0);
	
	// List of the Index.
	vector <int> IndexList;

	// Get the Index where the List is True;
	for (size_t i = 0; i < List.size(); i++) {
		if (List[i] == true) 
		{
			// Save the Index Order
			IndexList.push_back(i);
		};
	};

	// Loop through the Node List to Add Each Cell
	for (int j = 0; j < FreeDOF; j++) {
		// Get the Global Index Location of the Row.
		int GlobalRow = IndexList[j];

		for (int k = 0; k < FreeDOF; k++) {
			// Get the Global Index Location of the Column.
			int GlobalCol = IndexList[k];
			
			// Store the Cell inside the Free DOF Stiffness.
			K_Free(j, k) = K_Global(GlobalRow, GlobalCol);
		};
	};
	
	// Return the Global Stiffness Matrix with Free Degrees of Freedom.
	return K_Free;
};

/// <summary></summary>
mat ElasticFrame2D::GetPermutatedGlobalStiffnessMatrix()
{
	// Find the Matrix.
	mat K = GetGlobalStiffnessMatrix();
	mat P = GetPermutationMatrix();
	
	mat Kp = P.t() * K * P;
	
	// Return the Permutated Global Stiffness Matrix.
	return Kp;
};

///<summary>Return the Permutation Matrix.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
mat ElasticFrame2D::GetPermutationMatrix()
{
	// Number of Elements
	size_t TotalDOF = this -> GetDofIndex();
	size_t NumOfBeam = MyVectorBeams.size();

	// Declare the Permutation Matrix.
	mat P(TotalDOF, TotalDOF);
	
	// Initialize the Permutation Matrix.
	P.fill(0.0);
	
	// Set the Fixity List to determine which node is 
	vector <int> FixityList = this -> GetNodeFixityIndexes();
	
	// Set the Size of the First Indexes List.
	size_t FreeIndex = 0;
	size_t FixityIndex = TotalDOF - FixityList.size();
	
	// Counter that is used to locate the Fixity Index;
	size_t FixityCounter = 0;

	// Add an negative value at the end of the Fixity List so index does not go out of bounds.
	FixityList.push_back(-1);

	// Fill the Permutation Matix.
	for (size_t i = 0; i < TotalDOF; i++)
	{
		if (i == FixityList[FixityCounter])
		{
			P(i, FixityIndex) = 1.0;

			// Increment the Index and Counter.
			FixityIndex++;
			FixityCounter++;
		}
		else
		{
			P(i, FreeIndex) = 1.0;

			// Increment the Index.
			FreeIndex++;
		}		
	};
	
	return P;
};
/// <summary>Return the full Global Stiffness Matrix bases off the order of the Nodes.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
mat ElasticFrame2D::GetGlobalStiffnessMatrix()
{
	// Find the total number of Degrees of Freedom.
	size_t TotalDOF = this -> GetDofIndex();

	// Set the Number of Beams in the frame.
	size_t NumOfBeam = MyVectorBeams.size();

	// Set the Size of all the Degrees of Freedom.
	mat K_global(TotalDOF, TotalDOF);
	
	// Initialize the Global Stiffness Matrix.
	K_global.fill(0.0);

	// Fill the Global Stiffness Matrix from the Beams.
	for (size_t i = 0; i < NumOfBeam; i++)
	{
		// Return the Beam Stiffness Matrix in Global Coordinates.
		mat k_local = MyVectorBeams[i].GetGlobalStiffnessMatrix();
		
		vector <int> Indexes = MyVectorBeams[i].GetDOFIndexNodes();

		// Store the Matrix.
		for (size_t j = 0; j < DOF * 2; j++)
		{
			// Step the Rows.
			int Rows = Indexes[j];

			for (size_t k = 0; k < DOF * 2; k++)
			{
				// Step the Columns.
				int Columns = Indexes[k];

				// Store the Columns.
				K_global(Rows, Columns) += k_local(j, k);
			};
		};
	};

	return K_global;
};

///<summary>Return the global geometric stiffness matrix using a set of global joint vector.</summary>
///<param name="GlobalJointVector"></param>
///<returns>description</returns>
///<remarks>description</remarks>
mat ElasticFrame2D::GetGlobalGeometricStiffnessMatrix(vector <mat> LocalBeamEndForces)
{
	// Find the total number of Degrees of Freedom.
	size_t TotalDOF = this -> GetDofIndex();

	// Set the Number of Beams in the frame.
	size_t NumOfBeam = MyVectorBeams.size();

	// Set the Size of all the Degrees of Freedom.
	mat K_global(TotalDOF, TotalDOF);

	// Clean the stiffness global.
	K_global.fill(0.0);

	// Fill the Global Stiffness Matrix from the Beams.
	for (size_t i = 0; i < NumOfBeam; i++)
	{
		// Get the local beam end forces.
		mat F = LocalBeamEndForces[i];

		// Return the Beam Stiffness Matrix in Global Coordinates.
		mat k_local = MyVectorBeams[i].GetGlobalGeometricStiffnessMatrix(F);
		
		// Return the Beam Indices.
		vector <int> Indexes = MyVectorBeams[i].GetDOFIndexNodes();

		// Store the Matrix.
		for (size_t j = 0; j < DOF * 2; j++)
		{
			// Step the Rows.
			int Rows = Indexes[j];

			for (size_t k = 0; k < DOF * 2; k++)
			{
				// Step the Columns.
				int Columns = Indexes[k];

				// Store the Columns.
				K_global(Rows, Columns) += k_local(j, k);
			};
		};
	};

	return K_global;
};

/// <summary></summary>
mat ElasticFrame2D::GetFreeForceVector()
{
	mat F = GetGlobalMemberForceVector();
	
	// Get the List of the Free Nodes.
	vector <bool> List = GetNodeFixityList();
	
	// Get the Number of Degrees of Freedom.
	int FreeDOF = GetNumberOfFreeDOF(List);
	
	// Set the Size of the Degrees of Freedom.
	mat F_Free(FreeDOF, 1);
	
	// List of the Index.
	vector <int> IndexList;

	// Get the Index where the List is True;
	for (size_t i = 0; i < List.size(); i++) {
		if (List[i] == true) 
		{
			// Save the Index Order
			IndexList.push_back(i);
		};
	};
	
	// Loop through the Node List to Add Each Cell
	for (int j = 0; j < FreeDOF; j++) {
		// Get the Global Index Location of the Row.
		int GlobalRow = IndexList[j];
		
		// The free Degree-of-Freedom.
		F_Free(j, 0) = F(GlobalRow, 0);
	};

	return F_Free;
};

///<summary>Return the global nodal force vector for the frame.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
mat ElasticFrame2D::GetGlobalJointForceVector()
{
	// Number of Elements
	size_t TotalDOF = this -> GetDofIndex();
	size_t NumOfNodeLoads = MyVectorNodeLoads.size();

	// Define the Force Vector.
	mat ForceVector(TotalDOF, 1);
	
	// Initialize the Force Vector.
	ForceVector.fill(0.0);

	// Fill the Global Stiffness Matrix from the Beams.
	for (size_t i = 0; i < NumOfNodeLoads; i++)
	{
		// Get the Node ID.
		int NodeID = MyVectorNodeLoads[i].GetNodeID();
		
		// Get the Node Index inside Vector.
		int Index = this -> GetNodeIndex(NodeID);

		// Get the Nodes Degree of Freedom.
		vector <int> DOFIndex =  MyVectorNodes[Index].GetNodeDOFIndexes();
		
		// Set the X Load.
		if (DOFIndex[0] != -1)
		{
			ForceVector(DOFIndex[0], 0) += MyVectorNodeLoads[i].GetGlobalXForce();
		}

		// Set the Y Load.
		if (DOFIndex[1] != -1)
		{
			ForceVector(DOFIndex[1], 0) += MyVectorNodeLoads[i].GetGlobalYForce();
		}
		
		// Set the Z Moment.
		if (DOFIndex[2] != -1)
		{
			ForceVector(DOFIndex[2], 0) += MyVectorNodeLoads[i].GetGlobalMoment();
		};
	};

	return ForceVector;
};


/// <summary>Find the Node Index inside the vector given the Node ID.</summary>
int ElasticFrame2D::GetNodeIndex(int NodeID)
{
	// TODO Need to Improve the search algorithm.
	int Index = -1;
	
	// Loop through the Vector to retreive the correct Nodes.
	for (size_t i = 0; i < MyVectorNodes.size(); i++)
	{
		if (NodeID == MyVectorNodes[i].GetID())
		{
			Index = i;
		};
	};

	return Index;
};

///<summary>Returns the Beam Index in the Frame.  If the Beam does not exist, it returns -1.</summary>
///<param name="BeamID"></param>
///<returns>description</returns>
///<remarks>description</remarks>
int ElasticFrame2D::GetBeamIndex(int BeamID)
{
	// TODO Need to improve the search algorithm.
	int Index = -1;

	for (size_t i = 0; i < MyVectorBeams.size(); i++)
	{
		if (BeamID == MyVectorBeams[i].GetID())
		{
			Index = i;
		}
	};

	return Index;
};

///<summary>Return the Section Index inside the vector given the Material ID.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
int ElasticFrame2D::GetSectionIndex(int SectionID)
{
	// TODO Need to improve the search algorithm.
	int Index = -1;

	for (size_t i = 0; i < MyVectorSections.size(); i++)
	{
		if (SectionID == MyVectorSections[i].GetID())
		{
			Index = i;
		}
	};

	return Index;
};

///<summary>Return the Member Force Vector.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
mat ElasticFrame2D::GetGlobalMemberForceVector()
{
	// Number of Elements
	size_t TotalDOF = this -> GetDofIndex();
	size_t NumOfBeam = MyVectorBeams.size();
	
	// Define the Force Vector.
	mat ForceVector(TotalDOF, 1);
	
	// Initialize the Force Vector.
	ForceVector.fill(0.0);
	
	// Loop through the Vector.
	for (size_t i = 0; i < MyVectorBeamLoads.size(); i++)
	{
		// Get the Beam ID.
		int BeamID = MyVectorBeamLoads[i] -> GetBeamID();
		
		// Get the Beam Index inside Vectr.
		int Index = this -> GetBeamIndex(BeamID);
		
		// Get the Beam Degree of Freedom.
		vector <int> Indexes = MyVectorBeams[Index].GetDOFIndexNodes();

		// Return the Globa Load Vector.
		mat LoadVector = MyVectorBeamLoads[i] -> GetGlobalLoadVector();
		
		// Store the Load Vector.
		for (size_t j = 0; j < 2 * DOF; j++)
		{
			// Store the Load inside the vector.
			ForceVector(Indexes[j], 0) += LoadVector(j, 0);
		};
	};

	return ForceVector;
};
/// <summary> Seach the list and return the index inside the parameter. </summary>
int ElasticFrame2D::GetIndexFromList(vector<int> List, int Item) {
	// Index to return.  Returning -1 means index not found.
	int Index = -1;
	
	size_t i = 0;
	bool PushExit = false;

	// Search through counter until finish or early exit counter when Index is found.
	while (i < List.size() && !PushExit) {
		// Assumption is that the 
		if (Item == List[i])
		{
			Index =	List[i];
			PushExit = true;
		};
		
		// Increment.
		i++;
	};

	return Index;
};

/// <summary>Return the Global Dispalcement vector.</summary>
mat ElasticFrame2D::GetGlobalDisplacement()
{
	// Return Displacement.
	return MyNodesDisplacements;
};

/// <summary> </summary>
size_t ElasticFrame2D::GetNumberOfDOF()
{
	size_t TotalDOF = this -> GetDofIndex();

	// Return the List of Nodal Index for use in Compilation.
	vector <int> FixityList = this -> GetNodeFixityIndexes();
	
	// Get the Number of Degrees of Freedom.
	size_t NumberOfDOF = TotalDOF - FixityList.size();
	
	return NumberOfDOF;
};

/// <summary>Return the Sorted Index List.</summary>
vector <int> ElasticFrame2D::GetNodeFixityIndexes()
{
	vector <int> IndexesList;

	// Store all the fixity inside the list.
	for (size_t i = 0; i < MyVectorNodes.size(); i++)
	{
		// Return the Nodal Index List.
		vector <int> NodeList = MyVectorNodes[i].GetNodeRestrainedIndexes();

		// Store the List.
		for (size_t j = 0; j < NodeList.size(); j++)
		{
			IndexesList.push_back(NodeList[j]);
		};
	};

	// Sort the List in Ascending Order.
	sort(IndexesList.begin(), IndexesList.end());

	// Return the Indexes List.
	return IndexesList;
};

/// <summary></summary>
vector <int> ElasticFrame2D::GetReleaseIndexes()
{
	vector <int> IndexesList;

	// Store all the fixity inside the list.
	for (size_t i = 0; i < MyVectorBeams.size(); i++)
	{
		// Return the Indexes List.
		vector <int> ReleaseList = MyVectorBeams[i].GetReleaseList();
		
		// Add to the Indexes List.
		for (size_t j = 0; j < ReleaseList.size(); j++)
		{
			IndexesList.push_back(ReleaseList[j]);
		};
	};
	
	// Return the Indexes List.
	return IndexesList;
};

/// <summary>Return the Amount of DOF for the whole system, including hinges.</summary>
size_t ElasticFrame2D::GetDofIndex()
{
	int DofIndex = 0;

	// Loop to access the DOF.
	for (size_t i = 0; i < MyVectorNodes.size(); i++)
	{
		//Increment the Degrees of Freedom.
		DofIndex += MyVectorNodes[i].GetNumOfDofTranslationX();
		DofIndex += MyVectorNodes[i].GetNumOfDofTranslationY();
		DofIndex += MyVectorNodes[i].GetNumOfDofRotationZ();
	}
	
	return DofIndex;
};

/// <summary></summary>
void ElasticFrame2D::SetDOFIndex()
{
	// Increment the first Index.
	int Index = 0;

	for (size_t i = 0; i < MyVectorNodes.size(); i++)
	{
		// Set the Nodal List.
		Index = MyVectorNodes[i].SetDOFIndex(Index);
	};
};

vector <mat> ElasticFrame2D::GetNodesGlobalDisplacement()
{
	vector <mat> NodeDisplacements;

	for (size_t i = 0; i < MyVectorNodes.size(); i++)
	{
		NodeDisplacements.push_back(this -> GetNodeGlobalDisplacement(i));
	};

	return NodeDisplacements;
};

///<summary></summary>
///<param name="ID">ID of the global displacements.</param>
///<returns>description</returns>
///<remarks>description</remarks>
mat ElasticFrame2D::GetNodeGlobalDisplacement(int ID)
{
	// Declare the Displacement Vector.
	mat Displacement = mat(3, 1);

	vector <int> IndexX = MyVectorNodes[ID].GetDOFIndexX();
	vector <int> IndexY = MyVectorNodes[ID].GetDOFIndexY();
	vector <int> IndexZ = MyVectorNodes[ID].GetDOFIndexZ();

	// (0, 0) = X-Directional Coordinate.
	Displacement(0, 0) = MyVectorNodes[ID].GetX();

	// Add the Nodal Displacements.
	if (IndexX[0] != -1)
	{
		Displacement(0, 0) += MyNodesDisplacements(IndexX[0], 0);
	};
	
	// (1, 0) = Y-Directional Coordinate.
	Displacement(1, 0) = MyVectorNodes[ID].GetY();

	if (IndexY[0] != -1)
	{
		// Add the Nodal Displacements.
		Displacement(1, 0) += MyNodesDisplacements(IndexY[0], 0);
	};
	
	// (2, 0) = Z-Moment
	Displacement(2, 0) = MyVectorNodes[ID].GetRotationZ();

	if (IndexZ[0] != -1)
	{
		// Add the Nodal Displacements.
		Displacement(2, 0) += MyNodesDisplacements(IndexZ[0], 0);
	};
	
	return Displacement;
};

vector <mat> ElasticFrame2D::GetBeamsGlobalDisplacement()
{
	vector <mat> BeamsDisplacements;

	return BeamsDisplacements;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
mat ElasticFrame2D::GetBeamLocalDisplacement(int ID)
{
	//TODO: NOT COMPLETE.
	mat BeamDisplacements(6, 1);

	mat Displacement = this -> GetBeamGlobalDisplacement(ID);

	return BeamDisplacements;
};

/// <summary>Return the Beam Virtual Dispalcement.</summary>
/// <param name="BeamIndex">ID of the Beam.</param>
mat ElasticFrame2D::GetBeamGlobalDisplacement(int BeamIndex)
{
	// Get the Degrees of Freedom Indexes.
	vector <int> Indexes = MyVectorBeams[BeamIndex].GetDOFIndexNodes();

	// Declare the Displacement Vector.
	mat DOF_Displacement = mat(6, 1);

	// Find the Displacement.
	for (size_t i = 0; i < DOF * 2; i++)
	{
		DOF_Displacement(i, 0) = MyNodesDisplacements(Indexes[i], 0);
	};

	return DOF_Displacement;
};

///<summary></summary>
///<remarks>description</remarks>
void ElasticFrame2D::Print() {
	if (ShowNodes)
	{
		// Print the Nodes
		for (size_t i = 0; i < MyVectorNodes.size() ; i++)
		{
			MyVectorNodes[i].Print();
		};
	}
	
	if (ShowBeams)
	{
		// Print the Beam.
		for (size_t j = 0; j < MyVectorBeams.size(); j++)
		{
			MyVectorBeams[j].Print();
		};
	}

	if (ShowNodeLoads)
	{
		// Print the Node Loads.
		for (size_t k = 0; k < MyVectorNodeLoads.size(); k++)
		{
			MyVectorNodeLoads[k].Print();
		};
	}

	if (ShowBeamLoads)
	{
		// Print the Beam Loads.
		for (size_t l = 0; l < MyVectorBeamLoads.size(); l++)
		{
			MyVectorBeamLoads[l] -> Print();
		};
	}

	MyTangentStiffnessMatrix.print("Tangent Stiffness Matrix");
	MyCombineForceVector.print("Combine Force Vector");
	MyPermutationMatrix.print("Permutation Matrix");
	MyNodesDisplacements.print("Nodes Displacements");
};

///<summary>Return the Number of Beams.</summary>
///<param name="name"></param>
///<returns>Data Type: size_t</returns>
///<remarks>description</remarks>
size_t ElasticFrame2D::GetNumberOfBeams()
{
	return MyVectorBeams.size();
};

///<summary>Return the Number of Nodes.</summary>
///<param name="name"></param>
///<returns>Data Type: size_t</returns>
///<remarks></remarks>
size_t ElasticFrame2D::GetNumberOfNodes()
{
	return MyVectorNodes.size();
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::SetBeamReleases(int BeamID, bool ReleaseNodeA, bool ReleaseNodeB)
{
	/// TODO: Check if Beam ID Exist.
	int Index = this -> GetBeamIndex(BeamID);

	// Set the Releases.
	MyVectorBeams[Index].SetNodeAReleaseMoment(ReleaseNodeA);
	MyVectorBeams[Index].SetNodeBReleaseMoment(ReleaseNodeB);
};

///<summary>The subroutine checks the moment capacity at each of the beam element
/// with respect to the applied moment.  The scale factor is modified if necessary.</summary>
///<param name="IncrementForce">The beam end forces at the iteration before the load is applied.</param>
///<param name="DeltaForce">The delta beam end forces accounting for the load.</param>
///<param name="Tolerance">The tolerance of the ratio.</param>
///<param name="ScaleFactor">The load scaling parameter.</param>
///<param name="HingingIndex">Hinging Index returning the scenario.</param>
///<remarks>description</remarks>
void ElasticFrame2D::CheckMomentCapacity(vector <mat> IncrementForces,
										 vector <mat> DeltaForces,
										 double Tolerance,
										 double &ScaleFactor,
										 int &HingingIndex)
{
	/*
	   /|\
		|										*
		|									*
		|							   *
		|                         *
		|                      *
		|                  *
	M_z |---------------*
		|         Mp *
	M_i |----------*
		|         *
		|       *
		|      *
		|    *
		|   *
		| *
		|*____________________________________________\
													  /
	*/

	//	Hinge Ratio = (Mz - Mp) / Mp
	//	Factor = (Mp - Mz) / (Mz - Mi)
	
	size_t NumOfBeams = MyVectorBeams.size();

	// Add the DeltaForces with the Increment Forces;
	vector <mat> TotalForces;
	
	// Loop to add the load to find the total forces.
	for (size_t h = 0; h < NumOfBeams; h++)
	{
		TotalForces.push_back(DeltaForces[h] + IncrementForces[h]);
	};
	
	// Set the Ratio Tables
	// 0 = Noda A Hinge Ratio
	// 1 = Node A Factor
	// 2 = Node B Hinge Ratio
	// 3 = Node B Factor
	mat RatioTables(NumOfBeams, 4);

	// Start the Loop off with tolerance not exceeded.
	bool ExceedTolerance = false;

	// Loop through the Beam and check if the nodal end forces exceeds the Moment Capacity.
	for (size_t i = 0; i < NumOfBeams; i++)
	{
		//	Hinge Ratio = (Mz - Mp) / Mp
		//	Factor = (Mp - Mi) / (Mz - Mi)
			
		// Node A Forces.
		double Mz_A = abs(TotalForces[i](2, 0));														// Moment at i + 1
		double Mi_A = abs(IncrementForces[i](2,0));														// Moment at i
		double Mp_A = abs(MyVectorBeams[i].GetPlasticMomentCapacityNodeA(TotalForces[i](2, 0)));		// Plastic Moment

		double Mz_B = abs(TotalForces[i](5, 0));														// Moment at i + 1
		double Mi_B = abs(IncrementForces[i](5, 0));													// Moment at i
		double Mp_B = abs(MyVectorBeams[i].GetPlasticMomentCapacityNodeB(TotalForces[i](5, 0)));		// Plastic Moment

		// =================== Node A =======================
		RatioTables(i, 0) = (Mz_A - Mp_A) / Mp_A;					// Hinge Ratio
		RatioTables(i, 1) = abs((Mp_A - Mi_A) / (Mz_A - Mi_A));		// Factor

		// If the hinge is not released, then we check the hinge ratio with the specified tolerance.
		if (!MyVectorBeams[i].IsRotationZReleaseNodeA())
		{
			if (RatioTables(i, 0) > Tolerance) { ExceedTolerance = true; };
		};

		// =================== Node B =======================
		RatioTables(i, 2) = (Mz_B - Mp_B) / Mp_B;					// Hinge Ratio
		RatioTables(i, 3) = abs((Mp_B - Mi_B) / (Mz_B - Mi_B));		// Factor

		// If the hinge is not released, then we check the hinge ratio with the specified tolerance.
		if (!MyVectorBeams[i].IsRotationZReleaseNodeB())
		{
			if (RatioTables(i, 2) > Tolerance) { ExceedTolerance = true; };
		};
	};

	// Display Ratio Tables in Console.
	if (ShowRatioTables) { RatioTables.print("Hinge Ratio A - Node A Factor - Hinge Ratio B - Node B Factor"); }

	// If the tolerance are exceeded, we will modify the load factor and run another analysis. 
	if (ExceedTolerance)
	{
		// Save the smallest load factor and use that as the basis.
		double ModifyLoadFactor = 1.0;

		for (size_t k = 0; k < NumOfBeams; k++)
		{
			// Check if Node A exceeds the Tolerance.
			if (RatioTables(k, 0) > Tolerance)
			{
				// Store the Lowest Number.
				if (RatioTables(k, 1) < ModifyLoadFactor)
				{
					ModifyLoadFactor = RatioTables(k, 1);
				};
			};

			// Check if Node B exceeds the Tolerance.
			if (RatioTables(k, 2) > Tolerance)
			{
				// Store the Lowest Number.
				if (RatioTables(k, 3) < ModifyLoadFactor)
				{
					ModifyLoadFactor = RatioTables(k, 3);
				};
			};
		};

		// Change the Scale Factor.
		ScaleFactor *= ModifyLoadFactor;

		// Change the Counter to over excceds.
		HingingIndex = ElasticFrame2D::HingingIndexOverExceeded;
	}
	else
	{
		// Start the index with not exceed.
		HingingIndex = ElasticFrame2D::HingingIndexNotExceeded;
		
		// Loop to find if the ratio is +/- between the tolerance.
		for (size_t l = 0; l < NumOfBeams; l++)
		{
			// Look at Node A.
			if (!MyVectorBeams[l].IsRotationZReleaseNodeA())
			{
				if (abs(RatioTables(l, 0)) < Tolerance)
				{
					// Release Node A.
					MyVectorBeams[l].SetNodeAReleaseMoment(true);

					// Change the Index.
					HingingIndex = ElasticFrame2D::HingingIndexConverged;
				};
			};
			
			// Look at Node B.
			if (!MyVectorBeams[l].IsRotationZReleaseNodeB())
			{
				if (abs(RatioTables(l, 2)) < Tolerance)
				{
					// Release Node B.
					MyVectorBeams[l].SetNodeBReleaseMoment(true);

					// Change the Index.
					HingingIndex = ElasticFrame2D::HingingIndexConverged;
				};
			};
		};
	};
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::CheckForHinges(vector <mat> TotalForce, double Tolerance)
{
	// Loop to find any hinging
	for (size_t i = 0; i < MyVectorBeams.size(); i++)
	{

		// Node A Applied Moment.
		double NodeAMoment = TotalForce[i](2, 0);

		// Node A Plastic Moment Capacity.
		double NodeAPlasticMoment = MyVectorBeams[i].GetPlasticMomentCapacityNodeA(NodeAMoment);

		// Set the Ratio
		double NodeARatio = (std::abs(NodeAPlasticMoment) - std::abs(NodeAMoment)) / std::abs(NodeAPlasticMoment);

		if (NodeARatio < Tolerance)
		{
			// Release Node A.
			MyVectorBeams[i].SetNodeAReleaseMoment(true);
		}
		
		// Node B Applied Moment.
		double NodeBMoment = TotalForce[i](5, 0);

		// Node B Plastic Moment Capacity.
		double NodeBPlasticMoment = MyVectorBeams[i].GetPlasticMomentCapacityNodeA(NodeBMoment);

		// Set the Ratio
		double NodeBRatio = (std::abs(NodeBPlasticMoment) - std::abs(NodeBMoment)) / std::abs(NodeBPlasticMoment);

		if (NodeBRatio < Tolerance)
		{
			// Release Node B.
			MyVectorBeams[i].SetNodeBReleaseMoment(true);
		}
	};
};

/*========================================================================================
							BEAM EXTERNAL END FORCES
========================================================================================*/
///<summary>Return the Beam End Force Vector.</summary>
///<returns>Return a vector of 6 cell matrix.</returns>
///<remarks>description</remarks>
vector <mat> ElasticFrame2D::GetExternalBeamEndForces()
{
	vector <mat> LocalForces;

	// Loop through the Beam and Check if it exceeds the Moment Capacity.
	for (size_t i = 0; i < MyVectorBeams.size(); i++)
	{
		// Retreive the Local Force.
		mat LocalForce = this -> GetExternalBeamLocalForces(MyVectorBeams[i].GetID());

		// Add Local Force of Beam 'i' to vector.
		LocalForces.push_back(LocalForce);
	};

	// Return the Local Force by value.
	return LocalForces;
};


///<summary>Return the beam nodal forces.</summary>
///<param name="BeamID">Beam Identifier.</param>
///<returns>description</returns>
///<remarks>description</remarks>
mat ElasticFrame2D::GetExternalBeamLocalForces(int BeamID)
{
	// Get the Beam Index.
	int BeamIndex = this -> GetBeamIndex(BeamID);

	// Get the Beam Member Local Forces.
	mat LocalBeamMemberForces = MyVectorBeams[BeamIndex].GetBeamLocalForceVector();

	// Get the Nodal Local Forces.
	mat LocalNodeMemberForces = MyVectorBeams[BeamIndex].GetNodeLocalForceVector();

	// Sum of the beam and node force.
	mat LocalForce = LocalBeamMemberForces + LocalNodeMemberForces;

	// Return Local Force.
	return LocalForce;
};


/*========================================================================================
							BEAM INTERNAL END FORCES
========================================================================================*/

///<summary>Return the Beam End Force Vector.</summary>
///<returns>Return a vector of 6 cell matrix.</returns>
///<remarks>description</remarks>
vector <mat> ElasticFrame2D::GetInternalBeamEndForces()
{
	vector <mat> LocalForces;

	// Loop through the Beam and Check if it exceeds the Moment Capacity.
	for (size_t i = 0; i < MyVectorBeams.size(); i++)
	{
		// Retreive the Local Force.
		mat LocalForce = this -> GetInternalBeamLocalForces(MyVectorBeams[i].GetID());

		// Add Local Force of Beam 'i' to vector.
		LocalForces.push_back(LocalForce);
	};

	// Return the Local Force by value.
	return LocalForces;
};


///<summary>Return the Global Internal Force Vector.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
mat ElasticFrame2D::GetGlobalInternalForceVector()
{
	// Number of Elements
	size_t TotalDOF = this -> GetDofIndex();
	size_t NumOfBeam = MyVectorBeams.size();
	
	// Define the Force Vector.
	mat ForceVector(TotalDOF, 1);
	
	// Initialize the Force Vector.
	ForceVector.fill(0.0);

	// Loop through the Vector.
	for (size_t BeamIndex = 0; BeamIndex < MyVectorBeams.size(); BeamIndex++)
	{	
		// Get the Beam Degree of Freedom.
		vector <int> Indexes = MyVectorBeams[BeamIndex].GetDOFIndexNodes();

		// Return the Transformation Matrix.
		mat Transformation = MyVectorBeams[BeamIndex].GetTransformationMatrix();

		// Return the Internal Forces in the local coordinates.
		//mat LocalForce = this -> GetInternalBeamLocalForces(MyVectorBeams[BeamIndex].GetID());
		mat LocalForce = this -> GetInternalForces(MyVectorBeams[BeamIndex].GetID());

		// Return the Globa Load Vector.
		mat LoadVector = Transformation.t() * LocalForce;
		
		// Store the Load Vector.
		for (size_t j = 0; j < 2 * DOF; j++)
		{
			// Store the Load inside the vector.
			ForceVector(Indexes[j], 0) += LoadVector(j, 0);
		};
	};

	return ForceVector;
};

///<summary>Return the Beam Internal Forces not accounting for the externally applied load.</summary>
///<param name="BeamID">Beam Identifier.</param>
///<returns>description</returns>
///<remarks>description</remarks>
mat ElasticFrame2D::GetInternalForces(int BeamID)
{
	// Get the Beam Index.
	int BeamIndex = this -> GetBeamIndex(BeamID);

	// Return the GlobalDisplacement for the beam.
	mat GlobalDisplacement = this -> GetBeamGlobalDisplacement(BeamIndex);
	
	// Return the Transformation Matrix.
	mat Transformation = MyVectorBeams[BeamIndex].GetTransformationMatrix();

	// Get Local Displacement.
	mat LocalDisplacement = Transformation * GlobalDisplacement;

	// Get Local Stiffness.
	mat LocalStiffness = MyVectorBeams[BeamIndex].GetLocalStiffnessMatrix();

	// Check for the Local Stiffness.
	if (IncludeGeometricNonLinearity)
	{
		// Find the Axial Load.
		double AxialLoad = MyLocalBeamEndForces[BeamIndex](3, 0);

		// Find the Geometric Stiffness Matrix.
		mat LocalGeometricStiffness = MyVectorBeams[BeamIndex].GetLocalGeometricStiffnessMatrix(AxialLoad);

		// Add to create the tangent stiffness matrix.
		LocalStiffness += LocalGeometricStiffness;

		// Printing Results.
		if (false)
		{
			LocalGeometricStiffness.print("Geometric Stiffness Matrix");
			LocalStiffness.print("Tangent Stiffness");
		}
	};

	// Return the Internal Force of the system.
	mat InternalForce = LocalStiffness * LocalDisplacement;

	// Return Local Force.
	return InternalForce;
};

///<summary>Return the Beam Nodal Forces forces at the bend ends.  This method accounts for beam end forces.</summary>
///<param name="BeamID">Beam Identifier.</param>
///<returns>description</returns>
///<remarks>description</remarks>
mat ElasticFrame2D::GetInternalBeamLocalForces(int BeamID)
{
	// Get the Beam Index.
	int BeamIndex = this -> GetBeamIndex(BeamID);

	// Return the GlobalDisplacement for the beam.
	mat GlobalDisplacement = this -> GetBeamGlobalDisplacement(BeamIndex);
	
	// Return the Transformation Matrix.
	mat Transformation = MyVectorBeams[BeamIndex].GetTransformationMatrix();

	// Get Local Displacement.
	mat LocalDisplacement = Transformation * GlobalDisplacement;

	// Get Local Stiffness.
	mat LocalStiffness = MyVectorBeams[BeamIndex].GetLocalStiffnessMatrix();

	// Check for the Local Stiffness.
	if (IncludeGeometricNonLinearity)
	{
		// Find the Axial Load.
		double AxialLoad = MyLocalBeamEndForces[BeamIndex](3, 0);

		// Find the Geometric Stiffness Matrix.
		mat LocalGeometricStiffness = MyVectorBeams[BeamIndex].GetLocalGeometricStiffnessMatrix(AxialLoad);

		// Add to create the tangent stiffness matrix.
		LocalStiffness += LocalGeometricStiffness;

		// Printing Results.
		if (false)
		{
			LocalGeometricStiffness.print("Geometric Stiffness Matrix");
			LocalStiffness.print("Tangent Stiffness");
		}
	};

	mat BeamEndForces = MyVectorBeams[BeamIndex].GetBeamLocalForceVector();

	// Return the Internal Force of the system.
	mat InternalForce = LocalStiffness * LocalDisplacement;

	// Return Local Force.
	return InternalForce - BeamEndForces;
};

///<summary>Return the Nodal List of the newly deform Nodes.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <Node2D> ElasticFrame2D::GetDeformNodes()
{
	vector <Node2D> DeformNodes = MyVectorNodes;

	// Loop through the new deform and increment it.
	for (size_t i = 0; i < DeformNodes.size(); i++)
	{
		// Get the Nodal Indices.
		vector <int> Indexes = DeformNodes[i].GetNodeDOFIndexes();
		
		// Change the X-Location.
		if (Indexes[0] != -1)
		{
			DeformNodes[i].IncrementTranslationX(MyNodesDisplacements(Indexes[0], 0));
		};

		// Change the Y-Location.
		if (Indexes[1] != -1)
		{
			DeformNodes[i].IncrementTranslationY(MyNodesDisplacements(Indexes[1], 0));
		};

		// Change the Z-Rotation.
		if (Indexes[2] != -1)
		{
			DeformNodes[i].IncrementRotationZ(MyNodesDisplacements(Indexes[2], 0));
		};
	};

	return DeformNodes;
};

///<summary>Change the nodes from the displacement vector give.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::SetDeformNodes(arma::mat Displacements)
{
	// Loop through the new deform and increment it.
	for (size_t i = 0; i < MyVectorNodes.size(); i++)
	{
		// Get the Nodal Indices.
		vector <int> Indexes = MyVectorNodes[i].GetNodeDOFIndexes();
		
		// Change the X-Location.
		if (Indexes[0] != -1)
		{
			MyVectorNodes[i].IncrementTranslationX(Displacements(Indexes[0], 0));
		};

		// Change the Y-Location.
		if (Indexes[1] != -1)
		{
			MyVectorNodes[i].IncrementTranslationY(Displacements(Indexes[1], 0));
		};

		// Change the Z-Rotation.
		if (Indexes[2] != -1)
		{
			MyVectorNodes[i].IncrementRotationZ(Displacements(Indexes[2], 0));
		};
	};
};

///<summary>Set the Nodal Displacements of the section.</summary>
///<param name="Displacements"></param>
///<remarks>description</remarks>
void ElasticFrame2D::SetNodesDisplacements(mat Displacements)
{
	MyNodesDisplacements = Displacements;
};

///<summary>Return the Nodal List of the newly deform Nodes.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <Node2D> ElasticFrame2D::GetNodeDeformation()
{
	vector <Node2D> DeformNodes;

	// Loop through the new deform and increment it.
	for (size_t i = 0; i < MyVectorNodes.size(); i++)
	{
		Node2D Node = Node2D();

		// Get the Nodal Indices.
		vector <int> Indexes = MyVectorNodes[i].GetNodeDOFIndexes();
		
		// Change the X-Location.
		if (Indexes[0] != -1)
		{
			Node.IncrementTranslationX(MyNodesDisplacements(Indexes[0], 0));
		};

		// Change the Y-Location.
		if (Indexes[1] != -1)
		{
			Node.IncrementTranslationY(MyNodesDisplacements(Indexes[1], 0));
		};

		// Change the Z-Rotation.
		if (Indexes[2] != -1)
		{
			Node.IncrementRotationZ(MyNodesDisplacements(Indexes[2], 0));
		};

		// Save Deform Nodes.
		DeformNodes.push_back(Node);
	};

	return DeformNodes;
};

///<summary></summary>
///<param name="Nodes">The new deform nodes it's latest deform shape.</param>
///<remarks>description</remarks>
void ElasticFrame2D::SetDeformNodes(vector<Node2D> Nodes)
{
	// Do a complete swap of the old nodes with the new deform nodes.
	MyVectorNodes = Nodes;
};

///<summary>Clear all the Loads on the Frame.</summary>
///<remarks>description</remarks>
void ElasticFrame2D::ClearLoads()
{
	// Clear All Loads.
	MyVectorNodeLoads.clear();

	MyVectorMomentLoads.clear();
	MyVectorPointLoads.clear();
	MyVectorPyramidLoads.clear();
	MyVectorTriangularLoads.clear();
	MyVectorUniformLoads.clear();

	MyVectorBeamLoads.clear();
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
Node2D ElasticFrame2D::GetNodeDisplacement(int ID)
{
	// Return the Node Index.
	int Index = this -> GetNodeIndex(ID);

	// Get the Node ID.
	Node2D Node = MyVectorNodes[Index];

	// Get the Nodal Indices.
	vector <int> Indexes = MyVectorNodes[Index].GetNodeDOFIndexes();
		
	// Change the X-Location.
	if (Indexes[0] != -1)
	{
		Node.IncrementTranslationX(MyNodesDisplacements(Indexes[0], 0));
	};

	// Change the Y-Location.
	if (Indexes[1] != -1)
	{
		Node.IncrementTranslationY(MyNodesDisplacements(Indexes[1], 0));
	};

	// Change the Z-Rotation.
	if (Indexes[2] != -1)
	{
		Node.IncrementRotationZ(MyNodesDisplacements(Indexes[2], 0));
	};

	// Return the Nodal Value.
	return Node;
};

/// <summary>Return the Member Force Vector.</summary>
mat ElasticFrame2D::GetGlobalMassVector()
{
	// Number of Elements
	size_t TotalDOF = this -> GetDofIndex();
	size_t NumOfBeam = MyVectorBeams.size();
	
	// Define the Mass Vector.
	mat MassVector(TotalDOF, 1);
	
	// Initialize the Mass Vector.
	MassVector.fill(0.0);
	
	// Loop through the Vector.
	for (size_t i = 0; i < MyVectorBeams.size(); i++)
	{				
		// Get the Beam Degree of Freedom.
		vector <int> Indexes = MyVectorBeams[i].GetDOFIndexNodes();

		// Return the Global Mass Vector.
		mat LocalVector = MyVectorBeams[i].GetMassVector();
		
		// Store the Mass Vector.
		for (size_t j = 0; j < 2 * DOF; j++)
		{
			// Store the Mass inside the vector.
			MassVector(Indexes[j], 0) += LocalVector(j, 0);
		};
	};

	return MassVector;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <int> ElasticFrame2D::GetListOfDOFIndexes(int DOF)
{
	vector <int> ListOfDOF;

	// Set the Number of Beams in the frame.
	size_t NumOfBeam = MyVectorBeams.size();

	// Fill the Global Stiffness Matrix from the Beams.
	for (size_t i = 0; i < NumOfBeam; i++)
	{
		// Return the Beam Index List.		
		vector <int> BeamIndexes = MyVectorBeams[i].GetDOFIndexNodes();

		int IndexNodeA = -1;
		int IndexNodeB = -1;

		// Determine which Index To Save.
		switch(DOF)
		{
			case ElasticFrame2D::TranslationX:
				// Store the X Index for Node A.
				ListOfDOF.push_back(BeamIndexes[0]); 

				// Store the Y Index for Node B.
				ListOfDOF.push_back(BeamIndexes[3]);
				break;
			case ElasticFrame2D::TranslationY:
				// Store the X Index for Node A.
				ListOfDOF.push_back(BeamIndexes[1]); 

				// Store the Y Index for Node B.
				ListOfDOF.push_back(BeamIndexes[4]);

				break;
			case ElasticFrame2D::RotationZ:
				// Store the X Index for Node A.
				ListOfDOF.push_back(BeamIndexes[2]); 

				// Store the Y Index for Node B.
				ListOfDOF.push_back(BeamIndexes[5]);

				break;
		};
	};

	vector <int> ReducedListOfDOF;

	// Add the first DOF>
	ReducedListOfDOF.push_back(ListOfDOF[0]);

	for (size_t j = 1; j < ListOfDOF.size(); j ++)
	{
		// Initialize.
		size_t k = 0;
		bool Duplicate = false;

		// Add the ListOfDOF.
		while ((k < ReducedListOfDOF.size()) & !Duplicate)
		{
			// Check to see if the this is a duplicate.
			if (ListOfDOF[j] == ReducedListOfDOF[k]) { Duplicate = true; };

			k++;
		};

		// Add the new DOF to the reduced list.
		if (!Duplicate) { ReducedListOfDOF.push_back(ListOfDOF[j]); };
	};

	return ReducedListOfDOF;
};

///<summary>Return the Displacement Vector for the Frame.</summary>
///<param name="DOF">Degree-of-freedom (X = 1, Y = 2, Z = 3)</param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <double> ElasticFrame2D::GetDisplacementVector(int DOF)
{
	// Initialize the Displacement Vector.
	vector <double> DisplacementVector;

	// Find the list of degrees-of-freedom.
	vector <int> ListOfDOF = this -> GetListOfDOFIndexes(DOF);

	// Store the Nodal Displacement
	for (size_t i = 0; i < ListOfDOF.size(); i++)
	{
		DisplacementVector.push_back(MyNodesDisplacements(ListOfDOF[i], 0));
	};

	return DisplacementVector;
};

///<summary>Return the Load Vector for the Frame.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <double> ElasticFrame2D::GetBeamLocalLoadVector(int DOF)
{

	vector <double> LoadVector;

	// Find the total number of Degrees of Freedom.
	size_t TotalDOF = this -> GetDofIndex();

	// Set the Number of Beams in the frame.
	size_t NumOfBeams = MyVectorBeams.size();

	for (size_t i = 0; i < NumOfBeams; i++)
	{
		// Return the Beam Force Vector.
		mat BeamVector = MyVectorBeams[i].GetBeamLocalForceVector();

		// Determine which Index To Save.
		switch(DOF)
		{
			case ElasticFrame2D::TranslationX:
				// Store the Nodal Displacement for Node A.
				LoadVector.push_back(BeamVector(0, 0));

				// Store the Nodal Displacement for Node B.
				LoadVector.push_back(BeamVector(3, 0));

				break;
			case ElasticFrame2D::TranslationY:
				// Store the Nodal Displacement for Node A.
				LoadVector.push_back(BeamVector(1, 0));

				// Store the Nodal Displacement for Node B.
				LoadVector.push_back(BeamVector(4, 0));

				break;
			case ElasticFrame2D::RotationZ:
				// Store the Nodal Displacement for Node A.
				LoadVector.push_back(BeamVector(2, 0));

				// Store the Nodal Displacement for Node B.
				LoadVector.push_back(BeamVector(5, 0));

				break;
		};
	};

	return LoadVector;
};

///<summary>Return the Load Vector for the Frame.</summary>
///<param name="DOF"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <double> ElasticFrame2D::GetBeamGlobalLoadVector(int DOF)
{
	// Initialize the Displacement Vector.
	vector <double> Loads;

	// Find the list of degrees-of-freedom.
	vector <int> ListOfDOF = this -> GetListOfDOFIndexes(DOF);

	// Store the Nodal Displacement
	for (size_t i = 0; i < ListOfDOF.size(); i++)
	{
		Loads.push_back(MyMemberForceVector(ListOfDOF[i], 0));
	};

	return Loads;
};

///<summary>Return the Nodal Loads in vector form.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <double> ElasticFrame2D::GetNodeLoadVector(int DOF)
{
	// Initialize the Displacement Vector.
	vector <double> Loads;

	// Find the list of degrees-of-freedom.
	vector <int> ListOfDOF = this -> GetListOfDOFIndexes(DOF);

	// Store the Nodal Displacement
	for (size_t i = 0; i < ListOfDOF.size(); i++)
	{
		Loads.push_back(MyJointForceVector(ListOfDOF[i], 0));
	};

	return Loads;
};

///<summary>Return the Shape Vector for the Translational X, Translational Y, and Rotational Z Degree of Freedoms.</summary>
///<param="DOF">Translational X = 1, Translational Y = 2, and Rotational Z = 3</param>
///<returns></returns>
///<remarks>The Shape Vector is taken with retrospect to the beam.  For every beam there is two end nodal displacement.  Therefore, there
/// will be twice as many beam end displacement in the shape vector as there beams.</remarks>
vector <double> ElasticFrame2D::GetShapeVector(int DOF)
{
	vector <double> ShapeVector;

	// Find the total number of Degrees of Freedom.
	//size_t TotalDOF = this -> GetDofIndex();

	// Set the Number of Beams in the frame.
	size_t NumOfBeam = MyVectorBeams.size();

	// Fill the Global Stiffness Matrix from the Beams.
	for (size_t i = 0; i < NumOfBeam; i++)
	{
		// Return the Beam Index List.		
		vector <int> BeamIndexes = MyVectorBeams[i].GetDOFIndexNodes();

		int IndexNodeA = -1;
		int IndexNodeB = -1;

		// Determine which Index To Save.
		switch(DOF)
		{
			case ElasticFrame2D::TranslationX:
				// Store the X Index for Node A.
				IndexNodeA = BeamIndexes[0]; 

				// Store the Y Index for Node B.
				IndexNodeB = BeamIndexes[3];
				break;
			case ElasticFrame2D::TranslationY:
				// Store the X Index for Node A.
				IndexNodeA = BeamIndexes[1]; 

				// Store the Y Index for Node B.
				IndexNodeB = BeamIndexes[4];

				break;
			case ElasticFrame2D::RotationZ:
				// Store the X Index for Node A.
				IndexNodeA = BeamIndexes[2]; 

				// Store the Y Index for Node B.
				IndexNodeB = BeamIndexes[5];

				break;
		};
		

		// Store the Nodal Displacement for Node A.
		ShapeVector.push_back(MyNodesDisplacements(IndexNodeA, 0));

		// Store the Nodal Displacement for Node B.
		ShapeVector.push_back(MyNodesDisplacements(IndexNodeB, 0));
	};

	// Initialize the Max Diplacement.
	double MaxDisplacement = 0.0;

	// Loop through the Shape Vector and find which magnitude is the largest.
	for (size_t j = 0; j < NumOfBeam * 2; j++)
	{
		// Compare the magnitude of the Shape Vector.
		if(abs(ShapeVector[j]) > MaxDisplacement)
		{
			// Replace Max Displacement value with the current ShapeVector.
			MaxDisplacement = abs(ShapeVector[j]);
		};
	};

	// Scale the Shape Vector.
	for (size_t k = 0; k < NumOfBeam * 2; k++)
	{
		ShapeVector[k] = ShapeVector[k] / MaxDisplacement;
	};

	return ShapeVector;
};

///<summary>Return the Mass Vector.</summary>
///<param name="DOF"></param>
///<returns>description</returns>
///<remarks>description</remarks>
vector <double> ElasticFrame2D::GetMassVector(int DOF)
{
	// Initialize the Mass Vector.
	vector <double> MassVector;

	// Find the list of degrees-of-freedom.
	vector <int> ListOfDOF = this -> GetListOfDOFIndexes(DOF);

	// Store the Nodal Displacement
	for (size_t i = 0; i < ListOfDOF.size(); i++)
	{
		MassVector.push_back(MyMassVector(ListOfDOF[i], 0));
	};
	
	// Return the Mass Vector.
	return MassVector;
};

///<summary></summary>
///<returns>description</returns>
///<remarks>description</remarks>
vector <Node2D> ElasticFrame2D::GetNodes()
{
	return MyVectorNodes;
};

///<summary></summary>
///<returns>description</returns>
///<remarks>description</remarks>
vector <Beam2D> ElasticFrame2D::GetBeams()
{
	return MyVectorBeams;
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
Node2D ElasticFrame2D::GetNode(int ID)
{
	int Index = this-> GetBeamIndex(ID);

	// Return the Nodes.
	return MyVectorNodes[Index];
};

///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
Beam2D ElasticFrame2D::GetBeam(int ID)
{
	int Index = this-> GetBeamIndex(ID);

	// Return the Beams.
	return MyVectorBeams[Index];
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
void ElasticFrame2D::SetName(string Name, int ID)
{
	MyName = Name;
	MyID = ID;
};

vector <NodeLoad2D> ElasticFrame2D::GetNodeLoads()
{
	return MyVectorNodeLoads;
};

vector <MomentLoad2D> ElasticFrame2D::GetMomentLoads()
{
	return MyVectorMomentLoads;
};

vector <PointLoad2D> ElasticFrame2D::GetPointLoads()
{
	return MyVectorPointLoads;
};

vector <PyramidLoad2D> ElasticFrame2D::GetPyramidLoads()
{
	return MyVectorPyramidLoads;
};

vector <TriangularLoad2D> ElasticFrame2D::GetTriangularLoads()
{
	return MyVectorTriangularLoads;
};

vector <UniformLoad2D> ElasticFrame2D::GetUniformLoads()
{
	return MyVectorUniformLoads;
};


///<summary></summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
ElasticFrame2D ElasticFrame2D::GetPlasticFrameGeometry()
{
	ElasticFrame2D Frame = ElasticFrame2D();

	// Store the Sections.
	for (size_t h = 0; h < MyVectorSections.size(); h++)
	{
		Frame.AddSection(MyVectorSections[h]);
	};

	// Store the Nodes. (Storing Pointers).
	for (size_t i = 0; i < MyVectorNodes.size(); i++)
	{
		Frame.AddNode(MyVectorNodes[i]);
	};
	
	// Store the Beams and re-reference the Nodes and Section.
	for (size_t j = 0; j < MyVectorBeams.size(); j++)
	{
		Frame.AddBeam(MyVectorBeams[j]);
	};

	// Return Frame Geometry.
	return Frame;
};

ElasticFrame2D ElasticFrame2D::GetPlasticDeformFrameGeometry()
{
	// Get the Static Geometry.
	ElasticFrame2D Frame = this -> GetPlasticFrameGeometry();

	// Change the Nodes.
	Frame += this -> GetNodeDeformation();

	// Return the Frame Geometry.
	return Frame;
};


void ElasticFrame2D::ModifyVectorForHinges(mat HingingMatrix)
{
	MyNodesDisplacements = HingingMatrix.t() * MyNodesDisplacements;
};