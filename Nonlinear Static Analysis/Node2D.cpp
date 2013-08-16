#include "Node2D.h"
#include "stdafx.h"

//using namespace std;

///<summary>Initialize the Node2D object.</summary>
///<param name="ID"></param>
///<param name="X"></param>
///<param name="Y"></param>
///<param name="ReleaseTranslationX"></param>
///<param name="ReleaseTranslationY"></param>
///<param name="ReleaseRotationZ"></param>
///<remarks>description</remarks>
void Node2D::Initialize(int ID = -1, 
						double X = 0.0, 
						double Y = 0.0, 
						bool ReleaseTranslationX = true, 
						bool ReleaseTranslationY = true, 
						bool ReleaseRotationZ = true)
{
	// Set the ID
	MyID = ID;

	// Set the Location.
	MyX = X;
	MyY = Y;
	
	// Default Rotation is zeros.
	MyRotation = 0.0;

	// Set the Translation.
	MyDofTranslationX = ReleaseTranslationX;
	MyDofTranslationY = ReleaseTranslationY;
	MyDofRotationZ = ReleaseRotationZ;

	// Set Default to -1 meaning index undefined until set.
	MyNodeDOFIndexX = -1;
	MyNodeDOFIndexY = -1;
	MyNodeDOFIndexZ = -1;
}

/// <summary> </summary>
void Node2D::Clone(const Node2D &Node)
{
	MyID = Node.MyID;

	// Copy the Unit
	MyX = Node.MyX;
	MyY = Node.MyY;
	
	MyDofTranslationX = Node.MyDofTranslationX;
	MyDofTranslationY = Node.MyDofTranslationY;
	MyDofRotationZ = Node.MyDofRotationZ;
	
	MyNodeDOFIndexX = Node.MyNodeDOFIndexX;
	MyNodeDOFIndexY = Node.MyNodeDOFIndexY;
	MyNodeDOFIndexZ = Node.MyNodeDOFIndexZ;

	MyVectorBeams = Node.MyVectorBeams;

	MyBeamDOFIndexX = Node.MyBeamDOFIndexX;
	MyBeamDOFIndexY = Node.MyBeamDOFIndexY;
	MyBeamDOFIndexZ = Node.MyBeamDOFIndexZ;
}

/// <summary> Constructors and Destructors </summary>
Node2D::Node2D() 
{
	// Call the Initialize Method.
	this -> Initialize();
};

/// <summary> Initialize the X Coordinates. </summary>
Node2D :: Node2D(int ID, 
				 double X, 
				 double Y, 
				 bool ReleaseTranslationX, 
				 bool ReleaseTranslationY, 
				 bool ReleaseRotationZ) 
{
	// Call the Initialize Method.
	this -> Initialize(ID, X, Y, ReleaseTranslationX, ReleaseTranslationY, ReleaseRotationZ);
};

/// <summary> Copy Constructor. </summary>
Node2D::Node2D(const Node2D &Node)
{
	this -> Clone(Node);
};

// Set the Operator
Node2D& Node2D :: operator =(const Node2D &Node)
{
	this -> Clone(Node);

	// Return Pointer of this.
	return *this;
};

///<summary>Add to the Node location and rotation.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void Node2D::operator+= (const Node2D &Node)
{
	MyX += Node.MyX;
	MyY += Node.MyY;
	MyRotation += Node.MyRotation;
};

///<summary>Return a copy of the Node.</summary>
///<param name="Node"></param>
///<returns>description</returns>
///<remarks>description</remarks>
Node2D Node2D::CopyNode(Node2D *Node)
{
	Node2D NewNode;

	NewNode.MyID = Node -> MyID;
	
	NewNode.MyX = Node -> MyX;
	NewNode.MyY = Node -> MyY;

	NewNode.MyDofTranslationX = Node -> MyDofTranslationX;
	NewNode.MyDofTranslationY = Node -> MyDofTranslationY;
	NewNode.MyDofRotationZ = Node -> MyDofRotationZ;

	return NewNode;
};

// Release the Variables.
Node2D::~Node2D(void)
{
	// Delete All Dynamic Arrays
};

/// <summary> Return Nodal ID </summary>
int Node2D::GetID() { return MyID; }

/// <summary> Return X Coordinate. </summary>
double Node2D::GetX() { return MyX; }
		
/// <summary> Return Y Coordinate. </summary>
double Node2D::GetY() { return MyY; }

double Node2D::GetRotationZ() { return MyRotation; };

/// <summary> Return if the true/false Translation X Degrees of Freedom. </summary>
bool Node2D::GetDofTranslationX() { return MyDofTranslationX; }

/// <summary> Return if the true/false Translation Y Degrees of Freedom. </summary>
bool Node2D::GetDofTranslationY() { return MyDofTranslationY; }

/// <summary> Return if the true/false Translation Degrees of Freedom. </summary>
bool Node2D::GetDofRotationZ() { return MyDofRotationZ; }

/// <summary> Print the Node. </summary>
void Node2D::Print() {
	cout << "Node ID: \t" << MyID << "\t Address: \t" << this  << endl;
	cout << "X: \t" << MyX << endl;
	cout << "Y: \t" << MyY << endl;
	cout << "Dof Translation X: \t" << MyDofTranslationX << endl;
	cout << "Dof Translation Y: \t" << MyDofTranslationY << endl;
	cout << "Dof Rotation Z: \t" << MyDofRotationZ << endl;

	for (int i = 0; i < MyVectorBeams.size(); i++)
	{
		cout << "Beam ID: \t" << MyVectorBeams[i] -> GetID();
		cout << "\t Address: \t" << MyVectorBeams[i] << endl;
	};

	for (int j = 0; j < MyVectorNodeLoads.size(); j++)
	{
		cout << "Node Load Address: \t" << MyVectorNodeLoads[j];
	};

	cout << endl;
};

///<summary>Clear all the References between the two sequence.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void Node2D::ClearAll()
{
	this -> ClearBeams();
	this -> ClearLoads();
};

///<summary>Add the Beam Pointer to the Vector.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void Node2D::AddBeam(Beam2D *Beam)
{
	MyVectorBeams.push_back(Beam);
};

///<summary>Clear all pointers of beams connected to node.</summary>
///<remarks></remarks>
void Node2D::ClearBeams()
{
	MyVectorBeams.clear();

	// Clear all references to DOF Index.
	MyBeamDOFIndexX.clear();
	MyBeamDOFIndexY.clear();
	MyBeamDOFIndexZ.clear();
};

///<summary>Add the Load Pointer to the Vector.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void Node2D::AddLoad(NodeLoad2D* LoadPtr)
{
	MyVectorNodeLoads.push_back(LoadPtr);
};

///<summary>Clear all pointers of loads connect to node.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void Node2D::ClearLoads()
{
	MyVectorNodeLoads.clear();
};

int Node2D::GetNumOfDofTranslationX()
{
	// Initialize the Degrees of Freedom.
	size_t DOF = 0;

	// Get the Amount of Beams.
	size_t NumOfBeamAtJoint = MyVectorBeams.size();

	// Initialize the Number of Releases.
	size_t NumOfReleases = 0;

	for (size_t i = 0; i < NumOfBeamAtJoint; i++)
	{
		if (MyVectorBeams[i] -> IsTranslationXRelease(MyID) == true)
		{
			// Increment the Releases.
			NumOfReleases++;
		};
	}
	
	if (NumOfReleases == NumOfBeamAtJoint)
	{
		// The Amount of DOF is equal to the number of releases.
		DOF = NumOfReleases;
	}	
	else
	{
		DOF = NumOfReleases + 1;
	};

	return DOF;
};

int Node2D::GetNumOfDofTranslationY()
{
	// Initialize the Degrees of Freedom.
	size_t DOF = 0;

	// Get the Amount of Beams.
	size_t NumOfBeamAtJoint = MyVectorBeams.size();

	// Initialize the Number of Releases.
	size_t NumOfReleases = 0;

	for (size_t i = 0; i < NumOfBeamAtJoint; i++)
	{
		if (MyVectorBeams[i] -> IsTranslationYRelease(MyID) == true)
		{
			// Increment the Releases.
			NumOfReleases++;
		};
	}
	
	if (NumOfReleases == NumOfBeamAtJoint)
	{
		// The Amount of DOF is equal to the number of releases.
		DOF = NumOfReleases;
	}	
	else
	{
		DOF = NumOfReleases + 1;
	};

	return DOF;
};

int Node2D::GetNumOfDofRotationZ()
{
	// Initialize the Degrees of Freedom.
	size_t DOF = 0;

	// Get the Amount of Beams.
	size_t NumOfBeamAtJoint = MyVectorBeams.size();

	// Initialize the Number of Releases.
	size_t NumOfReleases = 0;

	for (size_t i = 0; i < NumOfBeamAtJoint; i++)
	{
		if (MyVectorBeams[i] -> IsRotationZRelease(MyID) == true)
		{
			// Increment the Releases.
			NumOfReleases++;
		};
	}
	
	if (NumOfReleases == NumOfBeamAtJoint)
	{
		// The Amount of DOF is equal to the number of releases.
		DOF = NumOfReleases;
	}	
	else
	{
		DOF = NumOfReleases + 1;
	};

	return DOF;
};

///<summary>Set the series of Degree of Freedom in the total system.</summary>
///<param name="DOF"></param>
///<remarks>description</remarks>
void Node2D::SetDOF(vector <DegreeOfFreedom2D> &DOF)
{
	/// TODO: The DOF should be establish instead of using an index list.  
	/// The idea is to add a DOF for each node constraint.  And then for each
	/// Hinge add another DOF.  At a later time, if the DOF does not have any 
	/// Stiffness from member contribution, then remove it from the system.
	/// Otherwise keep it.  A consideration is needed for releases at oblique angles.
	/// this needs further refinement.
};

///<summary>Set the constraint of the Node.</summary>
///<param name="DofTranslationX">True is release, False is constrain.</param>
///<param name="DofTranslationY">True is release, False is constrain.</param>
///<param name="DofRotationZ">True is release, False is constrain.</param>
///<returns>description</returns>
///<remarks>description</remarks>
void Node2D::SetDOF(bool DofTranslationX, 
					bool DofTranslationY, 
					bool DofRotationZ)
{
	MyDofTranslationX = DofTranslationX;
	MyDofTranslationY = DofTranslationY;
	MyDofRotationZ = DofRotationZ;
};

/// <summary>Set the DOF Index for each beam.</summary>
int Node2D::SetDOFIndex(int NextAvaliableIndex)
{
	int Index = NextAvaliableIndex;
	
	// Set the X Index.
	Index = this -> SetDOFIndexX(Index);

	// Set the Y Index.
	Index = this -> SetDOFIndexY(Index);

	// Set the Z Index.
	Index = this -> SetDOFIndexZ(Index);

	// Retun the Index.
	return Index;
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
int Node2D::SetDOFIndexX(int NextAvaliableIndex)
{
	// Get the Amount of Beams.
	size_t NumOfBeam = MyVectorBeams.size();

	// Initialize the Number of Releases.
	size_t NumOfReleases = 0;

	for (size_t i = 0; i < NumOfBeam; i++)
	{
		if (MyVectorBeams[i] -> IsTranslationXRelease(MyID) == true)
		{
			// Increment the Releases.
			NumOfReleases++;
		};
	}

	// Set the DOF Index for the Beams.
	if (NumOfReleases == 0) 
	{
		MyNodeDOFIndexX = NextAvaliableIndex;

		// If there is not release, all the DOF have the same Index.
		for (int i = 0; i < NumOfBeam; i++)
		{
			// Store the Index relative to the Beam Index.
			MyBeamDOFIndexX.push_back(MyNodeDOFIndexX);
		};

		// Increment the Index.
		NextAvaliableIndex++;
	}
	else if (NumOfBeam == 1 && NumOfReleases >= 0)
	{
		MyNodeDOFIndexX = NextAvaliableIndex;

		// Store the Index relative to the Beam Index.
		MyBeamDOFIndexX.push_back(MyNodeDOFIndexX);

		// Increment the Index.
		NextAvaliableIndex++;
	}
	else if (NumOfBeam == NumOfReleases)
	{
		// A full hinge, so there needs to be DOF for all of them.
		for (int i = 0; i < NumOfBeam; i++)
		{
			// Store the Index relative to the Beam Index.
			MyBeamDOFIndexX.push_back(NextAvaliableIndex);

			// Increment the Next Avalible DOF
			NextAvaliableIndex++;
		};

		// Release the Constraint.
		MyDofTranslationX = true;
	}
	else
	{
		// Set the Constrain Index.
		MyNodeDOFIndexX = NextAvaliableIndex;;

		// Increment the Next Avaliable Index.
		NextAvaliableIndex++;

		// A full hinge, so there needs to be DOF for all of them.
		for (int i = 0; i < NumOfBeam; i++)
		{
			if (MyVectorBeams[i] -> IsTranslationXRelease(MyID) == true)
			{
				// Store the Index relative to the Beam Nodal Index.
				MyBeamDOFIndexX.push_back(NextAvaliableIndex);

				// Increment the Releases.
				NextAvaliableIndex++;
			}
			else
			{
				// Store the Constrain Index reltive to the Beam Nodal Index.
				MyBeamDOFIndexX.push_back(MyNodeDOFIndexX);

				// No Need to Increment.
			};
		};
	};
	
	return NextAvaliableIndex;
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
int Node2D::SetDOFIndexY(int NextAvaliableIndex)
{
	// Get the Amount of Beams.
	size_t NumOfBeam = MyVectorBeams.size();

	// Initialize the Number of Releases.
	size_t NumOfReleases = 0;

	// Find the Number of Beams with Hinges.
	for (size_t i = 0; i < NumOfBeam; i++)
	{
		if (MyVectorBeams[i] -> IsTranslationYRelease(MyID) == true)
		{
			// Increment the Releases.
			NumOfReleases++;
		};
	}

	// Set the DOF Index for the Beams.
	if (NumOfReleases == 0) 
	{
		MyNodeDOFIndexY = NextAvaliableIndex;

		// If there is not release, all the DOF have the same Index.
		for (int i = 0; i < NumOfBeam; i++)
		{
			// Store the Index relative to the Beam Index.
			MyBeamDOFIndexY.push_back(MyNodeDOFIndexY);
		};

		// Increment the Index.
		NextAvaliableIndex++;
	}
	else if (NumOfBeam == 1 && NumOfReleases >= 0)
	{
		MyNodeDOFIndexY = NextAvaliableIndex;

		// Store the Index relative to the Beam Index.
		MyBeamDOFIndexY.push_back(MyNodeDOFIndexY);

		// Increment the Index.
		NextAvaliableIndex++;
	}
	else if (NumOfBeam == NumOfReleases)
	{
		// A full hinge, so there needs to be DOF for all of them.
		for (int i = 0; i < NumOfBeam; i++)
		{
			// Store the Index relative to the Beam Index.
			MyBeamDOFIndexY.push_back(NextAvaliableIndex);

			// Increment the Next Avalible DOF
			NextAvaliableIndex++;
		};

		// Release the Constraint.
		MyDofTranslationY = true;
	}
	else
	{
		// Set the Constrain Index.
		MyNodeDOFIndexY = NextAvaliableIndex;

		// Increment the Next Avaliable Index.
		NextAvaliableIndex++;

		// A full hinge, so there needs to be DOF for all of them.
		for (int i = 0; i < NumOfBeam; i++)
		{
			if (MyVectorBeams[i] -> IsTranslationYRelease(MyID) == true)
			{
				// Store the Index relative to the Beam Nodal Index.
				MyBeamDOFIndexY.push_back(NextAvaliableIndex);

				// Increment the Releases.
				NextAvaliableIndex++;
			}
			else
			{
				// Store the Constrain Index reltive to the Beam Nodal Index.
				MyBeamDOFIndexY.push_back(MyNodeDOFIndexY);

				// No Need to Increment.
			};
		};
	};
	
	return NextAvaliableIndex;
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
int Node2D::SetDOFIndexZ(int NextAvaliableIndex)
{

	// Get the Amount of Beams.
	size_t NumOfBeam = MyVectorBeams.size();

	// Initialize the Number of Releases.
	size_t NumOfReleases = 0;

	for (size_t i = 0; i < NumOfBeam; i++)
	{
		if (MyVectorBeams[i] -> IsRotationZRelease(MyID) == true)
		{
			// Increment the Releases.
			NumOfReleases++;
		};
	}

	// Set the DOF Index for the Beams.
	if (NumOfReleases == 0) 
	{
		// Set the Node DOF.
		MyNodeDOFIndexZ = NextAvaliableIndex;

		// If there is not release, all the DOF have the same Index.
		for (int i = 0; i < NumOfBeam; i++)
		{
			// Store the Index relative to the Beam Index.
			MyBeamDOFIndexZ.push_back(MyNodeDOFIndexZ);
		};
		
		// Increment to Next Index.
		NextAvaliableIndex++;
	}
	else if (NumOfBeam == 1 && NumOfReleases >= 0)
	{
		MyNodeDOFIndexZ = NextAvaliableIndex;

		// Store the Index relative to the Beam Index.
		MyBeamDOFIndexZ.push_back(MyNodeDOFIndexZ);

		// Increment the Index.
		NextAvaliableIndex++;
	}
	else if (NumOfBeam == NumOfReleases)
	{
		// A full hinge, so there needs to be DOF for all of them.
		for (int i = 0; i < NumOfBeam; i++)
		{
			// Store the Index relative to the Beam Index.
			MyBeamDOFIndexZ.push_back(NextAvaliableIndex);

			// Increment the Next Avalible DOF
			NextAvaliableIndex++;
		};

		// Release the Constraint.
		MyDofRotationZ = true;
	}
	else
	{
		// Set the Constrain Index.
		MyNodeDOFIndexZ = NextAvaliableIndex;

		// Increment the Next Avaliable Index.
		NextAvaliableIndex++;

		// A full hinge, so there needs to be DOF for all of them.
		for (int i = 0; i < NumOfBeam; i++)
		{
			if (MyVectorBeams[i] -> IsRotationZRelease(MyID) == true)
			{
				// Store the Index relative to the Beam Nodal Index.
				MyBeamDOFIndexZ.push_back(NextAvaliableIndex);

				// Increment the Releases.
				NextAvaliableIndex++;
			}
			else
			{
				// Store the Constrain Index relative to the Beam Nodal Index.
				MyBeamDOFIndexZ.push_back(MyNodeDOFIndexZ);

				// No Need to Increment.
			};
		};
	};
	
	return NextAvaliableIndex;
};

/// <summary>Get the Beam Index.</summary>
int Node2D::GetBeamIndex(int BeamID)
{
	int Index = -1;
	
	// Loop through the Vector.
	for (int i = 0; i < MyVectorBeams.size(); i++)
	{
		// Check if the Beam ID is the same as what is in the vector.
		if (BeamID == MyVectorBeams[i] -> GetID())
		{
			// Store the Index.
			Index = i;
		};
	};
	
	// Return the Index. If -1 then Beam ID not found.
	return Index;
};

/// <summary>Get the Beam Index for the DOF in the X Coordinate.
int Node2D::GetBeamDOFIndexX(int BeamID)
{
	// Find the Beam Index inside the vector and Return DOF Freedom.
	return MyBeamDOFIndexX[this -> GetBeamIndex(BeamID)];
};

/// <summary>Get the Beam Index for the DOF in the Y Coordinate.
int Node2D::GetBeamDOFIndexY(int BeamID)
{
	// Find the Beam Index inside the vector and Return DOF Freedom.
	return MyBeamDOFIndexY[this -> GetBeamIndex(BeamID)];
};

/// <summary>Get the Beam Index for the DOF in the X Coordinate.
int Node2D::GetBeamDOFIndexZ(int BeamID)
{
	// Find the Beam Index inside the vector and Return DOF Freedom.
	return MyBeamDOFIndexZ[this -> GetBeamIndex(BeamID)];
};

/// <summary>Return the Degress of Freedom Indexes X, Y, and Z for the Beams. </summary>
vector <int> Node2D::GetBeamDOFIndexes(int BeamID)
{
	// Store the Beam Index from ID.
	int Index = this -> GetBeamIndex(BeamID);

	vector <int> Indexes;

	// Store the X, Y, and Z Index.
	Indexes.push_back(MyBeamDOFIndexX[Index]);
	Indexes.push_back(MyBeamDOFIndexY[Index]);
	Indexes.push_back(MyBeamDOFIndexZ[Index]);

	return Indexes;
};

/// <summary>Return the Degree of Freedom Indexes X, Y, and Z for the Nodes. '-1' means invalid.</summary>
vector <int> Node2D::GetNodeDOFIndexes()
{
	vector <int> Indexes;

	// Store the X, Y, and Z Index.
	Indexes.push_back(MyNodeDOFIndexX);
	Indexes.push_back(MyNodeDOFIndexY);
	Indexes.push_back(MyNodeDOFIndexZ);

	return Indexes;
};

/// <summary>Return the Indexes for all the Release DOF.</summary>
vector <int> Node2D::GetNodeRestrainedIndexes()
{
	vector <int> RestrainIndexes;

	// Check to see if the Translation X is Restrained.
	if(MyDofTranslationX == false)
	{
		RestrainIndexes.push_back(MyNodeDOFIndexX);
	};

	// Check to see if the Translation Y is Restrained.
	if (MyDofTranslationY == false)
	{
		RestrainIndexes.push_back(MyNodeDOFIndexY);
	};

	// Check to see if the Translation Z is Restrained.
	if (MyDofRotationZ == false)
	{
		RestrainIndexes.push_back(MyNodeDOFIndexZ);
	};

	return RestrainIndexes;
};

///<summary>Check if all the Beam formed a hinge, if so then release the constraint.</summary>
///<remarks>description</remarks>
void Node2D::CheckBeamMomentHinging()
{
	bool AllDofHinge = true;

	// Loop through the Beams to see if it hinge.
	for (size_t i = 0; MyVectorBeams.size(); i++)
	{
		// If it does not hinge, then we release the DOF.
		if (MyBeamDOFIndexZ[i] != true)
		{
			AllDofHinge = false;
		};
	};

	// Release the Constraint if all the DOF hinged.
	if (AllDofHinge == true)
	{
		MyDofRotationZ = true;
	};
};

///<summary>Add to existing X Coordinate for the translated distance.</summary>
///<param name="DeltaX"></param>
///<remarks>description</remarks>
void Node2D::IncrementTranslationX(double DeltaX)
{
	MyX += DeltaX;
};

///<summary>Add to existing Y Coordinate for the translated distance.</summary>
///<param name="DeltaY"></param>
///<remarks>description</remarks>
void Node2D::IncrementTranslationY(double DeltaY)
{
	MyY += DeltaY;
};

///<summary>Add to existing Rotation.</summary>
///<param name="DeltaZ"></param>
///<remarks>description</remarks>
void Node2D::IncrementRotationZ(double DeltaZ)
{
	MyRotation += DeltaZ;
};

///<summary>Return the index vector for the x-degree of freedom.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
vector <int> Node2D::GetDOFIndexX() 
{ 
	return MyBeamDOFIndexX;
};

///<summary>Return the index vector for the y-degree of freedom.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
vector <int> Node2D::GetDOFIndexY()
{
	return MyBeamDOFIndexY;
};

///<summary>Return the index vector for the z-degree of freedom.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
vector <int> Node2D::GetDOFIndexZ()
{
	return MyBeamDOFIndexZ;
};

///<summary>Return the local force vector.</summary>
///<param name="name"></param>
///<returns>description</returns>
///<remarks>description</remarks>
mat Node2D::GetLocalForceVector()
{	
	// Declare the Force Vector
	mat LocalForceVector(3, 1);

	// Initialize the Force Vector.
	LocalForceVector.fill(0.0);

	// Loop to find the forces.
	for(size_t i = 0; i < MyVectorNodeLoads.size(); i++)
	{
		// Add the Local Force Vector.
		LocalForceVector = LocalForceVector + MyVectorNodeLoads[i] -> GetLocalLoadVector();
	};

	return LocalForceVector;
};