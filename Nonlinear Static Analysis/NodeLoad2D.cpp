#include "NodeLoad2D.h"

/// <summary></summary>
void NodeLoad2D::Initialize(double GlobalXForce = 0.0, 
							double GlobalYForce = 0.0, 
							double Moment = 0.0, 
							int XForceInitialTimeStep = 0.0, 
							int YForceInitialTimeStep = 0.0, 
							int MomentInitialTimeStep = 0.0)
{
	// Magnitude of the force.
	MyGlobalXForce = GlobalXForce;
	MyGlobalYForce = GlobalYForce;
	MyMoment = Moment;

	// Initial Time Step;
	MyXForceInitialTimeStep = XForceInitialTimeStep;
	MyYForceInitialTimeStep = YForceInitialTimeStep;
	MyMomentInitialTimeStep = MomentInitialTimeStep;
};

/// <summary></summary>
void NodeLoad2D::Clone(const NodeLoad2D &Load)
{
	// Store the Relevant Node.
	MyNode = Load.MyNode;
	
	// Store the Various Magnitude;
	MyGlobalXForce = Load.MyGlobalXForce;
	MyGlobalYForce = Load.MyGlobalYForce;
	MyMoment = Load.MyMoment;

	// Initial Time Step:
	MyXForceInitialTimeStep = Load.MyXForceInitialTimeStep;
	MyYForceInitialTimeStep = Load.MyYForceInitialTimeStep;
	MyMomentInitialTimeStep = Load.MyMomentInitialTimeStep;

	MyVectorXForcePercentage = Load.MyVectorXForcePercentage;
	MyVectorYForcePercentage = Load.MyVectorYForcePercentage;
	MyVectorMomentPercentage = Load.MyVectorMomentPercentage;
};

/// <summary></summary>
NodeLoad2D::NodeLoad2D(void)
{
	MyNode = NULL;
	
	this -> Initialize();
};

/// <summary></summary>
NodeLoad2D::NodeLoad2D(Node2D &Node, 
					   double GlobalXForce, 
					   double GlobalYForce, 
					   double Moment)
{
	// Node 
	MyNode = &Node;
	
	this -> Initialize(GlobalXForce, GlobalYForce, Moment);

	// Set the Percentage at a 100% Load applied at time 0.
	MyVectorXForcePercentage.push_back(1.00);
	MyVectorYForcePercentage.push_back(1.00);
	MyVectorMomentPercentage.push_back(1.00);

	// Set the Pointer to Node();
	this -> SetLoadPointerToNode();
};

///<summary></summary>
///<param name="name"></param>
///<remarks>description</remarks>
NodeLoad2D::NodeLoad2D(Node2D &Node, 
					   double GlobalXForce, 
					   double GlobalYForce, 
					   double Moment, 
					   int XForceInitialTimeStep, 
					   int YForceInitialTimeStep, 
					   int MomentInitialTimeStep, 
					   std::vector<double> XForcePercentage, 
					   std::vector<double> YForcePercentage, 
					   std::vector<double> MomentPercentage)
{
	// Node 
	MyNode = &Node;
	
	// Initialize the Values.
	this -> Initialize(GlobalXForce, 
					   GlobalYForce, 
					   Moment, 
					   XForceInitialTimeStep, 
					   YForceInitialTimeStep, 
					   MomentInitialTimeStep);

	// Set the Percentage at a 100% Load applied at time 0.
	MyVectorXForcePercentage = XForcePercentage;
	MyVectorYForcePercentage = YForcePercentage;
	MyVectorMomentPercentage = MomentPercentage;

	// Set the Pointer to Node();
	this -> SetLoadPointerToNode();
};

/// <summary></summary>
NodeLoad2D::~NodeLoad2D(void) 
{

};

/// <summary>Assignment Operator.</summary>
NodeLoad2D& NodeLoad2D::operator=(const NodeLoad2D &Load)
{
	this -> Clone(Load);

	return *this;
};

///<summary>Assignment Operator: Multiplies the Magnitude of the Load by the increment.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
NodeLoad2D& NodeLoad2D::operator*(double Factor)
{
	// Modifiy the Magnitude.
	MyGlobalXForce = Factor * MyGlobalXForce;
	MyGlobalYForce = Factor * MyGlobalYForce;
	MyMoment = Factor * MyMoment;

	// Return the Address of this Object.
	return *this;
};

///<summary>Assign the Node Location.</summary>
///<param name="Node"></param>
///<remarks>description</remarks>
void NodeLoad2D::SetNode(Node2D *NodePtr)
{
	// Set the Pointers.
	MyNode = NodePtr;
};

///<summary>Assign the load pointer to Node.</summary>
///<param name="name"></param>
///<remarks>description</remarks>
void NodeLoad2D::SetLoadPointerToNode()
{
	MyNode -> AddLoad(this);
};

/// <summary>Return the Node Load Increment by value.</summary>
NodeLoad2D NodeLoad2D::GetNodeLoadIncrement(int TimeStep)
{
	NodeLoad2D NodeLoad = NodeLoad2D(*MyNode, 
									 this -> GetGlobalXForce(TimeStep),
									 this -> GetGlobalYForce(TimeStep),
									 this -> GetGlobalMoment(TimeStep));

	return NodeLoad;
};

/// <summary></summary>
int NodeLoad2D::GetNodeID()
{
	return MyNode -> GetID();
};

/// <summary>Return the Global Force in the X-Direction.</summary>
double NodeLoad2D::GetGlobalXForce() 
{ 
	return MyGlobalXForce; 
};
		
/// <summary>Return the Global Force in the Y-Direction.</summary>
double NodeLoad2D::GetGlobalYForce() 
{ 
	return MyGlobalYForce; 
};

/// <summary>Return the Global Moment.</summary>
double NodeLoad2D::GetGlobalMoment() 
{ 
	return MyMoment; 
};

/// <summary>Return the Global Force in the X-Direction.</summary>
double NodeLoad2D::GetGlobalXForce(int TimeStep) 
{ 
	// Initialize the Force System.
	double Magnitude = 0.0;

	// Find the Number of Time Step.
	int NumOfTimeStep = MyVectorXForcePercentage.size();
	
	// Check to see if there are percentages to use.
	if (NumOfTimeStep != 0)
	{
		if (TimeStep < MyXForceInitialTimeStep)
		{
			// Before the Initial Time Step. No Magnitude.
			Magnitude = 0.0;
		}
		else if (TimeStep >= NumOfTimeStep + MyXForceInitialTimeStep)
		{
			// After the Full Time Step. No Magnitude.
			Magnitude = 0.0;
		}
		else if (TimeStep = MyXForceInitialTimeStep)
		{
			// Start at the Time Step. Get the magnitude of the first percentage.
			Magnitude = MyGlobalXForce * MyVectorXForcePercentage[0];
		}
		else
		{
			// After the initial time step but still within the duration.  Return the percentage of the magnitude.
			Magnitude = MyGlobalXForce * (MyVectorXForcePercentage[TimeStep] - MyVectorXForcePercentage[TimeStep - 1]);
		};
	}
	
	// Return the Magnitude of the Force.
	return Magnitude; 
};
		
/// <summary>Return the Global Force in the Y-Direction.</summary>
double NodeLoad2D::GetGlobalYForce(int TimeStep) 
{ 
	// Initialize the Force System.
	double Magnitude = 0.0;

	// Find the Number of Time Step.
	int NumOfTimeStep = MyVectorYForcePercentage.size();
	
	// Check to see if there are percentages to use.
	if (NumOfTimeStep != 0)
	{
		if (TimeStep < MyYForceInitialTimeStep)
		{
			// Before the Initial Time Step. No Magnitude.
			Magnitude = 0.0;
		}
		else if (TimeStep >= NumOfTimeStep + MyYForceInitialTimeStep)
		{
			// After the Full Time Step. No Magnitude.
			Magnitude = 0.0;
		}
		else if (TimeStep = MyYForceInitialTimeStep)
		{
			// Start at the Time Step. Get the magnitude of the first percentage.
			Magnitude = MyGlobalYForce * MyVectorYForcePercentage[0];
		}
		else
		{
			// After the initial time step but still within the duration.  Return the percentage of the magnitude.
			Magnitude = MyGlobalYForce * (MyVectorYForcePercentage[TimeStep] - MyVectorYForcePercentage[TimeStep - 1]);
		};
	};

	// Return the Magnitude of the Force.
	return Magnitude; 
};

/// <summary>Return the Global Moment.</summary>
double NodeLoad2D::GetGlobalMoment(int TimeStep) 
{ 
	// Initialize the Force System.
	double Magnitude = 0.0;

	// Find the Number of Time Step.
	int NumOfTimeStep = MyVectorMomentPercentage.size();
	
	// Check to see if there are percentages to use.
	if (NumOfTimeStep != 0)
	{
		if (TimeStep < MyMomentInitialTimeStep)
		{
			// Before the Initial Time Step. No Magnitude.
			Magnitude = 0.0;
		}
		else if (TimeStep >= NumOfTimeStep + MyMomentInitialTimeStep)
		{
			// After the Full Time Step. No Magnitude.
			Magnitude = 0.0;
		}
		else if (TimeStep = MyMomentInitialTimeStep)
		{
			// Start at the Time Step. Get the magnitude of the first percentage.
			Magnitude = MyMoment * MyVectorMomentPercentage[0];
		}
		else
		{
			// After the initial time step but still within the duration.  Return the percentage of the magnitude.
			Magnitude = MyMoment * (MyVectorMomentPercentage[TimeStep] - MyVectorMomentPercentage[TimeStep - 1]);
		};
	};

	// Return the Magnitude of the Force.
	return Magnitude; 
};


void NodeLoad2D::Print()
{
	cout << "Node Load Address: \t" << this << endl;
	cout << "Node ID: \t" << MyNode -> GetID() << "\t Address: \t" << MyNode << endl;
	cout << "Global X Force: \t" << MyGlobalXForce << endl;
	cout << "Global Y Force: \t" << MyGlobalYForce << endl;
	cout << "Global Moment: \t\t" << MyMoment << endl;
	cout << endl;
};

///<summary>Return the Load Vector: Force in X, Force in Y, Moment in Z.</summary>
///<returns>description</returns>
///<remarks>description</remarks>
mat NodeLoad2D::GetLocalLoadVector()
{
	// Declare the Matrix.
	mat Load(3, 1);

	// Initialize the Vector.
	Load.fill(0.0);

	Load(0, 0) = MyGlobalXForce;
	Load(1, 0) = MyGlobalYForce;
	Load(2, 0) = MyMoment;

	return Load;
};