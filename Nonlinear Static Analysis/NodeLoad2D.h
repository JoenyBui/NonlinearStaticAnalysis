#include "stdafx.h"
#include "Node2D.h"

#pragma once

#ifndef NodeLoad2D_H
#define NodeLoad2D_H

using namespace std;
using namespace arma;

// Forward Declaration.
class Node2D;

class NodeLoad2D
{
	private:
		/// <summary>Node where the Load is applied.</summary>
		Node2D *MyNode;
		
		/// <summary>Magnitude of the Loads.</summary>
		double MyGlobalXForce;
		double MyGlobalYForce;
		double MyMoment;

		/// <summary>Type of Load.</summary>
		string MyType;
		
		///
		// The Initial Time Step where the Load will begin applied.
		int MyXForceInitialTimeStep;
		int MyYForceInitialTimeStep;
		int MyMomentInitialTimeStep;

		// The array
		vector <double> MyVectorXForcePercentage;
		vector <double> MyVectorYForcePercentage;
		vector <double> MyVectorMomentPercentage;

	public:
		void Initialize(double GlobalXForce, 
						double GlobalYForce, 
						double Moment,
						int XForceInitialTimeStep,
						int YForceInitialTimeStep,
						int MomentInitialTimeStep);

		void Clone(const NodeLoad2D &Load);

		/// <summary></summary>
		NodeLoad2D(void);
		
		/// <summary>Constructor: </summary>
		NodeLoad2D(Node2D &Node, 
					double GlobalXForce, 
					double GlobalYForce, 
					double Moment);
		
		/// <summary></summary>
		NodeLoad2D(Node2D &Node, 
					double GlobalXForce, 
					double GlobalYForce, 
					double Moment, 
					int XForceInitialTimeStep,
					int YForceInitialTimeStep,
					int MomentInitialTimeStep,
					vector <double> XForcePercentage,
					vector <double> YForcePercentage,
					vector <double> MomentPercentage);

		/// <summary>Destructor: </summary>
		~NodeLoad2D(void);
		
		/// <summary>Nodal Operator.</summary>
		NodeLoad2D& operator=(const NodeLoad2D &Load);
		NodeLoad2D& operator*(double Factor);

		/// <summary></summary>
		NodeLoad2D GetNodeLoadIncrement(int TimeStep);

		void SetNode(Node2D *NodePtr);

		void SetLoadPointerToNode();

		/// <summary>Return the Nodal ID.</summary>
		int GetNodeID();

		/// <summary></summary>
		double GetGlobalXForce();
		
		/// <summary></summary>
		double GetGlobalYForce();

		/// <summary></summary>
		double GetGlobalMoment();

		/// <summary></summary>
		double GetGlobalXForce(int TimeStep);
		
		/// <summary></summary>
		double GetGlobalYForce(int TimeStep);

		/// <summary></summary>
		double GetGlobalMoment(int TimeStep);

		mat GetLocalLoadVector();

		void Print();
};

#endif
