#include "StdAfx.h"
#include "NonLinearFrame2D.h"

NonLinearFrame2D::NonLinearFrame2D(void)
{
}

NonLinearFrame2D::~NonLinearFrame2D(void)
{
}

///<summary></summary>
///<param name="Frame"></param>
///<returns>description</returns>
///<remarks>description</remarks>
NonLinearFrame2D& NonLinearFrame2D::operator =(const NonLinearFrame2D &Frame)
{
	// Clone all the parameters.
	this -> Clone(Frame);

	// Return a pointer of this.
	return *this;
};

///<summary></summary>
///<param name="Frame"></param>
///<returns>description</returns>
///<remarks>description</remarks>
void NonLinearFrame2D::operator +=(const ElasticFrame2D &Frame)
{	
	//vector <Node2D> FrameNodes = Frame.GetNodes();

	for (size_t i = 0; i < MyVectorNodes.size(); i++)
	{
		// Add the New Displacement to the Nodes.
		//MyVectorNodes[i] += FrameNodes[i];
	};
};