/*******************************************************************************
* File:	        Supervisor.h
*		 
* Author:       Adan Moran-MacDonald
* Created:      26/Apr/21
*
* Systems Control Group
* Department of Electrical and Computer Engineering
* University of Toronto
*
* Last Modified: 26/Apr/21
* Last Editor:   Adan Moran-MacDonald
* Description:   A simple supervisor for switching between injection and
* dissipation VNHCs.
*******************************************************************************/
#ifndef __SUPERVISOR_H__
#define __SUPERVISOR_H__

namespace SUGAR
{

class Supervisor
{
public:
	Supervisor (); 
	virtual ~Supervisor ();

private:
	/* data */
}; // class Supervisor

} // namespace SUGAR

#endif // __SUPERVISOR_H__
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
