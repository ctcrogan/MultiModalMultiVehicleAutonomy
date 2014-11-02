
/**************************************************************************/
/* Convert XYZ coordinates to cartesien LLA (Latitude/Longitude/Altitude) */
/* Alcatel Alenia Space - Nicolas Hennion                                 */
/* Licence GPL															  */
/* Version 0.1                                                            */
/**************************************************************************/

#include <iostream>
#include <math.h>
using namespace std;


double* convertGPS(double x, double y, double z) 
{
	// Variables
	double b, ep, p, th, lon, lat, n, alt;
	
	// Constants (WGS ellipsoid) 
	const double a = 6378137;
	const double e = 8.1819190842622e-2;
	const double pi = 3.141592653589793;
	
	// Calculation
	b = sqrt(pow(a,2)*(1-pow(e,2)));
	ep = sqrt((pow(a,2)-pow(b,2))/pow(b,2));
	p = sqrt(pow(x,2)+pow(y,2));
	th = atan2(a*z, b*p);
	lon = atan2(y, x);
	lat = atan2((z+ep*ep*b*pow(sin(th),3)), (p-e*e*a*pow(cos(th),3)));
	n = a/sqrt(1-e*e*pow(sin(lat),2));
	alt = p/cos(lat)-n;	
	lat = (lat*180)/pi;
	lon = (lon*180)/pi;

	double *lla = new double[3];
	lla[0] = lat; lla[1] = lon; lla[2] = alt;

	return lla;
}