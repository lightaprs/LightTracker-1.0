/*
 * GEOFENCE.c
 *
 * Created: 5.11.2016 22:04:58
 *  Author: Tomy2
 */ 

 
#include "LightTrackerGeofence.h"


/*
	Adapted version of pointInPolygon() function from:	http://alienryderflex.com/polygon/
	
	Returns '0' if the point is outside of the polygon and '1' if it's inside.
	
	Expects input DEGREES * 100000 for latitude and longitude. Eg 4961070 for 49.61070 N.
	The reason is to make sure all calculations fit inside int32_t.
	
	However, this function is not very accurate due to rounding within the computation.
*/
int32_t pointInPolygon(int32_t polyCorners, int32_t * polygon, int32_t latitude, int32_t longitude)
{
	int32_t i;
	int32_t j = polyCorners * 2 - 2;
	int32_t oddNodes = 0;

	for(i = 0; i < polyCorners * 2; i += 2)
	{
		if((polygon[i + 1] < latitude && polygon[j + 1] >= latitude
			|| polygon[j + 1] < latitude && polygon[i + 1] >= latitude)
			&& (polygon[i] <= longitude || polygon[j] <= longitude))
		{
			oddNodes ^= (polygon[i] + (latitude - polygon[i + 1])
			/ (polygon[j + 1] - polygon[i + 1]) * (polygon[j] - polygon[i]) < longitude);
		}

		j = i;
	}

	return oddNodes;
}


/*
	Adapted version of pointInPolygon() function from:	http://alienryderflex.com/polygon/
	
	Returns '0' if the point is outside of the polygon and '1' if it's inside.
	
	Uses FLOAT input for better accuracy.
*/
int32_t pointInPolygonF(int32_t polyCorners, float * polygon, float latitude, float longitude)
{
	int32_t i;
	int32_t j = polyCorners * 2 - 2;
	int32_t oddNodes = 0;

	for(i = 0; i < polyCorners * 2; i += 2)
	{
		if((polygon[i + 1] < latitude && polygon[j + 1] >= latitude
		|| polygon[j + 1] < latitude && polygon[i + 1] >= latitude)
		&& (polygon[i] <= longitude || polygon[j] <= longitude))
		{
			oddNodes ^= (polygon[i] + (latitude - polygon[i + 1])
			/ (polygon[j + 1] - polygon[i + 1]) * (polygon[j] - polygon[i]) < longitude);
		}

		j = i;
	}

	return oddNodes;
}

void Lorawan_Geofence_position(float latitude, float longitude)
{

	// Europa, Africa, Middle East and West Asia
	if(longitude > -33.0 && longitude < 87.0)
	{

		//Belarus and Ukraine
		if (pointInPolygonF(8, BelarusUkraineF, latitude, longitude) == 1) {
			Lorawan_Geofence_no_tx = 0; 
			Lorawan_Geofence_region_code = _REGCODE_EU868;
			Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;
		} 
		//Israel
		else if (pointInPolygonF(6, IsraelF, latitude, longitude) == 1) {
			Lorawan_Geofence_no_tx = 0; 
			Lorawan_Geofence_region_code = _REGCODE_AS923;
			Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;		
		}					
		//Kazakhistan
		else if (pointInPolygonF(10, KazakhstanF, latitude, longitude) == 1) {
			Lorawan_Geofence_no_tx = 0; 
			Lorawan_Geofence_region_code = _REGCODE_EU868;
			Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;		
		}		
									
		//Russia - RU864
		else if(longitude > 30.0 && latitude > 44.0 )
		{
			Lorawan_Geofence_no_tx = 0; 
			Lorawan_Geofence_region_code = _REGCODE_EU868;
			Lorawan_Geofence_special_region_code = _REGCODE_RU864;
		}				 
		//India - IN865
		else if (pointInPolygonF(9, IndiaF, latitude, longitude) == 1) {

			Lorawan_Geofence_no_tx = 0; 
			Lorawan_Geofence_region_code = _REGCODE_IN865;
			Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;	
		
		}
		//Pakistan - IN865
		else if (pointInPolygonF(8, PakistanF, latitude, longitude) == 1) {

			Lorawan_Geofence_no_tx = 0; 
			Lorawan_Geofence_region_code = _REGCODE_IN865;
			Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;	
		
		}		
		//Rest of Europa and West Asia - EU868
		else {
			Lorawan_Geofence_no_tx = 0; 
			Lorawan_Geofence_region_code = _REGCODE_EU868;
			Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;			
	
		}		

	} 
	 //North & South America
	else if(longitude <= -33.0)
	{
		//United States, Canada and Mexico - US902
		if(latitude > 12.5)
		{
			Lorawan_Geofence_no_tx = 0; 
			Lorawan_Geofence_region_code = _REGCODE_US915;
			Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;		
		}		
		//Brazil
		else if (pointInPolygonF(9, BrazilF, latitude, longitude) == 1) {
			Lorawan_Geofence_no_tx = 0; 
			Lorawan_Geofence_region_code = _REGCODE_AU915;
			Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;								
		}		
		else
		{
			Lorawan_Geofence_no_tx = 0; 
			Lorawan_Geofence_region_code = _REGCODE_US915;
			Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;					
			
		}
	}

	//Russia, China, Japan, South Korea, Australia
	else if(longitude >= 87.0)
	{
		
		//Russia - RU864
		if(latitude > 52.0)
		{
			Lorawan_Geofence_no_tx = 0; 
			Lorawan_Geofence_region_code = _REGCODE_EU868;
			Lorawan_Geofence_special_region_code = _REGCODE_RU864;
		} 		
		
		else if(latitude > 22.2)
		{
				
			//Bangladesh
			if(pointInPolygonF(7, BangladeshF, latitude, longitude) == 1)		{				
				Lorawan_Geofence_no_tx = 0; 
				Lorawan_Geofence_region_code = _REGCODE_EU868;
				Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;		
			} 							
			//North Korea
			else if(pointInPolygonF(6, North_KoreaF, latitude, longitude) == 1)		{				
				Lorawan_Geofence_no_tx = 1; 
				Lorawan_Geofence_region_code = _REGCODE_AS923;
				Lorawan_Geofence_special_region_code = _REGCODE_KR920;					
			}
			//South Korea
			else if(pointInPolygonF(5, South_KoreaF, latitude, longitude) == 1)	{			
				Lorawan_Geofence_no_tx = 0; 
				Lorawan_Geofence_region_code = _REGCODE_AS923;
				Lorawan_Geofence_special_region_code = _REGCODE_KR920;			
			}
			//Japan
			else if(pointInPolygonF(7, JapanF, latitude, longitude) == 1)		{						
				Lorawan_Geofence_no_tx = 0; 
				Lorawan_Geofence_region_code = _REGCODE_AS923;
				Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;							
			}				
			//Hong Kong
			else if((longitude > 113.62 && longitude < 114.67) && (latitude > 21.6 && latitude < 22.5)) {
				Lorawan_Geofence_no_tx = 0; 
				Lorawan_Geofence_region_code = _REGCODE_AS923;
				Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;	
			}
			else if(pointInPolygonF(5, ThailandF, latitude, longitude) == 1)	{
				Lorawan_Geofence_no_tx = 0; 
				Lorawan_Geofence_region_code = _REGCODE_AS923;
				Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;								
			}			
			//South East China Sea
			else if((longitude > 120.0 && longitude < 123.45) && (latitude > 21.4 && latitude < 25.8)) {
				Lorawan_Geofence_no_tx = 0; 
				Lorawan_Geofence_region_code = _REGCODE_AS923;
				Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;		
			}
			//China (Not supported)			
			else if(pointInPolygonF(12, ChinaF, latitude, longitude) == 1)		{			
				Lorawan_Geofence_no_tx = 1; //CN470 not supported :(
				Lorawan_Geofence_region_code = _REGCODE_CN470;
				Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;										
			}
			//Mongolia
			else if(pointInPolygonF(10, MongoliaF, latitude, longitude) == 1)		{			
				Lorawan_Geofence_no_tx = 0;
				Lorawan_Geofence_region_code = _REGCODE_US915;
				Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;										
			}			
					
			else																{			
				Lorawan_Geofence_no_tx = 0; 
				Lorawan_Geofence_region_code = _REGCODE_AS923;
				Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;			
			}
		}
		
		//Australia, New Zealand
		else
		{
			if(pointInPolygonF(6, AustraliaF, latitude, longitude) == 1)		{
			
				Lorawan_Geofence_no_tx = 0; 
				Lorawan_Geofence_region_code = _REGCODE_AU915;
				Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;								
			
			}
			else if(pointInPolygonF(5, New_ZealandF, latitude, longitude) == 1)	{
			

				Lorawan_Geofence_no_tx = 0; 
				Lorawan_Geofence_region_code = _REGCODE_AU915;
				Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;	
						
			}
			else if(pointInPolygonF(5, ThailandF, latitude, longitude) == 1)	{
						
				Lorawan_Geofence_no_tx = 0; 
				Lorawan_Geofence_region_code = _REGCODE_AS923;
				Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;	
						
			}
			else																{
						
				Lorawan_Geofence_no_tx = 0; 
				Lorawan_Geofence_region_code = _REGCODE_AS923;
				Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;	
						
			}
		}
	}
	
	// shouldn't get here
	else
	{
		Lorawan_Geofence_no_tx = 0; 
		Lorawan_Geofence_region_code = _REGCODE_EU868;
		Lorawan_Geofence_special_region_code = _REGCODE_UNDEF;																			
																				
																				
	}

}


