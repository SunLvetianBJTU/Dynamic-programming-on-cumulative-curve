//  Portions Copyright 2010 Xuesong Zhou

//   If you help write or modify the code, please also list your names here.
//   The reason of having copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html

//    This file is part of DTALite.

//    DTALite is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    DTALite is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with DTALite.  If not, see <http://www.gnu.org/licenses/>.

// DTALite.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <list> 
#include <omp.h>
#include <algorithm>
#include <time.h>
#include "CSVParser.h"
#include "DTALite-S.h"
#include <functional>
#include<stdio.h>   
#include<tchar.h>
#define _MAX_LABEL_COST 99999
#define _MAX_NUMBER_OF_PAX 100
#define _MAX_NUMBER_OF_VEHICLES 100
#define _MAX_NUMBER_OF_TIME_INTERVALS 150
#define _MAX_NUMBER_OF_DEMAND_TYPES 20
#define _MAX_NUMBER_OF_PHYSICAL_NODES 10000

#define _MAX_STATES 2
// Linear congruential generator 
#define LCG_a 17364
#define LCG_c 0
#define LCG_M 65521  // it should be 2^32, but we use a small 16-bit number to save memory

// The one and only application object

CWinApp theApp;
using namespace std;
TCHAR g_SettingFileName[_MAX_PATH] = _T("./Settings.txt");

FILE* g_pFileDebugLog = NULL;

FILE* g_pFileOutputLog = NULL;
FILE* g_pFileDebugLog_LR = NULL;
FILE* g_pFileDebugLog_ADMM = NULL;
FILE* g_pTSViewOutput = NULL;
FILE* g_pNGSIMOuputLog = NULL;
FILE* g_ptrainDelayLog = NULL;

int enum_waiting_link_type = 5;  //To do: to be changed. 
int enum_road_capacity_link_type = 0;  //To do: to be changed. 
int enum_request_link_type = 100;  //To do: to be changed. 

int g_number_of_threads = 4;
int g_shortest_path_debugging_flag = 0;
int g_number_of_agents;
int g_number_of_demand_types = 1;

int time_index = 11;

double g_number_of_seconds_per_interval = 6;  // 0.2 seconds for 300 intervals per min
int g_number_of_simulation_intervals = 600 * 60 / g_number_of_seconds_per_interval;    // 60min
int g_number_of_optimization_time_intervals = 60;

int g_Simulation_StartTimeInMin = 9999;
int g_Simulation_EndTimeInMin = 0;
int g_start_simu_interval_no, g_end_simu_interval_no;

int g_Post_Simulation_DurationInMin = 120;
int g_dp_algorithm_debug_flag = 0;
float g_penalty_RHO = 0.5;
int g_optimization_method = 2;
float LR_multiplier;
//g_convert_abs_simu_interval_to_relative_simu_interval
int g_A2R_simu_interval(int abs_simu_interval)
{
	return abs_simu_interval - g_start_simu_interval_no;

}

// 6 seconds per interval
// 3600 -> 6
// 1800 -> 3
// 900 -> 1.5

std::map<int, int> g_link_key_to_seq_no_map;  // hush table, map key to internal link sequence no. 


//mfd
int g_TAU;


std::map<int, int> g_internal_node_seq_no_map;  // hush table, map external node number to internal node sequence no. 
std::map<int, int> g_internal_node_seq_no_to_node_id_map;  // hush table, map external node number to internal node sequence no. 


long g_GetLinkSeqNo(int from_node_id, int to_node_id)
{
	if (g_internal_node_seq_no_map.find(from_node_id) == g_internal_node_seq_no_map.end())
	{
		return -1; //have not been defined
	}

	if (g_internal_node_seq_no_map.find(from_node_id) == g_internal_node_seq_no_map.end())
	{
		return -1; //have not been defined
	}

	int from_node_seq_no = g_internal_node_seq_no_map[from_node_id];
	int to_node_seq_no = g_internal_node_seq_no_map[to_node_id];

	long link_key = from_node_seq_no * _MAX_NUMBER_OF_PHYSICAL_NODES + to_node_seq_no;

	if (g_link_key_to_seq_no_map.find(link_key) != g_link_key_to_seq_no_map.end())
		return g_link_key_to_seq_no_map[link_key];
	else
		return -1;
}

class CLink
{
public:
	CLink()  // construction 
	{
		//m_End_of_RedTime_in_simu_interval = 0;
		link_flag = 0;
		external_link_id = 0;
		service_type = 0;
		service_price = 0;
		VRP_load_id = -1;
		base_price = 20;
		VRP_load_difference = 0;
		LR_multiplier = 0;
		cost = 0;
		BRP_alpha = 0.15f;
		BRP_beta = 4.0f;
		link_capacity = 1000;
		free_flow_travel_time_in_min = 1;
		flow_volume = 0;
		number_of_lanes = 1;
		// mfd
		mfd_zone_id = 0;

		m_LinkOutFlowCapacity = NULL;
		m_LinkInFlowCapacity = NULL;
		m_LinkCumulativeArrival = NULL;
		m_LinkCumulativeDeparture = NULL;
		m_LinkCumulativeVirtualDelay = NULL;

		max_allowed_waiting_time = 0;

		VRP_time_window_begin = -1;
		VRP_time_window_end = 10000;
	}

	~CLink()
	{
		DeallocateMemory();
	}

	std::list<int>  m_waiting_traveler_queue;

	// all alocated as relative time
	float* m_LinkOutFlowCapacity;
	float* m_LinkInFlowCapacity;
	int m_End_of_RedTime_in_simu_interval;

	int* m_LinkCumulativeArrival;
	int* m_LinkCumulativeDeparture;
	int* m_LinkCumulativeVirtualDelay;
	float* m_LinkTravelTime;

	int m_CumulativeArrivalCount;
	int m_CumulativeDepartureCount;
	int m_CumulativeVirtualDelayCount;

	float LR_multiplier;
	float link_cost;
	int VRP_time_window_begin, VRP_time_window_end;
	float base_price;
	std::vector<float> travel_time_vector;
	std::vector<float> time_dependent_LR_multiplier_vector, time_dependent_external_cost_vector, time_dependent_ADMM_multiplier_vector, time_dependent_discharge_rate, time_dependent_inflow_rate;

	std::vector<int> time_dependent_visit_counts, time_dependent_ADMM_visit_counts,
		time_depedent_capacity_vector;
	std::vector<float> time_dependent_link_cost;
	std::vector<float> time_dependent_travel_time_vector;

	//ADMM_multiplier_matrix is used in the searching process and updated by LR_multiplier_matrix
	int max_allowed_waiting_time;

	void Setup_State_Dependent_Data_Matrix(int number_of_optimization_time_intervals)
	{

		for (int t = 0; t < number_of_optimization_time_intervals; t++)
		{
			time_dependent_visit_counts.push_back(0);
			time_dependent_ADMM_visit_counts.push_back(0);
			time_dependent_LR_multiplier_vector.push_back(0);
			time_depedent_capacity_vector.push_back(link_capacity);

			time_dependent_external_cost_vector.push_back(0);
			time_dependent_ADMM_multiplier_vector.push_back(0);
			travel_time_vector.push_back((int)(free_flow_travel_time_in_min));  //assume simulation time interval as free-flow travel time per cell 

		}

		VRP_time_window_begin = max(0, VRP_time_window_begin);
		VRP_time_window_end = min(number_of_optimization_time_intervals - 1, VRP_time_window_end);

	}

	float GetCapacityPerSimuInterval(float link_capacity_per_hour)
	{
		return link_capacity_per_hour / 3600.0 *g_number_of_seconds_per_interval;
	}

	void AllocateMemory()
	{
		m_LinkOutFlowCapacity = new float[g_number_of_simulation_intervals];
		m_LinkInFlowCapacity = new float[g_number_of_simulation_intervals];
		m_LinkCumulativeArrival = new int[g_number_of_simulation_intervals];
		m_LinkCumulativeDeparture = new int[g_number_of_simulation_intervals];
		m_LinkCumulativeVirtualDelay = new int[g_number_of_simulation_intervals];
		m_LinkTravelTime = new float[g_number_of_simulation_intervals];


		for (int t = 0; t < g_number_of_simulation_intervals; t++)
		{
			m_LinkOutFlowCapacity[t] = GetCapacityPerSimuInterval(link_capacity);
			m_LinkCumulativeArrival[t] = 0;
			m_LinkCumulativeDeparture[t] = 0;
			m_LinkCumulativeVirtualDelay[t] = 0;

		}

		free_flow_travel_time_in_simu_interval = int(free_flow_travel_time_in_min*60.0 / g_number_of_seconds_per_interval + 0.5);
	}

	void ResetMOE()
	{
		m_CumulativeArrivalCount = 0;
		m_CumulativeDepartureCount = 0;
		m_CumulativeVirtualDelayCount = 0;


	}

	void DeallocateMemory()
	{
		//if(m_LinkOutFlowCapacity != NULL) delete m_LinkOutFlowCapacity;
		//if (m_LinkInFlowCapacity != NULL) delete m_LinkInFlowCapacity;
		//if (m_LinkCumulativeArrival != NULL) delete m_LinkCumulativeArrival;
		//if (m_LinkCumulativeDeparture != NULL) delete m_LinkCumulativeDeparture;
		//if (m_LinkTravelTime != NULL) delete m_LinkTravelTime;

	}
	int link_flag;
	int external_link_id;
	int link_seq_no;  // internal seq no
	int from_node_seq_no;
	int to_node_seq_no;
	float cost;
	float free_flow_travel_time_in_min;
	int free_flow_travel_time_in_simu_interval;
	int number_of_lanes;
	bool demand_type_code[_MAX_NUMBER_OF_DEMAND_TYPES];
	float demand_type_TTcost[_MAX_NUMBER_OF_DEMAND_TYPES];

	int type;
	int service_type; // 0: moving, -1: drop off, +1, pick up

	float service_price; // for pick up or drop off
	int VRP_load_id;
	int VRP_group_id;

	int VRP_load_difference; // we use a single point time window now

	int link_capacity;
	float flow_volume;
	float travel_time;
	float BRP_alpha;
	float BRP_beta;
	float length;
	// mfd
	int mfd_zone_id;

	void CalculateBPRFunctionAndCost()
	{
		travel_time = free_flow_travel_time_in_min*(1 + BRP_alpha*pow(flow_volume / max(0.00001, link_capacity), BRP_beta));
		cost = travel_time;
	}

	float get_VOC_ratio()
	{
		return flow_volume / max(0.00001, link_capacity);

	}

	float get_speed()
	{
		return length / max(travel_time, 0.0001) * 60;  // per hour
	}
	// mfd 

	float get_link_in_flow_per_min(int time_in_min)
	{
		if (time_in_min < g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin - 1)
		{
			int next_time_in_internval = (time_in_min + 1) * 60 / g_number_of_seconds_per_interval;
			int time_in_interval = time_in_min * 60 / g_number_of_seconds_per_interval;
			return m_LinkCumulativeArrival[g_A2R_simu_interval(next_time_in_internval)] - m_LinkCumulativeArrival[g_A2R_simu_interval(time_in_interval)];
		}
		else
			return 0;

	}

	float get_link_out_flow_per_min(int time_in_min)
	{
		if (time_in_min < g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin - 1)
		{
			int next_time_in_internval = (time_in_min + 1) * 60 / g_number_of_seconds_per_interval;
			int time_in_interval = time_in_min * 60 / g_number_of_seconds_per_interval;

			return m_LinkCumulativeDeparture[g_A2R_simu_interval(next_time_in_internval)] - m_LinkCumulativeDeparture[g_A2R_simu_interval(time_in_interval)];
		}
		else
			return 0;

	}

	float get_number_of_vehicles(int time_in_min)
	{
		if (time_in_min < g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin)
		{
			int time_in_interval = g_A2R_simu_interval((time_in_min) * 60 / g_number_of_seconds_per_interval);


			return m_LinkCumulativeArrival[time_in_interval] - m_LinkCumulativeDeparture[time_in_interval];
		}
		else
			return 0;

	}


	float get_avg_delay_in_min(int time_in_min, int time_duration)
	{

		if (m_LinkCumulativeVirtualDelay != NULL && time_in_min + time_duration < g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin)
		{
			int time_next_in_interval = g_A2R_simu_interval((time_in_min + time_duration) * 60 / g_number_of_seconds_per_interval);
			int time_in_interval = g_A2R_simu_interval(time_in_min * 60 / g_number_of_seconds_per_interval);
			float total_delay = (m_LinkCumulativeVirtualDelay[time_next_in_interval] - m_LinkCumulativeVirtualDelay[time_in_interval]);
			return total_delay / max(1, get_number_of_vehicles(time_in_min));
		}
		else
		{
			return 0;
		}
	}

};


class CNode
{
public:
	CNode()
	{
		zone_id = 0;
		accessible_node_count = 0;
		bOriginNode_ForAgents = false;
		m_OriginNodeSeqNo = -1;
	}

	int accessible_node_count;

	int node_seq_no;  // sequence number 
	int external_node_id;      //external node number 
	int zone_id;
	float departure_time;
	double x;
	double y;

	int waiting_flag;
	int external_travel_time;
	float waiting_cost;

	bool bOriginNode_ForAgents;
	int m_OriginNodeSeqNo;

	std::vector<CLink> m_outgoing_node_vector;

};

std::vector<CNode> g_node_vector;
std::vector<CLink> g_link_vector;

class CAgent
{
public:
	unsigned int m_RandomSeed;
	bool m_bGenereated;
	CAgent()
	{
		agent_vector_seq_no = -1;
		agent_service_type = 0;  //0: pax vehicle 1: travler 2: scheduled transportation vehicle
		m_bMoveable = 1;
		fixed_path_flag = 0;
		vehicle_seat_capacity = 1;
		m_bGenereated = false;
		PCE_factor = 1.0;
		path_cost = 0;
		m_Veh_LinkArrivalTime_in_simu_interval = NULL;
		m_Veh_LinkDepartureTime_in_simu_interval = NULL;
		m_bCompleteTrip = false;
		departure_time_in_min = 0;
		// vrp

		transportation_time_cost = 0;
		schedule_early_cost = 0;
		schedule_delay_cost = 0;

		earliest_departure_time = 0;
		departure_time_window = 1;

		latest_arrival_time = 0;
		arrival_time_window = 1;

	}
	~CAgent()
	{

		DeallocateMemory();
	}


	float GetRandomRatio()
	{
		m_RandomSeed = (LCG_a * m_RandomSeed + LCG_c) % LCG_M;  //m_RandomSeed is automatically updated.

		return float(m_RandomSeed) / LCG_M;
	}



	int fixed_path_flag;
	int demand_type;
	int agent_id;
	int agent_vector_seq_no;
	int agent_service_type;
	int m_bMoveable;
	int origin_node_id;
	int destination_node_id;

	int origin_zone_seq_no;
	int destination_zone_seq_no;

	float departure_time_in_min;
	int departure_time_in_simu_interval;
	float arrival_time_in_min;
	float PCE_factor;  // passenger car equivalent : bus = 3
	float path_cost;
	std::vector<int> path_link_seq_no_vector;
	std::vector<float> time_seq_no_vector;
	std::vector<int> path_timestamp_vector;;

	int m_path_link_seq_no_vector_size;

	std::vector<int> path_node_id_vector;
	std::vector<int> path_schedule_time_vector;

	int m_current_link_seq_no;
	int* m_Veh_LinkArrivalTime_in_simu_interval;
	int* m_Veh_LinkDepartureTime_in_simu_interval;

	int vehicle_seat_capacity;
	int VRP_group_id;

	std::list<int>  m_PassengerList;

	vector<int> set_of_allowed_links;
	vector<int> m_set_of_allowed_links_flag;
	vector<int> set_of_allowed_nodes;
	vector<int> set_of_allowed_links_LR;
	vector<int> m_set_of_allowed_links_flag_LR;

	// STS
	float transportation_time_cost;
	float schedule_early_cost;
	float schedule_delay_cost;

	float earliest_departure_time;
	int departure_time_in_simulation_interval;
	float departure_time_window;

	float latest_arrival_time;
	float arrival_time_window;
	std::vector<float> time_seq_vector;

	// STS
	void Pickup(int p)
	{
		if (m_PassengerList.size() < vehicle_seat_capacity)
		{
			m_PassengerList.push_back(p);
		}

	}


	int GetRemainingCapacity()
	{

		return vehicle_seat_capacity - m_PassengerList.size();
	}

	//above are simulated 

	bool m_bCompleteTrip;
	bool operator<(const CAgent &other) const
	{
		return departure_time_in_min < other.departure_time_in_min;
	}

	void AllocateMemory()
	{
		if (m_Veh_LinkArrivalTime_in_simu_interval == NULL)
		{
			m_current_link_seq_no = 0;
			m_Veh_LinkArrivalTime_in_simu_interval = new int[path_link_seq_no_vector.size()];
			m_Veh_LinkDepartureTime_in_simu_interval = new int[path_link_seq_no_vector.size()];

			for (int i = 0; i < path_link_seq_no_vector.size(); i++)
			{
				m_Veh_LinkArrivalTime_in_simu_interval[i] = -1;
				m_Veh_LinkDepartureTime_in_simu_interval[i] = -1;

			}


			m_path_link_seq_no_vector_size = path_link_seq_no_vector.size();
			departure_time_in_simu_interval = int(departure_time_in_min*60.0 / g_number_of_seconds_per_interval + 0.5);  // round off

			if (path_link_seq_no_vector.size() > 0)
			{
				m_Veh_LinkArrivalTime_in_simu_interval[0] = departure_time_in_simu_interval;

				int FirstLink = path_link_seq_no_vector[0];

				m_Veh_LinkDepartureTime_in_simu_interval[0] = m_Veh_LinkArrivalTime_in_simu_interval[0] + g_link_vector[FirstLink].free_flow_travel_time_in_simu_interval;

				int relative_simulation_interval = g_A2R_simu_interval(departure_time_in_simu_interval);
				g_link_vector[FirstLink].m_LinkCumulativeArrival[relative_simulation_interval] += 1;
			}
		}
	}

	void DeallocateMemory()
	{
		//if (m_Veh_LinkArrivalTime_in_simu_interval != NULL) delete m_Veh_LinkArrivalTime_in_simu_interval;
		//if (m_Veh_LinkDepartureTime_in_simu_interval != NULL) delete m_Veh_LinkDepartureTime_in_simu_interval;

	}
	std::map<int, int> m_VRP_ADMM_link_time_map;

};

class CActiveAgentElement
{
public:
	CActiveAgentElement()
	{
		pAgent = NULL;
		bActive = true;
	}


	CAgent* pAgent;
	bool bActive;
};

class CFlow
{
public:
	CFlow()
	{};
	int time_index;
	int link_no;
	int cap;
	float wished_inflow_rate;
};
class Cagent_obj
{
public:
	Cagent_obj()
	{};
	int agent_no;
	int wished_depart_time;
};
vector<Cagent_obj> g_agent_obj_vector;
vector<CFlow> g_wished_inflow_vector;
vector<CAgent> g_agent_vector;
std::map<int, int> g_map_agent_id_to_agent_vector_seq_no;

int g_number_of_links = 0;
int g_number_of_nodes = 0;
int g_number_of_zones = 0;

int g_AddNewServiceNode(int load_id, int physical_node_id, float x, float y)
{
	int node_id = load_id;
	std::map<int, int> node_id_map;

	int internal_node_seq_no = g_number_of_nodes;

	if (g_internal_node_seq_no_map.find(node_id) != g_internal_node_seq_no_map.end())
	{
		return -1; //has been defined
	}
	g_internal_node_seq_no_map[node_id] = internal_node_seq_no;
	g_internal_node_seq_no_to_node_id_map[internal_node_seq_no] = node_id;

	CNode node;  // create a node object 

	node.external_node_id = node_id;
	node.node_seq_no = internal_node_seq_no;
	node.zone_id;

	node.x = x;
	node.y = y;

	g_node_vector.push_back(node);  // push it to the global node vector

	g_number_of_nodes++;
	if (g_number_of_nodes % 1000 == 0)
		cout << "reading " << g_number_of_nodes << " nodes.. " << endl;

	return internal_node_seq_no;
}

void g_AddServiceLinksBasedonPaxData(int physical_origin_node,
	int physical_destination_node,
	int service_type,
	int VRP_load_id,
	int VRP_group_id,
	int VRP_load,
	int	VRP_time_window_begin,
	int VRP_time_window_end)
{
	// step 1: add service node node
	int new_service_node = 100000 * VRP_group_id + VRP_load_id;
	int internal_physical_origin_node = g_internal_node_seq_no_map[physical_origin_node];

	float x = g_node_vector[internal_physical_origin_node].x;
	float y = g_node_vector[internal_physical_origin_node].y;
	int internal_service_node_seq_no = g_AddNewServiceNode(new_service_node, physical_origin_node, x, y);

	CLink link;  // create a link object 

	// first link from physical origin to service node 
	link.from_node_seq_no = internal_physical_origin_node;
	link.to_node_seq_no = internal_service_node_seq_no;

	g_node_vector[internal_physical_origin_node].m_outgoing_node_vector.push_back(link);  // add this link to the corresponding node as part of outgoing node/link
	long link_key = internal_physical_origin_node * _MAX_NUMBER_OF_PHYSICAL_NODES + internal_service_node_seq_no;
	link.link_seq_no = g_number_of_links;
	g_link_key_to_seq_no_map[link_key] = link.link_seq_no;

	link.type = 1;
	link.service_type = service_type;
	link.VRP_load_id = VRP_load_id;
	link.VRP_group_id = VRP_group_id;
	link.VRP_time_window_begin = VRP_time_window_begin;
	link.VRP_time_window_end = VRP_time_window_end;
	link.VRP_load_difference = VRP_load;
	link.number_of_lanes = 1;
	link.link_capacity = 1;

	for (int d = 0; d < _MAX_NUMBER_OF_DEMAND_TYPES; d++)
	{
		link.demand_type_code[d] = true;
		link.demand_type_TTcost[d] = 0;
	}

	link.free_flow_travel_time_in_min = 1;
	link.length = 1;
	link.cost = 0; // min // calculate link cost based length and speed limit // later we should also read link_capacity, calculate g_A2R_simu_interval 

	g_link_vector.push_back(link);
	g_number_of_links++;

	// second link from service node back to origin node

	link.from_node_seq_no = internal_service_node_seq_no;
	link.to_node_seq_no = internal_physical_origin_node;

	g_node_vector[internal_service_node_seq_no].m_outgoing_node_vector.push_back(link);  // add this link to the corresponding node as part of outgoing node/link
	link_key = internal_service_node_seq_no * _MAX_NUMBER_OF_PHYSICAL_NODES + internal_physical_origin_node;
	link.link_seq_no = g_number_of_links;
	g_link_key_to_seq_no_map[link_key] = link.link_seq_no;
	link.type = 1;
	link.service_type = 0;
	link.number_of_lanes = 1;
	link.link_capacity = 1;

	for (int d = 0; d < _MAX_NUMBER_OF_DEMAND_TYPES; d++)
	{
		link.demand_type_code[d] = true;
		link.demand_type_TTcost[d] = 0;
	}

	link.free_flow_travel_time_in_min = 1;
	link.length = 1;
	link.cost = 0; // min // calculate link cost based length and speed limit // later we should also read link_capacity, calculate g_A2R_simu_interval 

	g_link_vector.push_back(link);
	g_number_of_links++;


	if (g_number_of_links % 1000 == 0)
		cout << "creating " << g_number_of_links << " links.. " << endl;

}
int time_max = 0;
int total_arrival = 0;
void g_ReadInputData()
{
	g_number_of_nodes = 0;
	g_number_of_links = 0;  // initialize  the counter to 0

	int internal_node_seq_no = 0;
	double x, y;
	// step 1: read node file 
	CCSVParser parser;
	if (parser.OpenCSVFile("input_node.csv", true))
	{
		std::map<int, int> node_id_map;

		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{

			string name;

			int node_type;
			int node_id;

			if (parser.GetValueByFieldName("node_id", node_id) == false)
				continue;

			if (g_internal_node_seq_no_map.find(node_id) != g_internal_node_seq_no_map.end())
			{
				continue; //has been defined
			}
			g_internal_node_seq_no_map[node_id] = internal_node_seq_no;
			g_internal_node_seq_no_to_node_id_map[internal_node_seq_no] = node_id;

			parser.GetValueByFieldName("x", x, false);
			parser.GetValueByFieldName("y", y, false);


			CNode node;  // create a node object 

			node.external_node_id = node_id;
			node.node_seq_no = internal_node_seq_no;
			parser.GetValueByFieldName("zone_id", node.zone_id);
			node.departure_time = 0;
			node.x = x;
			node.y = y;
			internal_node_seq_no++;
			parser.GetValueByFieldName("waiting_flag", node.waiting_flag);
			if (node.waiting_flag == 1)
			{
				parser.GetValueByFieldName("external_travel_time", node.external_travel_time);
				parser.GetValueByFieldName("waiting_cost", node.waiting_cost);
			}

			g_node_vector.push_back(node);  // push it to the global node vector

			g_number_of_nodes++;
			if (g_number_of_nodes % 1000 == 0)
				cout << "reading " << g_number_of_nodes << " nodes.. " << endl;
		}

		cout << "number of nodes = " << g_number_of_nodes << endl;

		fprintf(g_pFileOutputLog, "number of nodes =,%d\n", g_number_of_nodes);
		parser.CloseCSVFile();
	}
	else
	{
		cout << "input_node.csv is not opened." << endl;
		g_ProgramStop();
	}

	// step 2: read link file 

	CCSVParser parser_link;

	if (parser_link.OpenCSVFile("input_link.csv", true))
	{
		while (parser_link.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int from_node_id = 0;
			int to_node_id = 0;
			if (parser_link.GetValueByFieldName("from_node_id", from_node_id) == false)
				continue;
			if (parser_link.GetValueByFieldName("to_node_id", to_node_id) == false)
				continue;

			// add the to node id into the outbound (adjacent) node list

			int internal_from_node_seq_no = g_internal_node_seq_no_map[from_node_id];  // map external node number to internal node seq no. 
			int internal_to_node_seq_no = g_internal_node_seq_no_map[to_node_id];

			CLink link;  // create a link object 

			parser_link.GetValueByFieldName("link_id", link.external_link_id);
			parser_link.GetValueByFieldName("link_cost", link.link_cost);

			link.from_node_seq_no = internal_from_node_seq_no;
			link.to_node_seq_no = internal_to_node_seq_no;
			link.link_seq_no = g_number_of_links;
			//link.to_node_seq_no = internal_to_node_seq_no;
			link.LR_multiplier = 0;
			float m_End_of_RedTime_in_min = 0;
			parser_link.GetValueByFieldName("m_End_of_RedTime_in_min", m_End_of_RedTime_in_min);
			link.m_End_of_RedTime_in_simu_interval = m_End_of_RedTime_in_min * 60 / g_number_of_seconds_per_interval;

			parser_link.GetValueByFieldName("link_flag", link.link_flag);
			parser_link.GetValueByFieldName("link_type", link.type);
			parser_link.GetValueByFieldName("service_type", link.service_type, false);

			if (link.service_type != 0)
			{
				parser_link.GetValueByFieldName("VRP_load_id", link.VRP_load_id, false);
				parser_link.GetValueByFieldName("VRP_group_id", link.VRP_group_id, false);
				parser_link.GetValueByFieldName("VRP_time_window_begin", link.VRP_time_window_begin, false);
				parser_link.GetValueByFieldName("VRP_time_window_end", link.VRP_time_window_end, false);
				parser_link.GetValueByFieldName("VRP_load_difference", link.VRP_load_difference, false);
			}
			float length = 1; // km or mile
			float speed_limit = 1;
			parser_link.GetValueByFieldName("length", length);
			parser_link.GetValueByFieldName("speed_limit", speed_limit);
			parser_link.GetValueByFieldName("base_price", link.base_price);
			parser_link.GetValueByFieldName("BPR_alpha_term", link.BRP_alpha);
			parser_link.GetValueByFieldName("BPR_beta_term", link.BRP_beta);

			int number_of_lanes = 1;
			float lane_cap = 1000;
			parser_link.GetValueByFieldName("number_of_lanes", link.number_of_lanes);
			parser_link.GetValueByFieldName("lane_cap", lane_cap);

			parser_link.GetValueByFieldName("link_cap", link.link_capacity);

			//link.link_capacity = lane_cap* number_of_lanes;

			string demand_type_code;
			for (int d = 0; d < _MAX_NUMBER_OF_DEMAND_TYPES; d++)
			{
				link.demand_type_code[d] = true;
				link.demand_type_TTcost[d] = 0;
			}

			parser_link.GetValueByFieldName("demand_type_code", demand_type_code);

			if (demand_type_code.size() > 0)  //demand type code has a string
			{
				for (int d = 0; d < _MAX_NUMBER_OF_DEMAND_TYPES; d++)
				{
					link.demand_type_code[d] = false;
					CString demand_type_number;
					demand_type_number.Format(_T("%d"), d);

					std::string str_number = CString2StdString(demand_type_number);

					if (demand_type_code.find(str_number) != std::string::npos)   // find this number
					{
						link.demand_type_code[d] = true;  // allow this demand type
					}
				}
			}


			for (int d = 0; d < _MAX_NUMBER_OF_DEMAND_TYPES; d++)
			{
				CString demand_type_number;
				demand_type_number.Format(_T("demand_type_%d_TTcost"), d);

				std::string str_number = CString2StdString(demand_type_number);
				parser_link.GetValueByFieldName(str_number, link.demand_type_TTcost[d]);

			}
			for (int t = 0;t < time_index;t++)
			{
				float link_cost;
				parser_link.GetValueByFieldName("link_cost", link_cost);
				link.time_dependent_link_cost.push_back(link_cost);

				int link_travel_time;
				parser_link.GetValueByFieldName("travel_time", link_travel_time);
				link.time_dependent_travel_time_vector.push_back(link_travel_time);

				float LR_multiplier = 0;
				link.time_dependent_LR_multiplier_vector.push_back(LR_multiplier);

				link.time_dependent_discharge_rate.push_back(link.link_capacity);
			}

			parser_link.GetValueByFieldName("link_cost", link.link_cost);
			parser_link.GetValueByFieldName("travel_time", link.travel_time);
			parser_link.GetValueByFieldName("link_cap", link.link_capacity);

			

			link.free_flow_travel_time_in_min = length / speed_limit * 60;

			float external_travel_time = -1;
			parser_link.GetValueByFieldName("external_travel_time", external_travel_time, false);

			if (external_travel_time >= 0.1)
			{  // reset 
				link.free_flow_travel_time_in_min = external_travel_time;
			}

			link.length = length;
			link.cost = length / speed_limit * 60; // min // calculate link cost based length and speed limit // later we should also read link_capacity, calculate g_A2R_simu_interval 

			g_node_vector[internal_from_node_seq_no].m_outgoing_node_vector.push_back(link);  // add this link to the corresponding node as part of outgoing node/link

			long link_key = internal_from_node_seq_no * _MAX_NUMBER_OF_PHYSICAL_NODES + internal_to_node_seq_no;

			g_link_key_to_seq_no_map[link_key] = link.link_seq_no;

			link.CalculateBPRFunctionAndCost(); // initial link travel time value
			g_link_vector.push_back(link);
			g_number_of_links++;

			if (g_number_of_links % 1000 == 0)
				cout << "reading " << g_number_of_links << " links.. " << endl;
		}
	}
	else
	{
		cout << "input_link.csv is not opened." << endl;
		g_ProgramStop();
	}


	cout << "number of links = " << g_number_of_links << endl;

	fprintf(g_pFileOutputLog, "number of links =,%d\n", g_number_of_links);

	parser_link.CloseCSVFile();

	g_number_of_agents = 0;
	CCSVParser parser_agent;
	std::vector<int> path_node_sequence;
	string path_node_sequence_str;

	std::vector<int> path_schedule_time_sequence;
	string path_schedule_time_sequence_str;

	if (parser_agent.OpenCSVFile("input_agent.csv", true))   // read agent as demand input 
	{
		while (parser_agent.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			CAgent agent;  // create an agent object 
			if (parser_agent.GetValueByFieldName("agent_id", agent.agent_id) == false)
				continue;

			parser_agent.GetValueByFieldName("agent_service_type", agent.agent_service_type);

			agent.m_RandomSeed = agent.agent_id;

			int origin_node_id = 0;
			int destination_node_id = 0;
			parser_agent.GetValueByFieldName("from_origin_node_id", origin_node_id, false);

			agent.origin_node_id = origin_node_id;
			parser_agent.GetValueByFieldName("to_destination_node_id", destination_node_id, false);
			agent.destination_node_id = destination_node_id;

			if (g_internal_node_seq_no_map.find(origin_node_id) == g_internal_node_seq_no_map.end() || g_internal_node_seq_no_map.find(destination_node_id) == g_internal_node_seq_no_map.end())
				continue;

			parser_agent.GetValueByFieldName("from_zone_id", agent.origin_zone_seq_no, false);
			parser_agent.GetValueByFieldName("to_zone_id", agent.destination_zone_seq_no, false);

			parser_agent.GetValueByFieldName("demand_type", agent.demand_type, false);

			if (agent.demand_type > g_number_of_demand_types)
				g_number_of_demand_types = agent.demand_type;

			ASSERT(g_number_of_demand_types + 1 < _MAX_NUMBER_OF_DEMAND_TYPES);

			parser_agent.GetValueByFieldName("VRP_group_id", agent.VRP_group_id);
			parser_agent.GetValueByFieldName("vehicle_seat_capacity", agent.vehicle_seat_capacity);
			parser_agent.GetValueByFieldName("fixed_path_flag", agent.fixed_path_flag);
			parser_agent.GetValueByFieldName("departure_time_in_min", agent.departure_time_in_min);
			parser_agent.GetValueByFieldName("arrival_time_in_min", agent.arrival_time_in_min);

			if (agent.departure_time_in_min < g_Simulation_StartTimeInMin)
				g_Simulation_StartTimeInMin = agent.departure_time_in_min;

			if (agent.departure_time_in_min > g_Simulation_EndTimeInMin)
				g_Simulation_EndTimeInMin = agent.departure_time_in_min;

			parser_agent.GetValueByFieldName("PCE", agent.PCE_factor);
			parser_agent.GetValueByFieldName("path_node_sequence", path_node_sequence_str);

			agent.path_node_id_vector = ParseLineToIntegers(path_node_sequence_str);

			parser_agent.GetValueByFieldName("path_schdule_time_sequence", path_schedule_time_sequence_str);

			agent.path_schedule_time_vector = ParseLineToIntegers(path_schedule_time_sequence_str);

			if (agent.path_node_id_vector.size() >= 2)
			{
				for (int n = 0; n < agent.path_node_id_vector.size() - 1; n++)
				{
					int link_seq_no = g_GetLinkSeqNo(agent.path_node_id_vector[n], agent.path_node_id_vector[n + 1]);

					if (link_seq_no == -1)
					{
						// trace: error
						break;
					}

					agent.path_link_seq_no_vector.push_back(link_seq_no);
				}

			}
			//To Do 1: load agent path from field path_node_sequence
			// initial loading multiplier: 0.66666

			if (agent.agent_service_type == 2)
			{
				parser_agent.GetValueByFieldName("earliest_departure_time", agent.earliest_departure_time);
				parser_agent.GetValueByFieldName("departure_time_window", agent.departure_time_window);
				parser_agent.GetValueByFieldName("latest_arrival_time", agent.latest_arrival_time);
				parser_agent.GetValueByFieldName("arrival_time_window", agent.arrival_time_window);
			}

			parser_agent.GetValueByFieldName("transportation_time_cost", agent.transportation_time_cost);
			parser_agent.GetValueByFieldName("schedule_early_cost", agent.schedule_early_cost);
			parser_agent.GetValueByFieldName("schedule_delay_cost", agent.schedule_delay_cost);

			agent.departure_time_in_simulation_interval = agent.earliest_departure_time / g_number_of_seconds_per_interval;  // covert departure time in min to an integer value of simulation time intervals

			if (agent.latest_arrival_time >= g_number_of_optimization_time_intervals)
				g_number_of_optimization_time_intervals = agent.latest_arrival_time + 1;

			g_agent_vector.push_back(agent);
			g_number_of_agents++;
			if (g_number_of_agents % 1000 == 0)
				cout << "reading = " << g_number_of_agents / 1000 << " k agents..." << endl;

		}
	}

	cout << "number of agents = " << g_agent_vector.size() << endl;

	cout << " Sort agents... " << endl;
	std::sort(g_agent_vector.begin(), g_agent_vector.end());

	// simulation
	for (int a = 0; a < g_agent_vector.size(); a++)
	{
		g_agent_vector[a].agent_vector_seq_no = a;

		g_map_agent_id_to_agent_vector_seq_no[g_agent_vector[a].agent_id] = a;// based on agent_id to find agent_vector
	}
	cout << " Sorting ends. ..." << endl;

	// use absoluate time scale
	g_start_simu_interval_no = g_Simulation_StartTimeInMin * 60 / g_number_of_seconds_per_interval;
	g_end_simu_interval_no = g_start_simu_interval_no + g_number_of_simulation_intervals;

	parser_agent.CloseCSVFile();

	CCSVParser parser_cumulative;
	if (parser_cumulative.OpenCSVFile("input_wish.csv", true))   // read agent as demand input 
	{
		while (parser_cumulative.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			Cagent_obj agent_obj;
			parser_cumulative.GetValueByFieldName("agent_no", agent_obj.agent_no);
			parser_cumulative.GetValueByFieldName("wished_depart_time", agent_obj.wished_depart_time);
			g_agent_obj_vector.push_back(agent_obj);
		}
		// get information
		int t_max = g_agent_obj_vector[g_agent_obj_vector.size()-1].wished_depart_time;
		for (int t = 0;t < t_max;t++)
		{
			int i = 0;
			for (int j = 0;j < g_agent_obj_vector.size();j++)
			{
				if (g_agent_obj_vector[j].wished_depart_time == t)
					i++;
			}
			CFlow flow;
			flow.time_index = t;
			flow.wished_inflow_rate = i;
			flow.link_no = 1;
			flow.cap = 2;
			g_wished_inflow_vector.push_back(flow);
		}
		for (int t = t_max;t < 2*t_max+1;t++)
		{
			CFlow flow;
			flow.wished_inflow_rate = 0;
			flow.time_index = t;
			flow.cap = 2;
			flow.link_no = 1;
			g_wished_inflow_vector.push_back(flow);
		}

	}
	parser_cumulative.CloseCSVFile();

}
std::vector<float> departure_time_vector;
int node_no_locate(int node_no)
{
	for (int i = 0;i < g_node_vector.size();i++)
	{
		if (g_node_vector[i].node_seq_no == node_no)
		{
			return i;
			break;
		}
	}
};
class NetworkForSP  // mainly for shortest path calculation
{
public:
	int m_threadNo;  // internal thread number 

	std::list<int>  m_SENodeList;  //scan eligible list as part of label correcting algorithm 

	float** m_node_label_cost; // label cost
	int** m_node_time_departure_flag; 
	int** m_vertex_node_predecessor;
	int** m_vertex_time_predecessor;
	int** m_vertex_link_predecessor;

	int* m_node_predecessor;  // predecessor for nodes
	int* m_time_of_node;
	int* m_time_predecessor;
	int* m_node_status_array; // update status 
	float** new_to_node_cost;
	int* m_link_predecessor;  // predecessor for this node points to the previous link that updates its label cost (as part of optimality condition) (for easy referencing)

	FILE* pFileAgentPathLog;  // file output

	float** m_link_volume_array; // link volume for all agents assigned in this network (thread)
	float** m_link_cost_array; // link cost 


	int m_private_origin_seq_no;
	std::vector<int>  m_agent_vector; // assigned agents for computing 
	std::vector<int>  m_node_vector; // assigned nodes for computing 

	NetworkForSP()
	{
		pFileAgentPathLog = NULL;
		m_private_origin_seq_no = -1;

	}

	void AllocateMemory(int number_of_nodes, int number_of_links, int time_index)
	{
		m_node_label_cost = new float*[number_of_nodes];
		m_link_cost_array = new float*[number_of_links];
		new_to_node_cost = new float*[number_of_nodes];
		m_node_time_departure_flag = new int*[number_of_nodes];
		m_vertex_node_predecessor = new int*[number_of_nodes];
		m_vertex_time_predecessor = new int*[number_of_nodes];
		m_vertex_link_predecessor = new int*[number_of_nodes];

		for (int j = 0;j < number_of_nodes;j++)
		{
			m_node_label_cost[j] = new float[time_index];
			new_to_node_cost[j] = new float[time_index];
			m_node_time_departure_flag[j] = new int[time_index];
			m_vertex_node_predecessor[j] = new int[time_index];
			m_vertex_time_predecessor[j] = new int[time_index];
			m_vertex_link_predecessor[j] = new int[time_index];

		}
	}

	~NetworkForSP()
	{

		if (m_node_label_cost != NULL)
			delete m_node_label_cost;

		if (m_node_predecessor != NULL)
			delete m_node_predecessor;

		if (m_node_status_array != NULL)
			delete m_node_status_array;

		if (m_link_predecessor != NULL)
			delete m_link_predecessor;

		if (m_link_volume_array != NULL)
			delete m_link_volume_array;

		if (m_link_cost_array != NULL)
			delete m_link_cost_array;

		if (pFileAgentPathLog != NULL)
			fclose(pFileAgentPathLog);
	}

	// SEList: scan eligible List implementation: the reason for not using STL-like template is to avoid overhead associated pointer allocation/deallocation
	void SEList_clear()
	{
		m_SENodeList.clear();
	}

	void SEList_push_front(int node)
	{
		m_SENodeList.push_front(node);
	}

	void SEList_push_back(int node)
	{
		m_SENodeList.push_back(node);
	}

	bool SEList_empty()
	{
		return m_SENodeList.empty();
	}

	int SEList_front()
	{
		return m_SENodeList.front();
	}

	void SEList_pop_front()
	{
		m_SENodeList.pop_front();
	}


	int optimal_label_correcting(int origin_node, int destination_node, int departure_time, int arrival_time, int agent_id)
		// time-dependent label correcting algorithm with double queue implementation
	{
		int internal_debug_flag = 0;

		if (g_node_vector[origin_node].m_outgoing_node_vector.size() == 0)
		{
			return 0;
		}

		for (int i = 0; i < g_number_of_nodes; i++) //Initialization for all nodes
		{
										 //by slt
			for (int t = 0;t < time_index;t++)
			{
				m_node_label_cost[i][t] = _MAX_LABEL_COST;
				m_node_time_departure_flag[i][t] = -1;
				m_vertex_time_predecessor[i][t] = -1;
				m_vertex_node_predecessor[i][t] = -1;
				m_vertex_link_predecessor[i][t] = -1;
			}
		}


		//Initialization for origin node at the preferred departure time, at departure time, cost = 0, otherwise, the g_A2R_simu_interval at origin node
		
		m_node_label_cost[origin_node][departure_time] = 0;
		// initialization
		m_node_time_departure_flag[origin_node][departure_time] = 1;//initialization
		for (int t = departure_time;t < arrival_time;t++)
		{
			for (int i = 0;i < g_node_vector.size();i++)
			{
				if (m_node_time_departure_flag[i][t] == 1)// for each available space-time vertex
				{
					for (int j = 0; j < g_node_vector[i].m_outgoing_node_vector.size();j++)
					{
						int current_travel_time = g_node_vector[i].m_outgoing_node_vector[j].travel_time;
						int next_time = t + current_travel_time;
						if (next_time <= arrival_time)// cannot exceed final ST vertex
						{
							int current_link = g_node_vector[i].m_outgoing_node_vector[j].link_seq_no;
							float current_cost = g_link_vector[current_link].time_dependent_link_cost[t];
							int to_node = g_node_vector[i].m_outgoing_node_vector[j].to_node_seq_no;
							float new_cost = m_node_label_cost[i][t] + current_cost;
							if (new_cost < m_node_label_cost[to_node][next_time])
							{
								m_node_label_cost[to_node][next_time] = new_cost;
								m_vertex_node_predecessor[to_node][next_time] = i;
								m_vertex_time_predecessor[to_node][next_time] = t;
								m_vertex_link_predecessor[to_node][next_time] = current_link;
								m_node_time_departure_flag[to_node][next_time] = 1;
							}
						}
					}
				}
			}
		}
		

		CAgent* p_agent = &(g_agent_vector[agent_id]);
		p_agent->path_link_seq_no_vector.clear();  // reset;
		p_agent->path_node_id_vector.clear();  // reset;
		p_agent->time_seq_no_vector.clear();

		int t = arrival_time-1;
		int i = destination_node;
		p_agent->time_seq_no_vector.push_back(t);
		p_agent->path_node_id_vector.push_back(i);
		while (m_vertex_time_predecessor[i][t] != departure_time)
		{
			cout << m_vertex_time_predecessor[i][t] <<  m_vertex_node_predecessor[i][t]  << endl;
			int l = m_vertex_link_predecessor[i][t];
			p_agent->path_link_seq_no_vector.push_back(l);

			int node = m_vertex_node_predecessor[i][t];
			p_agent->path_node_id_vector.push_back(node);

			int time = m_vertex_time_predecessor[i][t];
			p_agent->time_seq_no_vector.push_back(time);

			t = time;
			i = node;
		}

		p_agent->path_node_id_vector.push_back(origin_node);
		p_agent->time_seq_no_vector.push_back(departure_time);
		p_agent->path_link_seq_no_vector.push_back(m_vertex_link_predecessor[i][t]);
				if (p_agent->fixed_path_flag != 1)
				{
					std::reverse(std::begin(p_agent->path_node_id_vector),
						std::end(p_agent->path_node_id_vector));
					std::reverse(std::begin(p_agent->path_link_seq_no_vector),
						std::end(p_agent->path_link_seq_no_vector));
					std::reverse(std::begin(p_agent->time_seq_no_vector),
						std::end(p_agent->time_seq_no_vector));
				}
				 if (destination_node == -1)
					return 1;  // one to all shortest path
				else
					return -1;


	}


	// rewrite label correcting(or DP)

};
// definte global timestamps
clock_t start_t, end_t, total_t;
int g_state_to_load_mapping[_MAX_STATES];
int g_initial_state_no = 0;   // customized 
//class for vehicle scheduling states
int g_number_of_LR_iterations = 20;
int g_number_of_ADMM_iterations = 100;
int g_CurrentLRIterationNumber = 0;
int g_Number_Of_Iterations_With_Memory = 5;

float g_best_upper_bound = 99999;
float g_best_lower_bound = -99999;
float g_stepSize = 0;
float g_penalty_PHO = 0.1;
float g_minimum_subgradient_step_size = 0.1;

NetworkForSP* g_pNetworkForSP = NULL;
float k_critical = 2;
float k_jam = 6;

class CD_information

{
public:
	int pred_CD_positon;
	int current_no;
	int queue_length;
	int cost;
	int CD_value;
};
std::vector<CD_information> CD_vector;
class CD
{
public:
	std::vector<CD_information> m_CD_vector;
};
class CA_information
{
public:
	int pred_CA_position;
	int current_no;
	int CA_value;
	std::vector<CD_information> m_CD_vector;
};
class CA
{
public:
	std::vector<CA_information> m_CA_vector;
};
void Cumulative_Curve_Based_DP_3_dimension()
{
	//initialization
	std::vector<int>link_no_sequence;
	link_no_sequence.push_back(1);
	link_no_sequence.push_back(4);
	int max_inflow = 4;
	int max_cap = 2;
	int no_of_vehicle = 12;
	std::vector<CA> cumulative_vector;
	for (int t = 0;t < g_wished_inflow_vector.size() + 1;t++)
	{
		CA CA;
		if (t == 0)
		{
			CA_information CA_information;
			CA.m_CA_vector.push_back(CA_information);
			CA.m_CA_vector[t].CA_value = 0;
			CA.m_CA_vector[t].current_no = 0;
			CD_information CD_information;
			CA.m_CA_vector[t].m_CD_vector.push_back(CD_information);
			CA.m_CA_vector[t].m_CD_vector[t].CD_value = 0;
			CA.m_CA_vector[t].m_CD_vector[t].cost = 0;
			CA.m_CA_vector[t].m_CD_vector[t].current_no = 0;
			CA.m_CA_vector[t].m_CD_vector[t].pred_CD_positon = 0;
			CA.m_CA_vector[t].m_CD_vector[t].queue_length = 0;
		}
		cumulative_vector.push_back(CA);
	}
	
	for (int t = 1;t < g_wished_inflow_vector.size();t++)// for each t
	{
		//for each last CA
		for (int CA_size = 0; CA_size < cumulative_vector[t-1].m_CA_vector.size(); CA_size++)
		{
			// for each inflow rate
			for (int current_inflow = 1;current_inflow < max_inflow + 1; current_inflow++)
			{
				int next_A = min(cumulative_vector[t - 1].m_CA_vector[CA_size].CA_value + current_inflow, no_of_vehicle);
				int update_CA_flag = 0;
				for (int m_CA_size = 0; m_CA_size < cumulative_vector[t].m_CA_vector.size();m_CA_size++)
				{
					if (next_A == cumulative_vector[t].m_CA_vector[m_CA_size].CA_value)
					{
						update_CA_flag++;
					}
				}
				// if there is a CA with same CA_value(flag=1), update CD and cost. Else add a new CA_object(flag=0).
				if (update_CA_flag == 0)
				{
					CA_information CA_information;
					CA_information.CA_value = next_A;
					CA_information.current_no = cumulative_vector[t].m_CA_vector.size();
					CA_information.pred_CA_position = cumulative_vector[t - 1].m_CA_vector[CA_size].current_no;
					
					cumulative_vector[t].m_CA_vector.push_back(CA_information);
				}
				// for all last CDs
				for (int CA = 0;CA < cumulative_vector[t - 1].m_CA_vector.size();CA++)
				{
					for (int CD_size = 0;CD_size < cumulative_vector[t - 1].m_CA_vector[CA_size].m_CD_vector.size();CD_size++)
					{
						// for each outflow rate
						for (int current_outflow = 1;current_outflow < max_cap + 1;current_outflow++)
						{
							int next_D = min(no_of_vehicle, cumulative_vector[t - 1].m_CA_vector[CA_size].m_CD_vector[CD_size].CD_value + current_outflow);
							int current_queue = max(0, next_A - next_D);
							int schedule_delay = 0;
							if (g_wished_inflow_vector[t].wished_inflow_rate != 0)
							{
								schedule_delay = abs(current_inflow - g_wished_inflow_vector[t].wished_inflow_rate);
							}
							else
							{
								schedule_delay = max_cap;
							}
							int current_cost = current_queue + cumulative_vector[t - 1].m_CA_vector[CA_size].m_CD_vector[CD_size].cost + schedule_delay;

							int update_CD_flag = 0;
							for (int m_CD_size = 0;m_CD_size < cumulative_vector[t].m_CA_vector[cumulative_vector[t].m_CA_vector.size() - 1].m_CD_vector.size();m_CD_size++)
							{
								if (next_D == cumulative_vector[t].m_CA_vector[cumulative_vector[t].m_CA_vector.size()-1].m_CD_vector[m_CD_size].CD_value)
								{
									update_CD_flag++;
									// update labal
									if (current_cost < cumulative_vector[t].m_CA_vector[cumulative_vector[t].m_CA_vector.size() - 1].m_CD_vector[m_CD_size].cost)
									{
										cumulative_vector[t].m_CA_vector[cumulative_vector[t].m_CA_vector.size() - 1].m_CD_vector[m_CD_size].cost = current_cost;
										cumulative_vector[t].m_CA_vector[cumulative_vector[t].m_CA_vector.size() - 1].m_CD_vector[m_CD_size].pred_CD_positon = cumulative_vector[t - 1].m_CA_vector[CA_size].m_CD_vector[CD_size].current_no;
										cumulative_vector[t].m_CA_vector[cumulative_vector[t].m_CA_vector.size() - 1].m_CD_vector[m_CD_size].queue_length = current_queue;
									}
								}
							}
							// new CD
							if (update_CD_flag == 0)
							{
								CD_information CD_information;
								CD_information.CD_value = next_D;
								CD_information.current_no = cumulative_vector[t].m_CA_vector[CA_size].m_CD_vector.size();
								CD_information.cost = current_cost;
								CD_information.queue_length = current_queue;
								CD_information.pred_CD_positon = cumulative_vector[t - 1].m_CA_vector[CA_size].m_CD_vector[CD_size].current_no;
								cumulative_vector[t].m_CA_vector[cumulative_vector[t].m_CA_vector.size() - 1].m_CD_vector.push_back(CD_information);
							}
						}
					}

				}
			}
		}
	}
	/*(
	// trace back
	int minimum_cost = 100;
	int minimum_no;
	int pred_ca;
	int minimum_t;
	for (int t = g_wished_inflow_vector.size();t >= 0;t--)
	{
		for (int i = 0;i < cumulative_vector[t].m_CA_vector.size();i++)
		{
			if (cumulative_vector[t].m_CA_vector[i].cost < minimum_cost && cumulative_vector[t].m_CA_vector[i].CD_value == no_of_vehicle)
			{
				minimum_cost = cumulative_vector[t].m_CA_vector[i].cost;
				minimum_no = cumulative_vector[t].m_CA_vector[i].current_no;
				pred_ca = cumulative_vector[t].m_CA_vector[i].pred_CA_positon;
				minimum_t = t;
			}
		}
	}
	while (minimum_t != 0)
	{
		CA_vector.push_back(cumulative_vector[minimum_t].m_CA_vector[minimum_no]);
		minimum_no = cumulative_vector[minimum_t].m_CA_vector[minimum_no].pred_CA_positon;
		minimum_t--;
	}

	std::reverse(std::begin(CA_vector),
		std::end(CA_vector));
	//output_cumulative_files(link_no_sequence[link_no]);
	CA_vector.clear();
	*/
	
}

float*** g_Label_Cost = NULL;
int***   g_PredecessorCA = NULL;
int***   g_PredecessorCD = NULL;
int***   g_PredecessorTime = NULL;
int***    g_flag = NULL;
int*** current_queue = NULL;

int main(int argc, TCHAR* argv[], TCHAR* envp[])
{
	g_pFileDebugLog = fopen("Debug.txt", "w");
	if (g_pFileDebugLog == NULL)
	{
		cout << "File Debug.txt cannot be opened." << endl;
		g_ProgramStop();
	}
	g_pFileOutputLog = fopen("output_solution.csv", "w");
	if (g_pFileOutputLog == NULL)
	{
		cout << "File output_solution.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	g_ReadInputData();  // step 1: read input data of network and demand agent 

	Cumulative_Curve_Based_DP_3_dimension();

	cout << "it is done!" << endl;

	return 1;
	
}

